"""Real GICP with synthetic PCD/TF on a dedicated localhost ROS domain."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('CHAIRMAN_GICP_INTEGRATION') != '1',
    reason='Requires built GICP and isolated localhost ROS domain 197')


@pytest.fixture
def scene(tmp_path):
    import rclpy
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import PointCloud2
    from tf2_msgs.msg import TFMessage
    from tf2_ros import StaticTransformBroadcaster

    assert os.environ.get('ROS_DOMAIN_ID') == '197'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    executable = Path(os.environ['CHAIRMAN_GICP_EXECUTABLE'])
    points = np.random.default_rng(42).uniform([-2., -1.5, -0.2], [2., 1.5, 1.4], size=(1000, 3))
    pcd = tmp_path / 'map.pcd'
    with pcd.open('w') as stream:
        stream.write('# chairman_lidar_voxel_size 0.05\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                     f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n')
        np.savetxt(stream, points, fmt='%.8f')
    rclpy.init()
    node = rclpy.create_node('gicp_test_observer')
    received = []

    def record(message):
        for transform in message.transforms:
            if transform.header.frame_id == 'gicp_map' and transform.child_frame_id == 'gicp_odom':
                received.append((transform, node.get_clock().now().nanoseconds))

    node.create_subscription(TFMessage, '/tf', record, 100)
    cloud_pub = node.create_publisher(PointCloud2, '/gicp_test_cloud', 10)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    static = StaticTransformBroadcaster(node)
    log_path = tmp_path / 'gicp.log'
    log = log_path.open('w')
    process = subprocess.Popen([
        str(executable), '--ros-args',
        '-p', f'prior_pcd_file:={pcd}',
        '-p', 'initial_map_pose:=[4.0, 6.0, 0.0, 0.0, 0.0, 1.5707963267948966]',
        '-p', 'map_to_pcd:=[10.0, -2.0, 0.0, 0.0, 0.0, 1.5707963267948966]',
        '-p', 'map_frame:=gicp_map', '-p', 'odom_frame:=gicp_odom',
        '-p', 'robot_base_frame:=gicp_base', '-p', 'input_cloud_topic:=/gicp_test_cloud',
        '-p', 'num_threads:=2', '-p', 'num_neighbors:=10', '-p', 'max_dist_sq:=1.0',
        '-p', 'time_offset_sec:=0.1',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def until(condition, publish=None, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not condition():
            assert process.poll() is None, log_path.read_text()
            assert time.monotonic() < deadline, log_path.read_text()
            if publish:
                publish()
            rclpy.spin_once(node, timeout_sec=0.02)

    def advance(seconds):
        deadline = time.monotonic() + seconds
        until(lambda: time.monotonic() >= deadline)

    def publish_local_tf():
        body = TransformStamped()
        body.header.frame_id = 'gicp_odom'
        body.child_frame_id = 'gicp_base'
        body.transform.translation.x = -0.25
        body.transform.rotation.w = 1.0
        sensor = TransformStamped()
        sensor.header.frame_id = 'gicp_odom'
        sensor.child_frame_id = 'gicp_sensor_odom'
        sensor.transform.translation.x = 0.3
        sensor.transform.translation.y = -0.2
        sensor.transform.rotation.z = math.sin(-0.1 / 2)
        sensor.transform.rotation.w = math.cos(-0.1 / 2)
        static.sendTransform([body, sensor])

    try:
        yield node, received, points, cloud_pub, pose_pub, publish_local_tf, until, advance, log_path
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        log.close()


def test_initial_tf_waits_for_local_odometry_and_handles_manual_reinitialization(scene):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    node, received, _, _, pose_pub, local_tf, until, advance, log_path = scene
    advance(0.4)
    assert not received, 'No map -> odom may be invented before local TF exists'
    local_tf()
    until(lambda: len(received) >= 3)
    transform = received[-1][0].transform
    assert transform.translation.x == pytest.approx(4.0)
    assert transform.translation.y == pytest.approx(6.25)
    assert transform.rotation.z == pytest.approx(math.sqrt(0.5))
    assert '[定位就绪]' not in log_path.read_text()

    for x, y, yaw in [(-5., -1., -math.pi/2), (6., -4., math.pi/2),
                      (-5., 1., math.pi/2), (6., 4., -math.pi/2)]:
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'gicp_map'
        pose.pose.pose.position.x, pose.pose.pose.position.y = x, y
        pose.pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.pose.orientation.w = math.cos(yaw / 2)
        pose_pub.publish(pose)
        expected_y = y + (0.25 if yaw > 0 else -0.25)
        until(lambda: abs(received[-1][0].transform.translation.y - expected_y) < 1e-6)
        assert received[-1][0].transform.translation.x == pytest.approx(x)
    last = received[-1][0].transform
    pose.header.frame_id = 'wrong_frame'
    pose.pose.pose.position.x = 99.0
    pose_pub.publish(pose)
    advance(0.3)
    assert received[-1][0].transform == last


def test_gicp_updates_map_tf_and_keeps_last_result_on_invalid_matching(scene):
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    node, received, points, cloud_pub, _, local_tf, until, advance, log_path = scene
    local_tf()
    until(lambda: len(received) >= 3)

    def rotate(yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])

    # PCD <- odom is (8.45, 5.9, 0.04 rad); input cloud has a separate frame.
    odom_points = (points - [8.45, 5.9, 0.]) @ rotate(0.04)
    cloud_points = (odom_points - [0.3, -0.2, 0.]) @ rotate(-0.1)

    def publish(values):
        header = Header(frame_id='gicp_sensor_odom', stamp=node.get_clock().now().to_msg())
        cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, values))

    until(lambda: '[定位就绪]' in log_path.read_text(), lambda: publish(cloud_points))
    until(lambda: abs(received[-1][0].transform.translation.x - 4.1) < 0.02)
    corrected = received[-1][0].transform
    assert corrected.translation.y == pytest.approx(6.45, abs=0.02)
    yaw = 2 * math.atan2(corrected.rotation.z, corrected.rotation.w)
    assert yaw == pytest.approx(math.pi / 2 + 0.04, abs=0.02)
    until(lambda: '本轮匹配无效' in log_path.read_text(), lambda: publish(cloud_points + 100.))
    advance(0.4)
    last, arrival_ns = received[-1]
    assert last.transform.translation.x == pytest.approx(4.1, abs=0.02)
    assert last.transform.translation.y == pytest.approx(6.45, abs=0.02)
    stamp_ns = last.header.stamp.sec * 10**9 + last.header.stamp.nanosec
    assert 0.0 < (stamp_ns - arrival_ns) / 1e9 < 0.25
    assert (received[-1][1] - received[-6][1]) / 1e9 < 0.6

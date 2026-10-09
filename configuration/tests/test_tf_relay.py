"""Installed C++ TF relay contract; live tests use the isolated localhost ROS domain."""
import ast
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

ROOT = Path(__file__).parents[2]


def test_navigation_launch_starts_cpp_tf_relay():
    launch = ROOT / 'src/fishbot/fishbot_navigation2/launch/navigation2.launch.py'
    calls = [item for item in ast.walk(ast.parse(launch.read_text())) if isinstance(item, ast.Call)]
    entries = []
    for call in calls:
        values = {kw.arg: kw.value.value for kw in call.keywords if isinstance(kw.value, ast.Constant)}
        if values.get('package') == 'small_gicp_relocalization':
            entries.append(values.get('executable'))
    assert 'small_gicp_relocalization_node' in entries
    assert entries.count('tf_relay_node') == 1


@pytest.mark.skipif(os.environ.get('CHAIRMAN_ROS_INTEGRATION') != '1', reason='Requires isolated ROS domain 197')
def test_cpp_relay_fallback_live_transform_hold_last_value_and_fresh_stamp(tmp_path):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformBroadcaster

    executable = ROOT / 'install/small_gicp_relocalization/lib/small_gicp_relocalization/tf_relay_node'
    assert executable.is_file(), 'Build and install the standalone C++ relay'
    assert executable.read_bytes()[:4] == b'\x7fELF'
    assert os.environ.get('ROS_DOMAIN_ID') == '197'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    node = rclpy.create_node('relay_test_observer')
    received = []

    def record(message):
        for transform in message.transforms:
            if transform.header.frame_id == 'relay_target_map' and transform.child_frame_id == 'relay_target_odom':
                received.append((transform, node.get_clock().now().nanoseconds))

    node.create_subscription(TFMessage, '/tf', record, 100)
    broadcaster = TransformBroadcaster(node)
    log = (tmp_path / 'relay.log').open('w+')
    process = subprocess.Popen([
        'ros2', 'run', 'small_gicp_relocalization', 'tf_relay_node', '--ros-args',
        '-p', 'source_parent:=relay_source_map', '-p', 'source_child:=relay_source_odom',
        '-p', 'target_parent:=relay_target_map', '-p', 'target_child:=relay_target_odom',
        '-p', 'init_pose:=[1.0, 2.0, 3.0, 0.0, 0.0, 1.5707963267948966]',
        '-p', 'publish_rate_hz:=20.0', '-p', 'time_offset_sec:=0.2',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def until(condition, publish=None):
        deadline = time.monotonic() + 6.0
        while not condition():
            assert process.poll() is None, 'Relay exited unexpectedly'
            assert time.monotonic() < deadline, 'Timed out waiting for relay TF'
            if publish is not None:
                publish()
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        until(lambda: len(received) >= 3)
        first = received[-1][0].transform
        assert (first.translation.x, first.translation.y, first.translation.z) == (1.0, 2.0, 3.0)
        assert first.rotation.z == pytest.approx(math.sqrt(0.5))
        assert first.rotation.w == pytest.approx(math.sqrt(0.5))
        source = TransformStamped()
        source.header.frame_id = 'relay_source_map'
        source.child_frame_id = 'relay_source_odom'
        source.transform.translation.x = 7.0
        source.transform.translation.y = -8.0
        source.transform.translation.z = 9.0
        source.transform.rotation.x = 0.6
        source.transform.rotation.w = 0.8

        def publish():
            source.header.stamp = node.get_clock().now().to_msg()
            broadcaster.sendTransform(source)

        until(lambda: received[-1][0].transform.translation.x == 7.0, publish)
        count = len(received)
        until(lambda: len(received) >= count + 6)
        last, arrival_ns = received[-1]
        assert last.transform == source.transform
        stamp_ns = last.header.stamp.sec * 10**9 + last.header.stamp.nanosec
        assert 0.05 < (stamp_ns - arrival_ns) / 1e9 < 0.3
        elapsed = (received[-1][1] - received[-6][1]) / 1e9
        assert 0.1 < elapsed < 0.6
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        log.close()

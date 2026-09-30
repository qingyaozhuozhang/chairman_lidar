#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener_node')
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_callback,
            10)
        self.get_logger().info("🟢 坐标监听已启动！请在 RViz2 中使用 '2D Pose Estimate' 框选位置...")

    def pose_callback(self, msg):
        # 提取坐标
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 提取四元数
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        print("\n" + "=".join([""]*60))
        print("🎯 获取到新坐标！请修改 src/config/odometry/initial_poses.yaml，再运行配置同步：\n")
        print(f"INIT_X = {x:.3f}")
        print(f"INIT_Y = {y:.3f}")
        print(f"INIT_Z = {z:.3f}")
        print(f"INIT_W = {w:.3f}")
        print("=".join([""]*60) + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

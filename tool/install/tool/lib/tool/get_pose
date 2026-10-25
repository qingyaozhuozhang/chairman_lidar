#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# 1. 将引入的消息类型更改为 PoseStamped
from geometry_msgs.msg import PoseStamped

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener_node')
        self.sub = self.create_subscription(
            PoseStamped,    # 2. 修改订阅的消息类型
            '/goal_pose',   # 3. 修改订阅的话题名称为 Nav2 默认的目标话题
            self.pose_callback,
            10)
        self.get_logger().info("🟢 坐标监听已启动！请在 RViz2 中使用 2D Nav Goal 框选目标位置...")

    def pose_callback(self, msg):
        # 4. 修改提取坐标的层级 (PoseStamped 没有嵌套的 .pose)
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # 提取四元数
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        
        print("\n" + "=".join([""]*60))
        print("🎯 获取到新目标点坐标！\n")
        print(f"GOAL_X = {x:.3f}")
        print(f"GOAL_Y = {y:.3f}")
        print(f"GOAL_Z = {z:.3f}")
        print(f"GOAL_W = {w:.3f}")
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
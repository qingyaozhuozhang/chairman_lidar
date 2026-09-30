#!/usr/bin/env python3
"""持续功能模板；这里只发心跳，不控制底盘。"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusNode(Node):
    def __init__(self):
        super().__init__('chairman_status_example')
        self.publisher = self.create_publisher(String, '/chairman/status_example', 10)
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        # 一直运行由spin和定时器实现；每次回调应尽快返回。
        self.publisher.publish(String(data='alive'))


def main():
    rclpy.init()
    node = StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

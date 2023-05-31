"""
    需求：编码设置话题名称
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
        4.调用spin函数，并传入节点对象；
        5.释放资源。
"""

# 1. 导包
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# 3. 定义节点类
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('hubei', namespace="china")
        # self.publisher_ = self.create_publisher(String, '/topic/chatter', 10)
        # self.publisher_ = self.create_publisher(String, 'topic/chatter', 10)
        self.publisher_ = self.create_publisher(String, '~/topic/chatter', 10)


def main(args=None):
    # 2. 初始化 ROS2 客户端
    rclpy.init(args=args)
    # 4. 调用 spin 函数，并传入节点对象
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    # 5. 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()


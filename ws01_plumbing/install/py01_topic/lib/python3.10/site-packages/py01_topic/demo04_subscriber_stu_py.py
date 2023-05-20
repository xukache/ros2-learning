"""  
    需求：订阅发布方发布的学生消息，并输出到终端。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建订阅方；
            3-2.处理订阅到的学生消息。
        4.调用spin函数，并传入节点对象；
        5.释放资源。
"""

# 1. 导包
import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student

# 定义节点类
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('stu_subscriber_py')
        # 3-1. 创建订阅方
        """
            参数： 
                1. 消息类型
                2. 话题名称
                3. 回调函数
                4. QOS(队列长度)
            返回值：订阅对象
        """
        self.subscription_ = self.create_subscription(
            Student, 'topic_stu', self.subscriber_callback, 10)

    # 3-2. 处理订阅到的消息
    def subscriber_callback(self, stu):
        self.get_logger().info(f'订阅到的学生消息：姓名：{stu.name}，年龄：{stu.age}，身高：{stu.height}')


def main(args=None):
    # 2. 初始化 ROS2 客户端
    rclpy.init(args=args)
    # 4. 调用 spin 函数，并传入节点对象
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # 5. 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()
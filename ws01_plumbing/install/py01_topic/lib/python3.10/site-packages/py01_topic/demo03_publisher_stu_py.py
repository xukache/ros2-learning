"""
    需求：以某个固定频率发送文本学生信息，包含学生的姓名、年龄、身高等数据。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建发布方；
            3-2.创建定时器；
            3-3.组织学生消息并发布。
        4.调用spin函数，并传入节点对象；
        5.释放资源。
"""

# 1. 导包
import rclpy
from rclpy.node import Node
from base_interfaces_demo.msg import Student


# 3. 定义节点类
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('stu_publisher_py')
        # 3-1. 创建发布方
        """
            参数： 
                1. 消息类型
                2. 话题名称
                3. QOS(队列长度)
            返回值：发布对象
        """
        self.publisher_ = self.create_publisher(Student, 'topic_stu', 10)
        # 3-2. 创建定时器
        timer_period = 0.5
        """
            参数：
                1. 时间间隔
                2. 回调函数
            返回值：定时器对象
        """
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # 设置计数器
        self.count = 0

    # 3-3. 组织消息并发布
    def timer_callback(self):
        stu = Student()
        stu.name = '小乌龟'
        stu.age = self.count
        stu.height = 1.10
        self.publisher_.publish(stu)
        self.get_logger().info(
            f'发布的学生信息：姓名：{stu.name}，年龄：{stu.age}，身高：{stu.height}')
        self.count += 1


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

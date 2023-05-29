"""
    需求：演示参数API适用
    流程：
        1. 导包
        2. 初始化 ROS2 客户端
        3. 自定义节点类
            3-1. 创建参数对象
            3-2. 解析参数
        4. 调用 spin 函数，并传入节点对象
        5. 资源释放
"""

# 1. 导包
import rclpy
from rclpy.node import Node


# 3. 定义节点类
class MyParam(Node):
    def __init__(self):
        super().__init__('my_param_node_py')
        self.get_logger().info("参数API使用（Python）")
        # 3-1. 创建参数对象
        p1 = rclpy.Parameter("car_name", value="BMW")
        p2 = rclpy.Parameter("length", value=2.6)
        p3 = rclpy.Parameter("wheels", value=4)
        # 3-2. 解析参数
        self.get_logger().info("car_name = %s" % p1.value)
        self.get_logger().info("length = %.2f" % p2.value)
        self.get_logger().info("wheels = %d" % p3.value)
        # 获取参数键
        self.get_logger().info("p1 name = %s" % p1.name)


def main(args=None):
    # 2. 初始化 ROS2 客户端
    rclpy.init(args=args)
    # 4. 调用 spin 函数，并传入节点对象
    my_param = MyParam()
    rclpy.spin(my_param)
    # 5. 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()

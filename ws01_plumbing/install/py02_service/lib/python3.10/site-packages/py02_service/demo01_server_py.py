"""
    需求：编写服务端，接收客户端发送请求，提取其中两个整型数据，相加后将结果响应回客户端。
    步骤：
        1. 导包
        2. 初始化 ROS2 客户端
        3. 定义节点类
            3-1. 创建服务端
            3-2. 编写回调函数处理请求并产生响应
        4. 调用 spin 函数，并传入节点对象
        5. 释放资源
"""

# 1. 导包
import rclpy
from rclpy.node import Node
from base_interfaces_demo.srv import AddInts


# 定义节点类
class AddIntsServer(Node):
    def __init__(self):
        super().__init__('add_ints_server_node_py')
        self.get_logger().info('服务端启动！(Python)')
        # 3-1. 创建服务端
        self.server_ = self.create_service(
            AddInts, 'add_ints', self.add_two_ints_callback)

    # 3-2. 编写回调函数处理请求并产生响应
    def add_two_ints_callback(self, request, response):
        response.sum = request.num1 + request.num2
        self.get_logger().info(
            f'请求数据：({request.num1}, {request.num2})，响应结果：{response.sum}')
        return response


def main():
    # 2. 初始化 ROS2 客户端
    rclpy.init()
    # 4. 调用 spin 函数，并传入节点对象
    rclpy.spin(AddIntsServer())
    # 5. 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()

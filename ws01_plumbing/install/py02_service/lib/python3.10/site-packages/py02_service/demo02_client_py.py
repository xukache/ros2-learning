"""  
    需求：编写客户端，发送两个整型变量作为请求数据，并处理响应结果。
    步骤：
        1. 导包；
        2. 初始化 ROS2 客户端；
        3. 定义节点类；
            3-1. 创建客户端；
            3-2. 连接服务端
            3-3. 组织请求数据并发送
        4. 调用 spin 函数，并传入节点对象
            需要调用连接服务的函数，根据连接结果做下一步处理
            连接服务后，调用请求发送函数
            再处理响应结果
        5. 释放资源。
"""

# 1. 导包
import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from base_interfaces_demo.srv import AddInts


# 定义节点类
class AddIntsClient(Node):
    def __init__(self):
        super().__init__('add_ints_client_node_py')
        self.get_logger().info('客户端启动！(Python)')
        # 3-1. 创建客户端
        self.client_ = self.create_client(AddInts, 'add_ints')
        # 3-2. 等待服务连接
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务连接中，请稍候...')
        self.request = AddInts.Request()

    # 3-3. 组织请求数据并发送
    def send_request(self):
        self.request.num1 = int(sys.argv[1])
        self.request.num2 = int(sys.argv[2])
        self.future = self.client_.call_async(self.request)  # 发起异步请求


def main():
    # 校验操作
    print(sys.argv)
    if len(sys.argv) != 3:
        get_logger("rclpy").info("请提交两个整型数据")
        return
    # 2. 初始化 ROS2 客户端
    rclpy.init()

    # 4. 创建对象并调用其功能
    add_ints_client = AddIntsClient()
    # 发送请求
    add_ints_client.send_request()

    # 处理响应
    rclpy.spin_until_future_complete(add_ints_client, add_ints_client.future)
    try:
        response = add_ints_client.future.result()
    except Exception as e:
        add_ints_client.get_logger().info(f'服务请求失败：{e,}')
    else:
        add_ints_client.get_logger().info(
            f'响应结果：{add_ints_client.request.num1} + {add_ints_client.request.num2} = {response.sum}')

    # 5. 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()

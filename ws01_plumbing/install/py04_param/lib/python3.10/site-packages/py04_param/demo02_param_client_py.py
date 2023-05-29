"""
    需求：编写参数客户端，操作参数服务端
    步骤：
        1. 导包
        2. 初始化 ROS2 客户端
        3. 定义节点类
            3-1. 列出参数列表
            3-2. 获取参数
            3-3. 设置参数
        4. 创建节点对象，调用参数操作函数
            4-1. 获取参数列表
            4-2. 获取参数
            4-3. 设置参数
        5. 释放资源
"""

# 1.导包
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from ros2param.api import get_parameter_value


# 3. 定义节点类
class ParamClient(Node):

    def __init__(self):
        super().__init__('param_client_node_py')

    # 3-1. 列出参数列表
    def list_params(self):
        # 创建客户端；
        cli_list = self.create_client(
            ListParameters, '/param_server_node_py/list_parameters')
        # 等待服务连接；
        while not cli_list.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('列出参数服务连接中，请稍候...')
        # 发起异步请求
        req = ListParameters.Request()
        future = cli_list.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # 3-2. 获取参数
    def get_params(self, names):
        # 创建客户端；
        cli_get = self.create_client(
            GetParameters, '/param_server_node_py/get_parameters')
        # 等待服务连接；
        while not cli_get.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('获取参数服务连接中，请稍候...')
        
        # 发起异步请求
        req = GetParameters.Request()
        req.names = names
        future = cli_get.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # 3-3. 设置参数
    def set_params(self):
        # 创建客户端；
        cli_set = self.create_client(
            SetParameters, '/param_server_node_py/set_parameters')
        # 等待服务连接；
        while not cli_set.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('设置参数服务连接中，请稍候...')

        # 发起异步请求
        req = SetParameters.Request()

        # 设置参数: 两种方法
        p1 = Parameter()
        p1.name = "car_name"

        # (1) 第一种方法
        # v1 = ParameterValue()
        # v1.type = ParameterType.PARAMETER_STRING
        # v1.string_value = "Pig"
        # p1.value = v1
        # (2) 第二种方法
        p1.value = get_parameter_value(string_value="Pig")  # 根据传入的值自动判断参数类型

        p2 = Parameter()
        p2.name = "height"

        v2 = ParameterValue()
        v2.type = ParameterType.PARAMETER_DOUBLE
        v2.double_value = 0.3
        p2.value = v2
        # p2.value = get_parameter_value(string_value="0.3")

        req.parameters = [p1, p2]
        future = cli_set.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    # 2.初始化 ROS2 客户端；
    rclpy.init()
    # 4.创建对象并调用其功能；
    client = ParamClient()

    # 获取参数列表
    client.get_logger().info("---------获取参数列表---------")
    response = client.list_params()
    for name in response.result.names:
        client.get_logger().info(name)

    client.get_logger().info("-----------获取参数-----------")
    names = ["width", "car_name"]
    response = client.get_params(names)
    # print(response.values)
    for v in response.values:
        if v.type == ParameterType.PARAMETER_STRING:
            client.get_logger().info("%s: %s" % (names[1], v.string_value))
        elif v.type == ParameterType.PARAMETER_DOUBLE:
            client.get_logger().info("%s: %.2f" % (names[0], v.double_value))

    client.get_logger().info("-----------设置参数-----------")
    response = client.set_params()
    results = response.results
    client.get_logger().info("设置了%d个参数" % len(results))
    for result in results:
        if not result.successful:
            client.get_logger().info("参数设置失败")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

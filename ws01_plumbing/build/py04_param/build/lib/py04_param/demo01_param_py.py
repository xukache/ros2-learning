"""
    需求：编写参数服务端，设置并操作参数
    步骤：
        1. 导包
        2. 初始化 ROS2 客户端
        3. 定义节点类
            3-1. 声明参数
            3-2. 查询参数
            3-3. 修改参数
            3-4. 删除参数
        4. 创建节点对象，调用参数操作函数，并传递给 spin 函数
        5. 释放资源
"""

# 1. 导包
import rclpy
from rclpy.node import Node


# 3. 定义节点类
class ParamServer(Node):
    def __init__(self):
        # 如果允许删除参数，需要提前声明，通过 allow_undeclared_parameters=True 参数实现
        super().__init__("param_server_node_py", allow_undeclared_parameters=True)
        self.get_logger().info("参数服务端启动（Python）！")

    # 3-1. 声明参数
    def declare_param(self):
        self.get_logger.info("-------------新增参数-------------")

    # 3-2. 查询参数
    def get_param(self):
        self.get_logger.info("-------------查询参数-------------")

    # 3-3. 修改参数
    def update_param(self):
        self.get_logger.info("-------------修改参数-------------")

    # 3-4. 删除参数
    def delete_param(self):
        self.get_logger.info("-------------删除参数-------------")

def main():
    # 2. 初始化 ROS2 客户端
    rclpy.init()

    # 4. 创建节点对象，调用参数操作函数，并传递给 spin 函数
    param_server = ParamServer()

    param_server.declare_param()
    param_server.get_param()
    param_server.update_param()
    param_server.delete_param()

    rclpy.spin(param_server)

    # 5. 释放资源
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""  
    需求：客户端需要提交目标点坐标，并解析响应结果
    步骤：
        0. 解析动态传入的数据，作为目标点坐标
        1. 导包；
        2. 初始化 ROS2 客户端；
        3. 定义节点类；
            3-1. 构造函数创建客户端
            3-2. 客户端需要连接服务端
            3-3. 组织请求数据并发送
        4. 调用节点对象相关函数
        5. 释放资源。
"""

# 1. 导包
import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from base_interfaces_demo.srv import Distance


# 定义节点类
class Exe03Client(Node):
    def __init__(self):
        super().__init__('exe_client_node_py')
        self.get_logger().info('客户端启动！(Python)')
        # 3-1. 构造函数创建客户端
        self.client_ = self.create_client(Distance, "distance")
        # 3-2. 客户端需要连接服务端
        while not self.client_.wait_for_service(1.0):
            self.get_logger().info("服务连接中...")

        self.request = Distance.Request()

    # 3-3. 组织请求数据并发送
    def send_goal(self, x, y, theta):
        self.request.x = x
        self.request.y = y
        self.request.theta = theta
        self.future = self.client_.call_async(self.request)  # 发起异步请求


def main():
    # 0. 解析动态传入的数据，作为目标点坐标
    # sys.argv 返回的列表第一个值是节点安装路径，所以这里是 5
    # ['/home/xukai/00-Learn/ros2-learning/ws01_plumbing/install/py07_exercise/lib/py07_exercise/exe03_client_py', '8.54', '9.54', '0.0', '--ros-args']
    # get_logger("rclpy").info(str(sys.argv))
    if len(sys.argv) != 5:
        get_logger("rclpy").info("请提交 x 坐标、 y 坐标与 theta 航向角")
        return
    # 解析提交的参数
    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_theta = float(sys.argv[3])
    get_logger("rclpy").info("x = %.2f, y = %.2f, theta = %.2f" %
                             (goal_x, goal_y, goal_theta))

    # 2. 初始化 ROS2 客户端
    rclpy.init()
    # 4. 创建对象并调用其功能
    exe03_client = Exe03Client()
    # 发送请求
    exe03_client.send_goal(goal_x, goal_y, goal_theta)
    # 处理响应
    rclpy.spin_until_future_complete(exe03_client, exe03_client.future)
    try:
        response = exe03_client.future.result()
    except Exception as e:
        exe03_client.get_logger().info(f'服务请求失败：{e,}')
    else:
        exe03_client.get_logger().info('目标乌龟与原生乌龟的距离为：%.2f' % response.distance)

    # 5. 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()

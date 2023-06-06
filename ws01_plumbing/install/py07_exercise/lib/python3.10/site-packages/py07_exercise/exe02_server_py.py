"""
    需求：解析客户端提交的目标点坐标，获取原生乌龟坐标，计算二者距离并响应回客户端
    步骤：
        1. 导包
        2. 初始化 ROS2 客户端
        3. 定义节点类
            3-1. 创建乌龟姿态订阅方（原生乌龟位姿 /turtle1/pose)
            3-2. 创建服务端
            3-3. 回调函数解析客户端数据，计算距离并反馈结果
        4. 调用 spin 函数，并传入节点对象
        5. 释放资源
"""

# 1. 导包
import rclpy
from math import sqrt
from rclpy.node import Node
from turtlesim.msg import Pose
from base_interfaces_demo.srv import Distance


# 定义节点类
class Exe02Server(Node):
    def __init__(self):
        super().__init__('exe_server_node_py')
        self.get_logger().info('服务端启动！(Python)')
        # 3-1. 创建乌龟姿态订阅方（原生乌龟位姿 /turtle1/pose)
        self.subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        # 3-2. 创建服务端
        self.server_ = self.create_service(
            Distance, "distance", self.distance_callback)
        # 创建初始位姿
        self.x = 0.0
        self.y = 0.0

    # 回调函数订阅原生乌龟姿态
    def pose_callback(self, pose):
        self.x = pose.x
        self.y = pose.y

    # 3-3. 回调函数解析客户端数据，计算距离并反馈结果
    def distance_callback(self, request, response):
        # 1. 解析目标点坐标
        goal_x = request.x
        goal_y = request.y
        # 2. 计算距离
        distance_x = goal_x - self.x
        distance_y = goal_y - self.y
        # 3. 设置响应
        response.distance = sqrt(distance_x ** 2 + distance_y ** 2)
        self.get_logger().info("目标乌龟点坐标：(%.2f, %.2f)，原生乌龟点坐标：(%.2f, %.2f)，二者距离：%.2f" %
                               (goal_x, goal_y, self.x, self.y, response.distance))
        
        return response


def main():
    # 2. 初始化 ROS2 客户端
    rclpy.init()
    # 4. 调用 spin 函数，并传入节点对象
    rclpy.spin(Exe02Server())
    # 5. 释放资源
    rclpy.shutdown()


if __name__ == '__main__':
    main()

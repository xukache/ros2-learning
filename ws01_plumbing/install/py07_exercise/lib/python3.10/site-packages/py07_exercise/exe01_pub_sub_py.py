"""
    需求：订阅窗口1中的乌龟速度，然后生成控制窗口2乌龟运动的指令并发布。
    明确：
        订阅话题：/turtle1/pose
        订阅消息：turtlesim/msg/Pose
                x: 0.0
                y: 0.0
                theta: 0.0
                linear_velocty: 0.0
                angular_velocity: 0.0
        发布话题：/t2/turtle1/cmd_vel
        发布消息：geometry_msgs/msg/Twist
                linear:
                    x: 0.0 ---- 前后
                    y: 0.0 ---- 左右
    步骤：
        1. 导包
        2. 初始化 ROS2 客户端
        3. 定义节点类
            3-1. 创建控制第二个窗体乌龟运动的发布方
            3-2. 创建订阅第一个窗体乌龟 pose 的订阅方
            3-3. 根据订阅的乌龟的速度生成控制窗口2乌龟运动的速度消息并发布
        4. 调用 spin 函数，并传入节点对象
        5. 释放资源
    BUG描述：
        乌龟1后退时，乌龟2仍然前进。
    BUG原因：
        1. 和乌龟位姿发布有关，当乌龟实际速度为负数时，位姿中的速度仍是正数
        2. 发布的乌龟2的速度，与位姿中的线速度一致
    BUG修复：
        修改 turtlesim 源代码，将位姿中的线速度计算修改为直接等于 x 方向速度
"""

# 1. 导包
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


# 3. 定义节点类
class ExePubSub(Node):
    def __init__(self):
        super().__init__("demo01_pub_sub")
        # 3-1. 创建控制第二个窗体乌龟运动的发布方
        self.twist_pub_ = self.create_publisher(
            Twist, "/t2/turtle1/cmd_vel", 1)
        # 3-2. 创建订阅第一个窗体乌龟 pose 的订阅方
        self.pose_sub_ = self.create_subscription(
            Pose, "/turtle1/pose", self.poseCallback, 1)
        
    # 3-3. 根据订阅的乌龟的速度生成控制窗口2乌龟运动的速度消息并发布
    def poseCallback(self, pose):
        twist = Twist()
        twist.angular.z = -(pose.angular_velocity)  # 角速度取反
        twist.linear.x = pose.linear_velocity  # 线速度不变
        self.twist_pub_.publish(twist)



def main():
    # 2. 初始化 ROS2 客户端
    rclpy.init()
    # 4. 调用 spin 函数，并传入节点对象
    exe_pub_sub = ExePubSub()
    rclpy.spin(exe_pub_sub)
    # 5. 释放资源
    rclpy.shutdown()


if __name__ == "__main__":
    main()

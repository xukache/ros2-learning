"""
    需求：订阅窗口1中的乌龟速度，然后生成控制窗口2乌龟运动的指令并发布。
    步骤：
        1. 导包
        2. 初始化 ROS2 客户端
        3. 定义节点类
            3-1. 创建控制第二个窗体乌龟运动的发布方
            3-2. 创建订阅第一个窗体乌龟 pose 的订阅方
            3-3. 根据订阅的乌龟的速度生成控制窗口2乌龟运动的速度消息并发布
        4. 调用 spin 函数，并传入节点对象
        5. 释放资源
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

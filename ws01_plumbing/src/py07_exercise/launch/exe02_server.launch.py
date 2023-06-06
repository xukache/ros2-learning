from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 创建 turtlesim_node 节点
    turtle = Node(package="turtlesim", executable="turtlesim_node")
    # 创建测距服务端节点
    server = Node(package="py07_exercise", executable="exe02_server_py")

    return LaunchDescription([turtle, server])

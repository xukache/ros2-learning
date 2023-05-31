from launch import LaunchDescription
from launch_ros.actions import Node

# 节点命名空间和名称修改
# def generate_launch_description():

#     return LaunchDescription([
#         Node(package="turtlesim", executable="turtlesim_node", name="turtle1"),
#         Node(package="turtlesim", executable="turtlesim_node", namespace="t1"),
#         Node(package="turtlesim", executable="turtlesim_node",
#              namespace="t1", name="turtle1")
#     ])

# 话题命名空间和名称修改
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            namespace="t1"),
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            remappings=[("/turtle1/cmd_vel", "/cmd_vel")])
    ])


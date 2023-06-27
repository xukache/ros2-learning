from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

"""
    需求：启动 turtlesim_node 节点，并调用指令打印乌龟的位姿信息



"""


def generate_launch_description():

    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    # 封装指令
    cmd = ExecuteProcess(
        # cmd=["ros2 topic echo /turtle/pose"],
        # cmd=["ros2 topic", "echo", "/turtle1/pose"],
        cmd=[
            FindExecutable(name="ros2"),
            "topic",
            "echo",
            "/turtle1/pose"
        ],
        output="both",  # 打印日志信息
        shell=True
    )

    return LaunchDescription([turtle, cmd])

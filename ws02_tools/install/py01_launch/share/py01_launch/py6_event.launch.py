from launch import LaunchDescription
from launch_ros.actions import Node

# 事件相关
from launch.event_handlers import OnprocessStart, OnProcessExit
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import FindExecutable


def generate_launch_description():
    turtle = Node(package="turtlesim", executable="turtlesim_node")
    spawn = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),  # 不可以有空格
            " service call",
            " /spawn turtlsim/srv/Spawn",
            " \"{x: 8.0, y: 1.0, theta: 1.0, name: 'turtle2'}\""
        ],
        output="both",
        shell=True
    )

    start_event = RegisterEventHandler(
        event_handler=OnprocessStart(
            target_action=turtle,
            on_start=spawn
        )
    )
    exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=turtle,
            on_exit=[LogInfo(msg="turtlesim_node退出！")]
        )
    )
    return LaunchDescription([turtle, start_event, exit_event])

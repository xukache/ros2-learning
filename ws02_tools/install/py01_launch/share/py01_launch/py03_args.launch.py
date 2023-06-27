from launch import LaunchDescription
from launch_ros.actions import Node
# 参数声明与获取
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

"""
    需求：在 launch 文件启动时，动态的设置 turtlesim_node 的背景色
    步骤：
        1. 声明参数（变量）
        2. 调用参数（变量）
        3. 执行 launch 文件时动态设置参数

"""


def generate_launch_description():

    # 声明参数并设置默认值
    decl_bg_r = DeclareLaunchArgument(name="background_r", default_value="255")
    decl_bg_g = DeclareLaunchArgument(name="background_g", default_value="255")
    decl_bg_b = DeclareLaunchArgument(name="background_b", default_value="255")

    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[
            {
                "background_r": LaunchConfiguration("background_r"),
                "background_g": LaunchConfiguration("background_g"),
                "background_b": LaunchConfiguration("background_b")
            }
        ]
    )

    return LaunchDescription([decl_bg_r, decl_bg_g, decl_bg_b, turtle])

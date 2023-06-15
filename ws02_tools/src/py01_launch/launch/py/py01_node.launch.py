from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

"""
    需求：演示 Node 的使用
    构造函数参数说明：
        package 被执行的程序所属的功能包
        executable 可执行程序
        name 节点名称
        namespace 设置命名空间
        exec_name 设置程序标签
        parameters 设置参数
        remappings 实现话题重映射
        arguments 为节点传参
            xx yy zz --ros-args
        ros_arguments 为节点传参
            --ros-args xx yy
"""


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('py01_launch'),
        't2.yaml'
    )

    turtle1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="ns_1",
        name="t1",
        exec_name="my_turtle_label",  # 表示流程的标签
        respawn=True
    )

    turtle2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="t2",
        # 方式1
        # parameters=[{
        #     "background_r": 255,
        #     "background_g": 0,
        #     "background_b": 0
        # }],
        # 方式2 常用, 读取 yaml 文件（加载yaml文件的绝对路径读取）
        parameters=[config],
        respawn=True
    )

    turtle3 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="t3",
        remappings=[("/turtle1/cmd_vel", "/cmd_vel")]  # 话题重映射
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        # 节点启动时传参
        # -d 表示允许指定一个文件作为 rviz2 的启动配置
        # config 表示存储配置文件的路径
        arguments=["-d", config]
    )

    turtle4 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        # 节点启动时传参 相当于 arguments 传参时添加前缀 --ros-args
        ros_arguments=["--remap", "__ns:=/t4_ns", "--remap", "__node:=t4"]
    )

    return LaunchDescription([turtle1, turtle2, turtle3, rviz, turtle4])

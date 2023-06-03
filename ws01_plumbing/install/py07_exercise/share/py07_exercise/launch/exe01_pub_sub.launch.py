# 1. 导包
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


# 定义函数 固定写法，必须有
def generate_launch_description():
    # 1. 创建两个 turtlesim_node 节点
    t1 = Node(
        package="turtlesim",
        executable="turtlesim_node")
    t2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        namespace="t2")  # 修改第二只乌龟的命名空间

    # 2. 让第二只乌龟掉头
    rotate = ExecuteProcess(
        cmd=[
            "ros2 action send_goal /t2/turtle1/rotate_absolute turtlesim/action/RotateAbsolute \"{'theta': 3.14}\""],
        output="both",
        shell=True
    )

    # 3. 自实现的订阅方发布实现
    pub_sub = Node(package="py07_exercise", executable="exe01_pub_sub_py")

    # 4. 乌龟掉头完毕后，开始执行步骤3
    rotate_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rotate,
            on_exit=pub_sub
        )
    )

    return LaunchDescription([t1, t2, rotate, rotate_exit_event])

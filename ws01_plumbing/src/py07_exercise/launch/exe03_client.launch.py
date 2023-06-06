from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # 设置目标点的坐标，以及目标点乌龟的名称
    x = 8.54
    y = 9.54
    theta = 0.0
    name = "t2"

    # 生成新的乌龟
    spawn = ExecuteProcess(
        cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': %s, 'y': %s, 'theta': %s, 'name': %s}\"" %
             (str(x), str(y), str(theta), name)],
        # output="both",
        shell=True
    )

    # 创建客户端节点
    client = Node(package="py07_exercise",
                  executable="exe03_client_py",
                  arguments=[str(x), str(y), str(theta)])
                  # ros2 run py07_exercise exe03_client_py 8.54 9.54 0.0 --ros-args
    
    return LaunchDescription([spawn, client])

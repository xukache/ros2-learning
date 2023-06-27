from launch import LaunchDescription
from launch_ros.actions import Node
# 文件包含相关
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 获取功能包下 share 目录路径
from ament_index_python.packages import get_package_share_directory
import os


"""
    需求：在当前 launch 文件中，包含其他 launch 文件。

"""


def generate_launch_description():
    include_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("py01_launch"),
                "py03_args.launch.py"
            )
        ),
        launch_arguments=[
            ("background_r", "80"),
            ("background_g", "100"),
            ("background_b", "200"),
        ]
    )

    return LaunchDescription([include_launch])

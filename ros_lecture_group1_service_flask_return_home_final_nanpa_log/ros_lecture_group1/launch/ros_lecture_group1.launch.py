"""Launch skeleton for ros_lecture_group1.

このファイルには、State Machineノードを起動する設定を書く。
具体的な処理やROS通信は各Stateファイルのexecute()に追加する。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = 'ros_lecture_group1'
    params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml',
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable='service',
            name='announce_nanpa_qr_server',
            output='screen',
        ),
        Node(
            package=package_name,
            executable='task_node',
            name='task_node',
            parameters=[params_file],
            output='screen',
        ),
    ])

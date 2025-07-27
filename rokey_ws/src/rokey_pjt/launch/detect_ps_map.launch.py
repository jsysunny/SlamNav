# detect_ps_map.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'rokey_pjt'

    node_executable = 'detect_ps_map'  # setup.py entry_points 이름

    # robot_namespace = '/robot3'  # 로봇 TF 네임스페이스
    robot_namespace = '/robot2'  # 로봇 TF 네임스페이스



    return LaunchDescription([
        Node(
            package=package_name,
            executable=node_executable,
            name='detect_ps_map_node',
            output='screen',
            remappings=[
                ('/tf', f'{robot_namespace}/tf'),
                ('/tf_static', f'{robot_namespace}/tf_static'),
            ]
        )
    ])
#!/usr/bin/env python3
"""ロボット側PCで起動する。クライアント(フレーム送信・結果表示) + 追従制御ノードを立ち上げる。

リモートPC(lang_sam_server.launch.py)と同一ROS_DOMAIN_IDで動かすこと。
推論は行わないため、ロボット側PCにGPUは不要。
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lang_sam_executor_dir = get_package_share_directory('lang_sam_executor')
    params_file = os.path.join(lang_sam_executor_dir, 'config', 'params.yaml')

    lang_sam_client_node = Node(
        package='lang_sam_client',
        executable='lang_sam_client_node.py',
        name='lang_sam_client',
        output='screen',
        parameters=[params_file]
    )

    lang_sam_person_following_node = Node(
        package='lang_sam_person_following',
        executable='lang_sam_person_following_node',
        name='lang_sam_person_following',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        lang_sam_client_node,
        lang_sam_person_following_node,
    ])

#!/usr/bin/env python3
"""リモートPC(GPU)で起動する。LangSAM検出 + Cutie追跡を1プロセスで立ち上げる。

ロボット側PC(lang_sam_client.launch.py)と同一ROS_DOMAIN_IDで動かすこと。
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lang_sam_executor_dir = get_package_share_directory('lang_sam_executor')
    params_file = os.path.join(lang_sam_executor_dir, 'config', 'params.yaml')

    # nameは指定しない(1プロセス内に2ノードあるため、ノード名の上書きはしない)
    lang_sam_server_node = Node(
        package='lang_sam_executor',
        executable='lang_sam_server_node.py',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        lang_sam_server_node,
    ])

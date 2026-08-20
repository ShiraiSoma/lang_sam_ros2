#!/usr/bin/env python3
"""リモートPC(GPU)で起動する。LangSAM検出ノードのみを立ち上げる。"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lang_sam_executor_dir = get_package_share_directory('lang_sam_executor')
    params_file = os.path.join(lang_sam_executor_dir, 'config', 'params.yaml')

    lang_sam_detector_node = Node(
        package='lang_sam_detector',
        executable='lang_sam_detector_node.py',
        name='lang_sam_detector',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        lang_sam_detector_node,
    ])

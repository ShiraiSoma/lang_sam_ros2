#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パラメータファイルのパスを取得
    lang_sam_executor_dir = get_package_share_directory('lang_sam_executor')
    params_file = os.path.join(lang_sam_executor_dir, 'config', 'params.yaml')

    # LangSAM Trackerノードの起動設定
    lang_sam_tracker_node = Node(
        package='lang_sam_tracker',
        executable='lang_sam_tracker_node.py',
        name='lang_sam_tracker',
        output='screen',
        parameters=[params_file]
    )

    # 地面追従+ロボット回避ノードの起動設定
    # (person_followingと同時には使わない: どちらも/cmd_velを配信するため)
    lang_sam_ground_nav_node = Node(
        package='lang_sam_ground_nav',
        executable='lang_sam_ground_nav_node',
        name='lang_sam_ground_nav',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        lang_sam_tracker_node,
        lang_sam_ground_nav_node
    ])

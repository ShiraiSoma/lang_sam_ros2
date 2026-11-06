#!/usr/bin/env python3
"""リモートPC(GPU)で起動する。LangSAM検出 + Cutie追跡(1プロセス)に加え、
地面追従+障害物回避ノード(lang_sam_ground_nav)を立ち上げる。

lang_sam_ground_navが出す/cmd_velはros1_bridge経由でロボット側(ROS1)へ送る。
ロボット側はlang_sam_client(フレーム送信・結果表示)のみ起動すればよい。
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

    # 地面追従+障害物回避(/cmd_velを配信。person_followingとは同時に使わない)
    lang_sam_ground_nav_node = Node(
        package='lang_sam_ground_nav',
        executable='lang_sam_ground_nav_node',
        name='lang_sam_ground_nav',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        lang_sam_server_node,
        lang_sam_ground_nav_node,
    ])

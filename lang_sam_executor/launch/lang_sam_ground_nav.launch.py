#!/usr/bin/env python3
"""リモートPC(GPU)で起動する。LangSAM検出 + Cutie追跡(1プロセス)に加え、
地面追従+障害物回避ノード(lang_sam_ground_nav)を立ち上げる。

lang_sam_ground_navが出す速度指令はros1_bridge経由でロボット側(ROS1)へ送る。
ロボット側はlang_sam_client(フレーム送信・結果表示)のみ起動すればよい。

速度指令のトピック名は引数 cmd_vel_topic で変更できる(既定: /icart_mini/cmd_vel)。
ORNE-box(iCart mini)の足回りはdiff_drive_controller(名前: icart_mini)が/icart_mini/cmd_velを購読するため。
変更する場合は lang_sam_ros1 の bridge/bridge_topics.yaml のトピック名も合わせること。
  例: ros2 launch lang_sam_executor lang_sam_ground_nav.launch.py cmd_vel_topic:=/cmd_vel
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/icart_mini/cmd_vel',
        description='速度指令の出力先トピック(ロボットの足回りが購読するもの)')

    # 地面追従+障害物回避(速度指令を配信。person_followingとは同時に使わない)
    lang_sam_ground_nav_node = Node(
        package='lang_sam_ground_nav',
        executable='lang_sam_ground_nav_node',
        name='lang_sam_ground_nav',
        output='screen',
        parameters=[params_file],
        remappings=[('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))]
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        lang_sam_server_node,
        lang_sam_ground_nav_node,
    ])

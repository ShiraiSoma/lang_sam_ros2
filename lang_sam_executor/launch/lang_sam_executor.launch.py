#!/usr/bin/env python3
"""単一マシンで全ノード(検出+追跡+クライアント+追従)をまとめて起動する動作確認用launch。

2台構成で実運用する場合は lang_sam_server.launch.py (リモートPC) と
lang_sam_client.launch.py (ロボット側PC) を別々に使うこと。
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パラメータファイルのパスを取得
    lang_sam_executor_dir = get_package_share_directory('lang_sam_executor')
    params_file = os.path.join(lang_sam_executor_dir, 'config', 'params.yaml')

    # LangSAM Detectorノードの起動設定
    lang_sam_detector_node = Node(
        package='lang_sam_detector',
        executable='lang_sam_detector_node.py',
        name='lang_sam_detector',
        output='screen',
        parameters=[params_file]
    )

    # LangSAM Trackerノードの起動設定
    lang_sam_tracker_node = Node(
        package='lang_sam_tracker',
        executable='lang_sam_tracker_node.py',
        name='lang_sam_tracker',
        output='screen',
        parameters=[params_file]
    )

    # LangSAM Clientノードの起動設定(フレーム送信・結果表示)
    lang_sam_client_node = Node(
        package='lang_sam_client',
        executable='lang_sam_client_node.py',
        name='lang_sam_client',
        output='screen',
        parameters=[params_file]
    )


    return LaunchDescription([
        lang_sam_detector_node,
        lang_sam_tracker_node,
        lang_sam_client_node,
    ])
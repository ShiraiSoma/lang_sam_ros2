#!/usr/bin/env python3
"""リモートPC(GPU)側: LangSAM検出ノードとCutieトラッカーノードを1プロセスで起動する

同一プロセス内のノードは1つのDDSパーティシパントを共有するため、gpumng2の
TCPトランスポート構成(server_profile.xml)でも待ち受けポート1つで両ノードの
トピックをまとめてクライアントへ公開できる。パラメータは通常通り
--ros-args --params-file でノード名(lang_sam_detector / lang_sam_tracker)ごとに渡す。
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor

# 注意: sam2はimport時にHydraをグローバル初期化する。CutieTrackerはその状態を
#       退避・復元してから自身のcfgを合成するので、検出ノードを先に生成する
from lang_sam_detector.lang_sam_detector_node import LangSamDetectorNode
from lang_sam_tracker.lang_sam_tracker_node import LangSamTrackerNode


def main(args=None):
    rclpy.init(args=args)
    detector = LangSamDetectorNode()
    tracker = LangSamTrackerNode()
    # このターミナルでEnterを押すと検出の認識対象(text_prompt)を切り替える
    detector.start_keyboard_thread()

    # 検出(数百ms〜)が毎フレームの追跡を止めないよう、ノードごとに並列実行する
    executor = MultiThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(tracker)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        tracker.destroy_node()
        detector.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""LangSAMクライアントノード(ロボット側PC)

カメラ画像をJPEG圧縮してリモートのlang_sam_tracker(Cutie)へ送り、返ってきた
追跡結果(TrackArray + インデックスマスク)と検出結果(DetectionArray)を
手元に保持している元フレームに重ねて描画・表示する。

推論は一切行わないためtorch/GPUは不要。結果のheader.stampは送信したフレームの
stampがそのまま返ってくるので、それをキーに元フレームと対応付ける
(結果画像そのものは受け取らないので、下りの通信量はマスク分だけで済む)。
"""
import time
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image as ROSImage

from lang_sam_client.vis_utils import draw_image
from lang_sam_msgs.msg import DetectionArray, TrackArray

# 追跡結果(tracks/mask)の片方だけ届いた状態で保持しておく最大件数
MAX_PENDING_RESULTS = 30


def _stamp_key(stamp):
    return (int(stamp.sec), int(stamp.nanosec))


class LangSamClientNode(Node):
    def __init__(self):
        super().__init__('lang_sam_client')
        self.logger = self.get_logger()
        self.logger.info('Initializing LangSAM Client Node...')

        self._setup_parameters()

        self.bridge = CvBridge()

        # 送信済みフレーム(JPEGバイト列)をstampで引けるよう保持する
        self.sent_frames = OrderedDict()
        self.last_sent_time = None

        # 追跡結果は2トピックに分かれて届くため、stampが揃うまで待つ
        self.pending_tracks = OrderedDict()
        self.pending_masks = OrderedDict()

        # 表示用の状態
        self.latest_det_vis = None
        self.fps_alpha = 0.1
        self.track_last_time = None
        self.track_fps = 0.0
        self.det_last_time = None
        self.det_fps = 0.0

        # リモート(TCP)越しのやり取りはすべて最新1件のみ保持する(古いものは捨てる)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.image_sub = self.create_subscription(ROSImage, self.image_topic, self.image_callback, 1)
        self.frame_pub = self.create_publisher(CompressedImage, self.frame_topic, qos)
        self.tracks_sub = self.create_subscription(TrackArray, self.tracks_topic, self._on_tracks, qos)
        self.track_mask_sub = self.create_subscription(
            CompressedImage, self.track_mask_topic, self._on_track_mask, qos)
        self.detections_sub = self.create_subscription(
            DetectionArray, self.detections_topic, self._on_detections, qos)

        self.get_logger().info(f'Image topic: {self.image_topic}')
        self.get_logger().info(f'Frame topic (to tracker): {self.frame_topic}')
        self.get_logger().info(f'Tracks topic: {self.tracks_topic}')
        self.get_logger().info(f'Track mask topic: {self.track_mask_topic}')
        self.get_logger().info(f'Detections topic: {self.detections_topic}')
        self.get_logger().info('LangSAM Client initialized.')

    def _setup_parameters(self):
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('frame_topic', '/lang_sam/frame')
        self.declare_parameter('tracks_topic', '/lang_sam/tracks')
        self.declare_parameter('track_mask_topic', '/lang_sam/track_mask')
        self.declare_parameter('detections_topic', '/lang_sam/detections')
        self.declare_parameter('jpeg_quality', 85)          # 送信フレームのJPEG品質
        self.declare_parameter('stream_max_fps', 0.0)       # 送信フレームレートの上限。0以下で無制限(カメラのレート)
        self.declare_parameter('frame_buffer_size', 300)    # 結果と対応付けるため保持する送信済みフレーム数
        self.declare_parameter('visualize', True)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.frame_topic = self.get_parameter('frame_topic').get_parameter_value().string_value
        self.tracks_topic = self.get_parameter('tracks_topic').get_parameter_value().string_value
        self.track_mask_topic = self.get_parameter('track_mask_topic').get_parameter_value().string_value
        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').get_parameter_value().integer_value)
        self.stream_max_fps = float(self.get_parameter('stream_max_fps').get_parameter_value().double_value)
        self.frame_buffer_size = int(self.get_parameter('frame_buffer_size').get_parameter_value().integer_value)
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value

    # --- EMA による FPS 更新の共通処理 ---
    def _update_ema_fps(self, now, last_name, fps_name):
        last = getattr(self, last_name)
        fps = getattr(self, fps_name)
        setattr(self, last_name, now)
        if last is None:
            return
        dt = now - last
        if dt > 1e-6:
            inst = 1.0 / dt
            setattr(self, fps_name, (1.0 - self.fps_alpha) * fps + self.fps_alpha * inst if fps > 0 else inst)

    def _lookup_frame(self, stamp):
        jpeg = self.sent_frames.get(_stamp_key(stamp))
        if jpeg is None:
            return None
        return cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)

    # --- フレーム送信 ---
    def image_callback(self, msg: ROSImage):
        now = time.time()
        if self.stream_max_fps > 0 and self.last_sent_time is not None \
                and now - self.last_sent_time < 1.0 / self.stream_max_fps:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            self.get_logger().warning('JPEGエンコードに失敗したためフレームを送信しません')
            return

        frame_msg = CompressedImage()
        frame_msg.header = msg.header
        # stampは結果との対応付けに使うため、ドライバが入れていない場合は現在時刻で埋める
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            frame_msg.header.stamp = self.get_clock().now().to_msg()
        frame_msg.format = 'jpeg'
        frame_msg.data = jpeg.tobytes()

        self.sent_frames[_stamp_key(frame_msg.header.stamp)] = frame_msg.data
        while len(self.sent_frames) > self.frame_buffer_size:
            self.sent_frames.popitem(last=False)

        self.frame_pub.publish(frame_msg)
        self.last_sent_time = now

    # --- 追跡結果の受信 ---
    def _on_tracks(self, msg: TrackArray):
        self._store_pending(self.pending_tracks, msg)

    def _on_track_mask(self, msg: CompressedImage):
        self._store_pending(self.pending_masks, msg)

    def _store_pending(self, store, msg):
        key = _stamp_key(msg.header.stamp)
        store[key] = msg
        while len(store) > MAX_PENDING_RESULTS:
            store.popitem(last=False)
        if key in self.pending_tracks and key in self.pending_masks:
            tracks_msg = self.pending_tracks.pop(key)
            mask_msg = self.pending_masks.pop(key)
            # 対応が取れたものより古い片割れは今後揃うことがないので捨てる
            for s in (self.pending_tracks, self.pending_masks):
                for k in [k for k in s if k < key]:
                    del s[k]
            self._render_tracking(tracks_msg, mask_msg)

    def _render_tracking(self, tracks_msg: TrackArray, mask_msg: CompressedImage):
        self._update_ema_fps(time.time(), 'track_last_time', 'track_fps')
        if not self.visualize:
            return

        frame = self._lookup_frame(tracks_msg.header.stamp)
        if frame is None:
            self.logger.debug('対応する送信済みフレームがない追跡結果を破棄します')
            return
        idx_mask = cv2.imdecode(np.frombuffer(bytes(mask_msg.data), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if idx_mask is None or idx_mask.shape[:2] != frame.shape[:2]:
            self.get_logger().warning('トラックマスクのデコードに失敗、またはフレームとサイズが一致しません')
            return

        tracks = list(tracks_msg.tracks)
        h, w = frame.shape[:2]
        masks = np.zeros((len(tracks), h, w), dtype=bool)
        for i, t in enumerate(tracks):
            masks[i] = idx_mask == t.id
        boxes = np.asarray([[t.x_min, t.y_min, t.x_max, t.y_max] for t in tracks], dtype=np.int32) \
            if tracks else np.zeros((0, 4), dtype=np.int32)
        scores = np.asarray([t.score for t in tracks], dtype=np.float32)
        labels = [t.label for t in tracks]

        track_vis = self._draw(frame, masks, boxes, scores, labels)
        if track_vis is None:
            return
        self._show(track_vis)

    # --- 検出結果の受信 ---
    def _on_detections(self, msg: DetectionArray):
        self._update_ema_fps(time.time(), 'det_last_time', 'det_fps')
        if not self.visualize:
            return

        frame = self._lookup_frame(msg.header.stamp)
        if frame is None:
            self.logger.debug('対応する送信済みフレームがない検出結果を破棄します')
            return
        try:
            idx_mask = self.bridge.imgmsg_to_cv2(msg.index_mask, desired_encoding='mono16').astype(np.int32)
        except Exception as e:
            self.get_logger().error(f'index_maskのデコードに失敗: {e}')
            return

        n = len(msg.detections)
        h, w = idx_mask.shape
        masks = np.zeros((n, h, w), dtype=bool)
        for i in range(n):
            masks[i] = idx_mask == (i + 1)
        boxes = np.asarray([[d.x_min, d.y_min, d.x_max, d.y_max] for d in msg.detections], dtype=np.int32) \
            if n > 0 else np.zeros((0, 4), dtype=np.int32)
        scores = np.asarray([d.score for d in msg.detections], dtype=np.float32)
        labels = [d.label for d in msg.detections]

        det_vis = self._draw(frame, masks, boxes, scores, labels)
        if det_vis is not None:
            self.latest_det_vis = det_vis

    # --- 描画 ---
    def _draw(self, frame_bgr, masks, boxes, scores, labels):
        try:
            pil = PILImage.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
            vis = draw_image(image_rgb=pil, masks=masks, xyxy=boxes, probs=scores, labels=labels)
            return cv2.cvtColor(np.array(vis), cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().warning(f'draw_image失敗: {e}')
            return None

    @staticmethod
    def _put_fps(image, fps):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = max(0.6, image.shape[1] / 1000.0)
        thickness = 2
        text = f'FPS: {fps:.1f}'
        (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
        x = max(10, image.shape[1] - 10 - tw)  # 右端に寄せ、最小余白を確保
        y = 10 + th
        cv2.putText(image, text, (x, y), font, font_scale, (0, 255, 255), thickness, cv2.LINE_AA)

    def _show(self, track_vis):
        # 検出可視化と追跡可視化を横並びで表示(FPSを描画)
        h_trk, w_trk = track_vis.shape[:2]
        if self.latest_det_vis is None:
            det_vis = np.zeros((h_trk, w_trk, 3), dtype=np.uint8)
            cv2.putText(det_vis, 'No detection yet', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
        else:
            det_vis = self.latest_det_vis.copy()
            # サイズ合わせ(高さを基準に揃える)
            h_det, w_det = det_vis.shape[:2]
            if h_det != h_trk:
                det_vis = cv2.resize(det_vis, (int(w_det * h_trk / h_det), h_trk))

        self._put_fps(det_vis, self.det_fps)
        self._put_fps(track_vis, self.track_fps)

        combined = np.hstack([det_vis, track_vis])

        # ラベル描画: 左=Detection, 右=Tracking
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = max(0.6, combined.shape[1] / 1000.0)
        cv2.putText(combined, 'Detection', (10, 30), font, font_scale, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(combined, 'Tracking', (det_vis.shape[1] + 10, 30), font, font_scale, (0, 255, 0), 2, cv2.LINE_AA)

        # 非ブロッキング表示
        cv2.imshow('LangSAM', combined)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LangSamClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

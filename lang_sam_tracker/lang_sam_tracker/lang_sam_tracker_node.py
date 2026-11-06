#!/usr/bin/env python3
"""Cutie(VOS)によるマスク伝播トラッキングノード

リモートPC(GPU)で動作させることを想定。ロボット側のlang_sam_clientから
JPEG圧縮フレーム(frame_topic)を受け取り、毎フレームCutieでマスクを伝播する。
一定間隔でフレームをlang_sam_detectorへ転送して再検出し、結果をトラックへマージする。
可視化は行わず、結果はTrackArray(tracks_topic)とインデックスマスク
(track_mask_topic, 16bit PNG)として返す。どちらもheader.stampは入力フレームの
stampをそのまま引き継ぐので、クライアント側で元フレームと対応付けて描画できる。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge
import numpy as np
import cv2
import threading
import time

from lang_sam_msgs.msg import TrackArray, Track, DetectionArray  # カスタムメッセージ
from lang_sam_tracker.cutie_tracker import CutieTracker, get_device  # Cutie(VOS)マスク伝播

class LangSamTrackerNode(Node):
    def __init__(self):
        super().__init__('lang_sam_tracker')
        self.logger = self.get_logger()
        self.logger.info('Initializing LangSAM Tracker Node...')

        # パラメータ宣言・取得
        self._setup_parameters()

        # 使用デバイス
        self.device = get_device()

        # Cutie(VOS)トラッカーのロード
        # 検出マスクをメモリに記銘し、毎フレームのマスクを伝播で更新する
        # 注意: lang_sam_server_node.pyでlang_sam_detectorと同一プロセスで動かす場合、
        #       SAM2のHydraグローバル初期化との衝突はCutieTracker側で回避している
        self.cutie = CutieTracker(
            device=self.device,
            weights_path=(self.cutie_weights or None),
            max_internal_size=self.cutie_max_internal_size,
            mem_every=self.cutie_mem_every,
            use_long_term=self.cutie_use_long_term,
            use_amp=self.cutie_use_amp,
        )

        # ROS <-> OpenCV画像変換
        self.bridge = CvBridge()

        # トラッキング状態: id -> {label, score, box, mask, miss_det, lost_frames}
        # idはCutieのオブジェクトID(0は背景のため1始まり)
        self.tracks = {}
        self.next_track_id = 1
        # タイマー検出用に最新フレームを保持(デコード済みBGRと受信した圧縮メッセージ)
        self.latest_bgr = None
        self.latest_frame_msg = None

        # 共有状態ロック
        self.state_lock = threading.Lock()

        # リモート検出リクエストの状態(同時に1件まで): stamp/frame/送信時刻
        self.pending_stamp = None
        self.pending_frame = None
        self.pending_since = None

        # 遠隔(TCP)越しのやり取りはすべて最新1件のみ保持する(古いフレームは捨てる)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # I/O: クライアントからのフレーム受信 / 追跡結果の配信
        self.frame_sub = self.create_subscription(CompressedImage, self.frame_topic, self.frame_callback, qos)
        self.tracks_pub = self.create_publisher(TrackArray, self.tracks_topic, qos)
        self.track_mask_pub = self.create_publisher(CompressedImage, self.track_mask_topic, qos)

        # LangSAM検出ノードとのI/O(detect_request送信 / detections受信)
        self.detect_request_pub = self.create_publisher(CompressedImage, self.detect_request_topic, qos)
        self.detections_sub = self.create_subscription(
            DetectionArray, self.detect_response_topic, self._on_detections, qos)

        # 検出リクエストはタイマーで送信(detection_interval_secを周期として使用)
        self.detection_timer = self.create_timer(float(self.detection_interval_sec), self.timer_callback)

        # 処理時間の計測(カクつき・遅延の切り分け用)。stats_log_sec周期でまとめてログ出力する
        self.stats_lock = threading.Lock()
        self._reset_stats()
        if self.stats_log_sec > 0:
            self.stats_timer = self.create_timer(self.stats_log_sec, self._log_stats)

        # ログ
        self.get_logger().info(f'Using device: {self.device}')
        self.get_logger().info(f'Detection interval (sec): {self.detection_interval_sec}')
        self.get_logger().info(f'Frame topic: {self.frame_topic}')
        self.get_logger().info(f'Tracks topic: {self.tracks_topic}')
        self.get_logger().info(f'Track mask topic: {self.track_mask_topic}')
        self.get_logger().info(f'Detect request topic: {self.detect_request_topic}')
        self.get_logger().info(f'Detect response topic: {self.detect_response_topic}')
        self.get_logger().info('Cutie tracker initialized.')

    # パラメータ取得用の関数
    def _setup_parameters(self):
        # 注意: launch/config側で上書き可能
        self.declare_parameter('detection_interval_sec', 2.0)
        self.declare_parameter('frame_topic', '/lang_sam/frame')
        self.declare_parameter('tracks_topic', '/lang_sam/tracks')
        self.declare_parameter('track_mask_topic', '/lang_sam/track_mask')
        self.declare_parameter('detect_request_topic', '/lang_sam/detect_request')
        self.declare_parameter('detect_response_topic', '/lang_sam/detections')
        # 検出リクエストを送ってから応答が来ない場合に諦めて再送するまでの倍率
        # (detection_interval_sec * この倍率が経過したらタイムアウト扱い)
        self.declare_parameter('detect_timeout_multiplier', 3.0)

        # Cutie(VOS)のROSパラメータ
        self.declare_parameter('cutie_weights', '')              # 空なら~/.cache/cutie/へ自動ダウンロード
        self.declare_parameter('cutie_max_internal_size', 480)   # 内部処理の最小辺(px)。小さいほど速い/粗い
        self.declare_parameter('cutie_mem_every', 5)              # メモリ記銘の間隔(フレーム)
        self.declare_parameter('cutie_use_long_term', True)       # 長期メモリ(長時間の追跡でメモリ量を抑制)
        self.declare_parameter('cutie_use_amp', False)             # 混合精度で高速化(精度僅かに低下)

        # トラック管理(検出とのマージ規則)のROSパラメータ
        self.declare_parameter('merge_iou_threshold', 0.3)          # 検出と既存トラックを同一とみなすマスクIoU
        self.declare_parameter('duplicate_overlap_threshold', 0.5)  # 同一物体の多重登録とみなす重なり率(交差/小さい方の面積)
        self.declare_parameter('track_max_detection_misses', 3)     # 再検出で連続この回数裏付けなし→削除
        self.declare_parameter('track_lost_frames', 90)             # 伝播マスクが空のフレーム数がこれを超えたら削除
        self.declare_parameter('track_min_mask_area', 100)          # これ未満の面積(px)のマスクは消失扱い(重複登録の残骸対策)
        self.declare_parameter('stats_log_sec', 5.0)                # 処理時間の集計ログを出す周期(秒)。0以下で無効

        self.detection_interval_sec = self.get_parameter('detection_interval_sec').get_parameter_value().double_value
        self.frame_topic = self.get_parameter('frame_topic').get_parameter_value().string_value
        self.tracks_topic = self.get_parameter('tracks_topic').get_parameter_value().string_value
        self.track_mask_topic = self.get_parameter('track_mask_topic').get_parameter_value().string_value
        self.detect_request_topic = self.get_parameter('detect_request_topic').get_parameter_value().string_value
        self.detect_response_topic = self.get_parameter('detect_response_topic').get_parameter_value().string_value
        self.detect_timeout_multiplier = float(self.get_parameter('detect_timeout_multiplier').get_parameter_value().double_value)

        # Cutieパラメータの取得
        self.cutie_weights = self.get_parameter('cutie_weights').get_parameter_value().string_value
        self.cutie_max_internal_size = int(self.get_parameter('cutie_max_internal_size').get_parameter_value().integer_value)
        self.cutie_mem_every = int(self.get_parameter('cutie_mem_every').get_parameter_value().integer_value)
        self.cutie_use_long_term = bool(self.get_parameter('cutie_use_long_term').get_parameter_value().bool_value)
        self.cutie_use_amp = bool(self.get_parameter('cutie_use_amp').get_parameter_value().bool_value)

        # トラック管理パラメータの取得
        self.merge_iou_threshold = float(self.get_parameter('merge_iou_threshold').get_parameter_value().double_value)
        self.duplicate_overlap_threshold = float(self.get_parameter('duplicate_overlap_threshold').get_parameter_value().double_value)
        self.track_max_detection_misses = int(self.get_parameter('track_max_detection_misses').get_parameter_value().integer_value)
        self.track_lost_frames = int(self.get_parameter('track_lost_frames').get_parameter_value().integer_value)
        self.track_min_mask_area = int(self.get_parameter('track_min_mask_area').get_parameter_value().integer_value)
        self.stats_log_sec = float(self.get_parameter('stats_log_sec').get_parameter_value().double_value)

    # --- 処理時間の計測 ---
    def _reset_stats(self):
        self.stats = {
            'total': [], 'lock_wait': [], 'cutie': [], 'encode': [],   # 各区間の所要時間(秒)
            'merges': [],                      # 検出マージ(ロック保持)の所要時間(秒)
        }
        self.stats_since = time.time()

    def _add_stat(self, key, value):
        with self.stats_lock:
            self.stats[key].append(value)

    def _log_stats(self):
        with self.stats_lock:
            s = self.stats
            elapsed = max(time.time() - self.stats_since, 1e-6)
            self._reset_stats()

        def fmt(vals):
            if not vals:
                return '-'
            return f'avg {1000 * sum(vals) / len(vals):.0f} / max {1000 * max(vals):.0f} ms'

        self.get_logger().info(
            f'[stats {elapsed:.0f}s] 処理 {len(s["total"]) / elapsed:.1f} fps | '
            f'1フレーム合計 {fmt(s["total"])} | ロック待ち {fmt(s["lock_wait"])} | '
            f'Cutie {fmt(s["cutie"])} | PNG+publish {fmt(s["encode"])} | '
            f'検出マージ {len(s["merges"])}回 {fmt(s["merges"])}')

    # --- tensor/torch -> numpy 変換 ---
    def _to_numpy(self, x):
        if x is None:
            return None
        if hasattr(x, 'cpu'):
            try:
                x = x.cpu().numpy()
            except Exception:
                # 非 tensor オブジェクトなどをそのまま配列化
                pass
        return np.asarray(x)

    @staticmethod
    def _box_from_mask(mask_bool):
        # マスクの外接矩形 [x1,y1,x2,y2] を返す(空マスクはNone)
        ys, xs = np.where(mask_bool)
        if len(xs) == 0:
            return None
        return [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]

    @staticmethod
    def _mask_iou(a, b):
        inter = np.logical_and(a, b).sum()
        if inter == 0:
            return 0.0
        union = np.logical_or(a, b).sum()
        return float(inter) / float(union)

    @staticmethod
    def _label_match(a, b):
        # ラベル比較用の正規化(末尾ピリオド・前後空白・大小文字を無視)
        norm = lambda s: str(s).strip().rstrip('.').strip().lower()
        return norm(a) == norm(b)

    @staticmethod
    def _mask_overlap_min(a, b):
        # 交差 / 小さい方の面積。部分検出(全体の一部だけの検出)や包含関係は
        # IoUが低く出るため、多重登録の判定にはこちらを使う
        inter = np.logical_and(a, b).sum()
        if inter == 0:
            return 0.0
        smaller = min(a.sum(), b.sum())
        return float(inter) / float(smaller) if smaller > 0 else 0.0

    def _delete_tracks(self, ids):
        # Cutieメモリとトラック辞書の両方から削除(state_lock保持前提)
        if not ids:
            return
        self.cutie.delete(ids)
        for tid in ids:
            self.tracks.pop(tid, None)

    def _update_tracks_with_cutie(self, cv_image):
        # Cutieのメモリに基づき現フレームへマスクを伝播し、各トラックのマスク/bboxを更新
        # (state_lock保持前提)
        if not self.tracks:
            return
        idx_mask = self.cutie.track(cv_image)
        to_delete = []
        for tid, t in self.tracks.items():
            m = idx_mask == tid
            # 極小マスクは消失扱い(多重登録の残骸や誤伝播をlost_framesで淘汰)
            if m.sum() >= self.track_min_mask_area:
                t['mask'] = m
                t['box'] = self._box_from_mask(m)
                t['lost_frames'] = 0
            else:
                # 伝播でマスクが消失(遮蔽/画面外など)。bboxは最後の位置を保持
                t['mask'] = m
                t['lost_frames'] += 1
                if t['lost_frames'] > self.track_lost_frames:
                    to_delete.append(tid)
        self._delete_tracks(to_delete)

    def _merge_detections(self, det_frame, labels, scores, masks_bool):
        # 再検出結果と既存トラックをマスクIoUでマッチングしてマージ(state_lock保持前提)
        # - マッチ: IDを維持しつつ新鮮なSAMマスクで矯正(ドリフト解消)
        # - 未マッチの検出: 新規トラック
        # - 未マッチのトラック: missカウント増、閾値超過で削除
        h, w = det_frame.shape[:2]
        get_label = lambda i: labels[i] if i < len(labels) else 'obj'

        # 空マスクの検出は除外(疑似マスクは作らない設計)
        det_indices = [i for i in range(masks_bool.shape[0]) if masks_bool[i].any()]

        # 検出同士の重複除去(同一物体の多重検出対策):
        # スコア降順に走査し、採用済み検出と重なり率が閾値以上のものは捨てる
        # (ラベルが異なる場合は別物体とみなし、重複除去の対象外とする)
        det_indices.sort(key=lambda i: -(float(scores[i]) if i < len(scores) else 1.0))
        kept = []
        for i in det_indices:
            dup = any(self._mask_overlap_min(masks_bool[i], masks_bool[j]) >= self.duplicate_overlap_threshold
                      and self._label_match(get_label(i), get_label(j))
                      for j in kept)
            if not dup:
                kept.append(i)
        det_indices = kept

        # IoU降順の貪欲マッチング(ラベルが一致するトラックのみ対象)
        track_ids = list(self.tracks.keys())
        pairs = []
        for i in det_indices:
            for tid in track_ids:
                if not self._label_match(get_label(i), self.tracks[tid]['label']):
                    continue
                iou = self._mask_iou(masks_bool[i], self.tracks[tid]['mask'])
                if iou >= self.merge_iou_threshold:
                    pairs.append((iou, i, tid))
        pairs.sort(reverse=True)
        det_to_tid = {}
        matched_tids = set()
        for iou, i, tid in pairs:
            if i in det_to_tid or tid in matched_tids:
                continue
            det_to_tid[i] = tid
            matched_tids.add(tid)

        # 未マッチトラックのmiss処理
        to_delete = []
        for tid in track_ids:
            if tid in matched_tids:
                continue
            t = self.tracks[tid]
            t['miss_det'] += 1
            if t['miss_det'] > self.track_max_detection_misses:
                to_delete.append(tid)
        self._delete_tracks(to_delete)

        # 未マッチの検出は新規トラックとして採番
        # ただし既存トラックと重なり率が高いものは同一物体の部分検出とみなして捨てる
        # (IoUは低いがマスクが既存トラック内に包含されるケースの多重登録対策)
        # (ラベルが異なる既存トラックとの重なりは別物体のため対象外)
        for i in det_indices:
            if i in det_to_tid:
                continue
            if any(self._mask_overlap_min(masks_bool[i], t['mask']) >= self.duplicate_overlap_threshold
                   and self._label_match(get_label(i), t['label'])
                   for t in self.tracks.values()):
                continue
            det_to_tid[i] = self.next_track_id
            self.tracks[self.next_track_id] = {
                'label': get_label(i),
                'score': float(scores[i]) if i < len(scores) else 1.0,
                'box': [0, 0, 0, 0],
                'mask': np.zeros((h, w), dtype=bool),
                'miss_det': 0,
                'lost_frames': 0,
            }
            self.next_track_id += 1

        # マッチしたトラックはラベル/スコアを更新しmissをリセット
        for i, tid in det_to_tid.items():
            t = self.tracks[tid]
            if i < len(labels):
                t['label'] = labels[i]
            if i < len(scores):
                t['score'] = float(scores[i])
            t['miss_det'] = 0

        if not self.tracks:
            return

        # Cutieへ渡す統合インデックスマスクを構成
        # - 検出に裏付けられないが生存中のトラックは現在のマスクを維持
        # - 検出マスクはスコア昇順に塗る(重なりは高スコアが優先)
        combined = np.zeros((h, w), dtype=np.int32)
        for tid, t in self.tracks.items():
            if tid not in matched_tids and tid not in det_to_tid.values():
                combined[t['mask']] = tid
        order = sorted(det_to_tid.keys(), key=lambda i: float(scores[i]) if i < len(scores) else 1.0)
        for i in order:
            combined[masks_bool[i]] = det_to_tid[i]

        # 検出フレームを正解としてメモリに記銘(初期化・矯正兼用)
        alive_ids = list(self.tracks.keys())
        out_idx = self.cutie.seed(det_frame, combined, alive_ids)

        # 記銘後のマスクで各トラックを更新
        for tid, t in self.tracks.items():
            m = out_idx == tid
            if m.sum() >= self.track_min_mask_area:
                t['mask'] = m
                t['box'] = self._box_from_mask(m)
                t['lost_frames'] = 0
            else:
                t['mask'] = m

    def frame_callback(self, msg: CompressedImage):
        t_start = time.perf_counter()
        # 入力: JPEG圧縮フレーム -> OpenCV(BGR)
        cv_image = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warning('フレームのデコードに失敗したためスキップします')
            return
        h, w = cv_image.shape[:2]

        msg_tracks = TrackArray()
        msg_tracks.header = msg.header
        # インデックスマスク: 各画素にトラックID(背景0)。IDは単調増加するため16bit
        idx_mask = np.zeros((h, w), dtype=np.uint16)

        # Cutie伝播は共有状態を保護
        t_lock = time.perf_counter()
        with self.state_lock:
            t_locked = time.perf_counter()
            # タイマー検出用に最新フレームを保持
            self.latest_bgr = cv_image
            self.latest_frame_msg = msg
            self._update_tracks_with_cutie(cv_image)
            t_cutie = time.perf_counter()
            for tid, t in self.tracks.items():
                tr = Track()
                tr.id = int(tid)
                tr.label = str(t['label'])
                tr.score = float(t['score'])
                x1, y1, x2, y2 = t['box']
                tr.x_min = int(x1); tr.y_min = int(y1)
                tr.x_max = int(x2); tr.y_max = int(y2)
                msg_tracks.tracks.append(tr)
                if t['mask'].shape == idx_mask.shape:
                    idx_mask[t['mask']] = tid

        self.tracks_pub.publish(msg_tracks)

        # マスクは可逆圧縮(16bit PNG)で送る。生データだとTCP越しでは重すぎるため
        ok, png = cv2.imencode('.png', idx_mask)
        if not ok:
            self.get_logger().warning('トラックマスクのPNGエンコードに失敗しました')
            return
        mask_msg = CompressedImage()
        mask_msg.header = msg.header
        mask_msg.format = 'mono16; png compressed'
        mask_msg.data = png.tobytes()
        self.track_mask_pub.publish(mask_msg)

        t_end = time.perf_counter()
        with self.stats_lock:
            self.stats['total'].append(t_end - t_start)
            self.stats['lock_wait'].append(t_locked - t_lock)
            self.stats['cutie'].append(t_cutie - t_locked)
            self.stats['encode'].append(t_end - t_cutie)

    def timer_callback(self):
        # 最新フレームがなければスキップ
        if self.latest_frame_msg is None:
            return

        now = time.time()
        with self.state_lock:
            if self.pending_stamp is not None:
                # 応答待ち中: タイムアウトしていなければ何もせず待つ
                elapsed = now - self.pending_since
                timeout = self.detection_interval_sec * self.detect_timeout_multiplier
                if elapsed < timeout:
                    return
                self.get_logger().warning(
                    f'検出応答がタイムアウトしました({elapsed:.1f}s > {timeout:.1f}s)。リクエストを再送します')
                self.pending_stamp = None
                self.pending_frame = None

            frame_msg = self.latest_frame_msg
            self.pending_frame = self.latest_bgr
            self.pending_stamp = frame_msg.header.stamp
            self.pending_since = now

        # lang_sam_detectorへ検出リクエストを送信
        # 受信したJPEGをそのまま転送する(再エンコード不要。stampも元フレームのまま
        # なので、クライアント側は検出結果を手元のフレームに重ねて描画できる)
        self.detect_request_pub.publish(frame_msg)

    def _on_detections(self, msg: DetectionArray):
        with self.state_lock:
            if self.pending_stamp is None or (
                    msg.header.stamp.sec != self.pending_stamp.sec
                    or msg.header.stamp.nanosec != self.pending_stamp.nanosec):
                # 送信していないリクエストへの応答、またはタイムアウト後の遅延応答
                self.logger.debug('対応するリクエストのない検出応答を破棄します')
                return
            det_frame = self.pending_frame
            self.pending_stamp = None
            self.pending_frame = None

        try:
            idx_mask = self.bridge.imgmsg_to_cv2(msg.index_mask, desired_encoding='mono16').astype(np.int32)
        except Exception as e:
            self.get_logger().error(f'index_maskのデコードに失敗: {e}')
            return

        n = len(msg.detections)
        labels_det = [d.label for d in msg.detections]
        scores_np = np.asarray([d.score for d in msg.detections], dtype=np.float32) if n > 0 \
            else np.zeros((0,), dtype=np.float32)
        h, w = idx_mask.shape
        masks_np = np.zeros((n, h, w), dtype=bool)
        for i in range(n):
            masks_np[i] = idx_mask == (i + 1)

        with self.state_lock:
            t_merge = time.perf_counter()
            # 既存トラックとIoUマッチングしてCutieへ記銘
            self._merge_detections(det_frame, labels_det, scores_np.tolist(), masks_np)
            self._add_stat('merges', time.perf_counter() - t_merge)


def main(args=None):
    rclpy.init(args=args)
    node = LangSamTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

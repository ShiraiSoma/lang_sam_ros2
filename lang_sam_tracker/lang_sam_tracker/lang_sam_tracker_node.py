#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage

from cv_bridge import CvBridge
from PIL import Image as PILImage
import torch
import numpy as np
import cv2
import threading
from concurrent.futures import ThreadPoolExecutor
import time

from lang_sam import LangSAM
from lang_sam.utils import draw_image  # 可視化ユーティリティ
from lang_sam.models.utils import DEVICE  # 推論デバイス（cuda/cpu取得）
from lang_sam_msgs.msg import TrackArray, Track # カスタムメッセージ
from lang_sam_tracker.cutie_tracker import CutieTracker  # Cutie(VOS)マスク伝播

class LangSamTrackerNode(Node):
    def __init__(self):
        super().__init__('lang_sam_tracker')
        self.logger = self.get_logger()
        self.logger.info('Initializing LangSAM Tracker Node...')

        # パラメータ宣言・取得
        self._setup_parameters()

        # 使用デバイス（外部ユーティリティから自動選択）
        self.device = DEVICE

        # LangSAMモデルのロード（GroundingDINO+SAMの複合推論）
        # 注意: sam2がHydraを初期化するためCutieより先にロードする
        self.model = LangSAM()

        # Cutie(VOS)トラッカーのロード
        # 検出マスクをメモリに記銘し、毎フレームのマスクを伝播で更新する
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
        # idはCutieのオブジェクトID（0は背景のため1始まり）
        self.tracks = {}
        self.next_track_id = 1
        self.latest_bgr = None  # タイマー検出用に最新フレームを保持

        # 共有状態ロックと検出用スレッドプール
        self.state_lock = threading.Lock()
        self.detector_pool = ThreadPoolExecutor(max_workers=1)
        # バックグラウンド検出の状態: None または Future
        self.det_future = None

        # I/O: 入力画像サブスク / 出力画像パブリッシュ
        self.image_sub = self.create_subscription(ROSImage, self.image_topic, self.image_callback, 1)
        self.tracks_pub = self.create_publisher(TrackArray, '/lang_sam/tracks', 1)

        # 可視化用共有イメージ（検出スレッド -> 表示）
        self.latest_det_vis = None  # OpenCV BGR image or None

        # トラッキングFPS計測
        self.track_last_time = time.time()
        self.track_fps = 0.0

        # 平滑化係数 (EMA) を両方で共通化
        self.fps_alpha = 0.1
        self.det_last_time = None
        self.det_fps = 0.0

        # 検出はタイマーで実行（detection_interval_secを周期として使用）
        self.detection_timer = self.create_timer(float(self.detection_interval_sec), self.timer_callback)

        # ログ
        self.get_logger().info(f'Using device: {self.device}')
        self.get_logger().info(f'Using SAM model: {self.sam_model}')
        self.get_logger().info(f'Using text prompt: {self.text_prompt}')
        self.get_logger().info(f'Detection interval (sec): {self.detection_interval_sec}')
        self.get_logger().info(f'Image topic: {self.image_topic}')
        self.get_logger().info('LangSAM + Cutie models initialized.')

    # パラメータ取得用の関数
    def _setup_parameters(self):
        # 注意: launch/config側で上書き可能
        self.declare_parameter('sam_model', 'sam2.1_hiera_small')
        self.declare_parameter('text_prompt', 'wheel. car.')
        self.declare_parameter('box_threshold', 0.3)
        self.declare_parameter('text_threshold', 0.25)
        self.declare_parameter('detection_interval_sec', 2.0)
        self.declare_parameter('image_topic', '/camera/image_raw')

        # Cutie(VOS)のROSパラメータ
        self.declare_parameter('cutie_weights', '')              # 空なら~/.cache/cutie/へ自動ダウンロード
        self.declare_parameter('cutie_max_internal_size', 480)   # 内部処理の最小辺(px)。小さいほど速い/粗い
        self.declare_parameter('cutie_mem_every', 5)              # メモリ記銘の間隔(フレーム)
        self.declare_parameter('cutie_use_long_term', True)       # 長期メモリ(長時間の追跡でメモリ量を抑制)
        self.declare_parameter('cutie_use_amp', False)             # 混合精度で高速化(精度僅かに低下)

        # トラック管理（検出とのマージ規則）のROSパラメータ
        self.declare_parameter('merge_iou_threshold', 0.3)          # 検出と既存トラックを同一とみなすマスクIoU
        self.declare_parameter('duplicate_overlap_threshold', 0.5)  # 同一物体の多重登録とみなす重なり率(交差/小さい方の面積)
        self.declare_parameter('track_max_detection_misses', 3)     # 再検出で連続この回数裏付けなし→削除
        self.declare_parameter('track_lost_frames', 90)             # 伝播マスクが空のフレーム数がこれを超えたら削除
        self.declare_parameter('track_min_mask_area', 100)          # これ未満の面積(px)のマスクは消失扱い(重複登録の残骸対策)

        self.sam_model = self.get_parameter('sam_model').get_parameter_value().string_value
        self.text_prompt = self.get_parameter('text_prompt').get_parameter_value().string_value
        self.box_threshold = self.get_parameter('box_threshold').get_parameter_value().double_value
        self.text_threshold = self.get_parameter('text_threshold').get_parameter_value().double_value
        self.detection_interval_sec = self.get_parameter('detection_interval_sec').get_parameter_value().double_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

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

    # --- EMA による FPS 更新の共通処理 ---
    def _update_ema_fps(self, now, last_name, fps_name):
        last = getattr(self, last_name, None)
        fps = getattr(self, fps_name, 0.0)
        if last is None:
            setattr(self, last_name, now)
            if fps == 0.0:
                setattr(self, fps_name, 0.0)
            return getattr(self, fps_name)
        dt = now - last
        if dt > 1e-6:
            inst = 1.0 / dt
            new = (1.0 - self.fps_alpha) * fps + self.fps_alpha * inst if fps > 0 else inst
            setattr(self, fps_name, new)
        setattr(self, last_name, now)
        return getattr(self, fps_name)

    @staticmethod
    def _box_from_mask(mask_bool):
        # マスクの外接矩形 [x1,y1,x2,y2] を返す（空マスクはNone）
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
        # ラベル比較用の正規化（末尾ピリオド・前後空白・大小文字を無視）
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
        # Cutieメモリとトラック辞書の両方から削除（state_lock保持前提）
        if not ids:
            return
        self.cutie.delete(ids)
        for tid in ids:
            self.tracks.pop(tid, None)

    def _update_tracks_with_cutie(self, cv_image):
        # Cutieのメモリに基づき現フレームへマスクを伝播し、各トラックのマスク/bboxを更新
        # （state_lock保持前提）
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
                # 伝播でマスクが消失（遮蔽/画面外など）。bboxは最後の位置を保持
                t['mask'] = m
                t['lost_frames'] += 1
                if t['lost_frames'] > self.track_lost_frames:
                    to_delete.append(tid)
        self._delete_tracks(to_delete)

    def _merge_detections(self, det_frame, labels, scores, masks_bool):
        # 再検出結果と既存トラックをマスクIoUでマッチングしてマージ（state_lock保持前提）
        # - マッチ: IDを維持しつつ新鮮なSAMマスクで矯正（ドリフト解消）
        # - 未マッチの検出: 新規トラック
        # - 未マッチのトラック: missカウント増、閾値超過で削除
        h, w = det_frame.shape[:2]
        get_label = lambda i: labels[i] if i < len(labels) else 'obj'

        # 空マスクの検出は除外（疑似マスクは作らない設計）
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

        # IoU降順の貪欲マッチング（ラベルが一致するトラックのみ対象）
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
        # - 検出マスクはスコア昇順に塗る（重なりは高スコアが優先）
        combined = np.zeros((h, w), dtype=np.int32)
        for tid, t in self.tracks.items():
            if tid not in matched_tids and tid not in det_to_tid.values():
                combined[t['mask']] = tid
        order = sorted(det_to_tid.keys(), key=lambda i: float(scores[i]) if i < len(scores) else 1.0)
        for i in order:
            combined[masks_bool[i]] = det_to_tid[i]

        # 検出フレームを正解としてメモリに記銘（初期化・矯正兼用）
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

    def image_callback(self, msg):
        # 入力: ROS Image -> OpenCV(BGR)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # タイマー検出用に最新フレームを保持
        self.latest_bgr = cv_image

        # Cutie伝播は共有状態を保護
        with self.state_lock:
            self._update_tracks_with_cutie(cv_image)
            # 可視化用にスナップショットを作る（ロック時間を短くするため必要最小限をコピー）
            if self.tracks:
                boxes_for_draw = np.asarray([t['box'] for t in self.tracks.values()], dtype=np.int32)
                labels_for_draw = [t['label'] for t in self.tracks.values()]
                scores_for_draw = np.asarray([t['score'] for t in self.tracks.values()], dtype=np.float32)
                masks_for_draw = np.asarray([t['mask'] for t in self.tracks.values()], dtype=bool)
            else:
                h, w, _ = cv_image.shape
                boxes_for_draw = np.zeros((0, 4), dtype=np.int32)
                labels_for_draw = []
                scores_for_draw = np.zeros((0,), dtype=np.float32)
                masks_for_draw = np.zeros((0, h, w), dtype=bool)

        pil_base = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        try:
            track_image_pil = draw_image(
                image_rgb=pil_base,
                masks=masks_for_draw,
                xyxy=boxes_for_draw,
                probs=scores_for_draw,
                labels=labels_for_draw,
            )
        except Exception as e:
            self.get_logger().warning(f'draw_image失敗(tracking): {e}')
            return

        track_image_cv = cv2.cvtColor(np.array(track_image_pil), cv2.COLOR_RGB2BGR)

        # --- 検出可視化と追跡可視化を横並びで表示（FPSを描画） ---
        # トラッキングFPS更新（image_callback の呼び出し周期を利用）
        now = time.time()
        # 共通ヘルパーでトラックFPSを更新
        self._update_ema_fps(now, 'track_last_time', 'track_fps')

        # 検出可視化をスレッドセーフに取得（同時にdet_fpsも読み出す）
        with self.state_lock:
            det_vis = None if self.latest_det_vis is None else self.latest_det_vis.copy()
            det_fps = float(self.det_fps)

        # 検出画像がない場合は空白を作る
        if det_vis is None:
            h_t, w_t, _ = track_image_cv.shape
            det_vis = np.zeros((h_t, w_t, 3), dtype=np.uint8)
            cv2.putText(det_vis, 'No detection yet', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)

        # --- 検出FPSをdet_vis上に描画（スタイルを統一） ---
        try:
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale_det = max(0.6, det_vis.shape[1] / 1000.0)
            thickness_det = 2
            det_text = f'FPS: {det_fps:.1f}'
            (tw, th), _ = cv2.getTextSize(det_text, font, font_scale_det, thickness_det)
            x_det = max(10, det_vis.shape[1] - 10 - tw)  # 右端に寄せ、最小余白を確保
            y_det = 10 + th  # 上から少し下げて描画
            cv2.putText(det_vis, det_text, (x_det, y_det), font, font_scale_det, (0, 255, 255), thickness_det, cv2.LINE_AA)
        except Exception:
            pass

        # --- トラッキングFPSをtrack_image_cv上に描画（同じスタイル） ---
        try:
            font_scale_trk = max(0.6, track_image_cv.shape[1] / 1000.0)
            thickness_trk = 2
            trk_text = f'FPS: {self.track_fps:.1f}'
            (tw_t, th_t), _ = cv2.getTextSize(trk_text, font, font_scale_trk, thickness_trk)
            x_trk = max(10, track_image_cv.shape[1] - 10 - tw_t)
            y_trk = 10 + th_t
            cv2.putText(track_image_cv, trk_text, (x_trk, y_trk), font, font_scale_trk, (0, 255, 255), thickness_trk, cv2.LINE_AA)
        except Exception:
            pass

        # サイズ合わせ（高さを基準に揃える）
        h_det, w_det, _ = det_vis.shape
        h_trk, w_trk, _ = track_image_cv.shape
        if h_det != h_trk:
            scale = h_trk / h_det
            new_w = int(w_det * scale)
            det_vis = cv2.resize(det_vis, (new_w, h_trk))
            w_det = new_w

        # 横並び合成（左右にラベル）
        combined = np.hstack([det_vis, track_image_cv])

        # ラベル描画: 左=Detection, 右=Tracking
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = max(0.6, combined.shape[1] / 1000.0)
        thickness = 2
        # Detectionラベル位置
        cv2.putText(combined, 'Detection', (10, 30), font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
        # Trackingラベル位置（右側の画像の左端を計算）
        x_right = det_vis.shape[1] + 10
        cv2.putText(combined, 'Tracking', (x_right, 30), font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)

        # 非ブロッキング表示
        cv2.imshow('LangSAM', combined)
        cv2.waitKey(1)

        # トラック情報を/custom_msgs/TrackArrayで配信（既存）
        msg_tracks = TrackArray()
        msg_tracks.header.stamp = self.get_clock().now().to_msg()
        msg_tracks.header.frame_id = 'camera'
        with self.state_lock:
            for tid, t in self.tracks.items():
                tr = Track()
                tr.id = int(tid)
                tr.label = str(t['label'])
                tr.score = float(t['score'])
                x1, y1, x2, y2 = t['box']
                tr.x_min = int(x1); tr.y_min = int(y1)
                tr.x_max = int(x2); tr.y_max = int(y2)
                msg_tracks.tracks.append(tr)
        self.tracks_pub.publish(msg_tracks)

    def timer_callback(self):
        # 最新フレームがなければスキップ
        if self.latest_bgr is None:
            return
        # 既にバックグラウンド推論が走っていれば重複起動しない (Futureベース)
        if self.det_future is not None and not self.det_future.done():
            return

        frame = self.latest_bgr.copy()
        # バックグラウンドで推論・描画・トラックマージ
        self.det_future = self.detector_pool.submit(self._run_detection_job, frame)

    def _run_detection_job(self, cv_image):
        try:
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            with torch.no_grad():
                # 元の重い処理（GPU/CPU同期もここで実施）
                results = self.model.predict([pil_image], [self.text_prompt])

            det = results[0]

            # boxes/masks/scores/labels を numpy に正規化
            boxes = self._to_numpy(det.get('boxes', None))
            boxes_np = np.zeros((0, 4), dtype=np.float32) if boxes is None else np.asarray(boxes, dtype=np.float32).reshape(-1, 4)

            h, w, _ = cv_image.shape
            masks = self._to_numpy(det.get('masks', None))
            if masks is None:
                masks_np = np.zeros((boxes_np.shape[0], h, w), dtype=bool)
            else:
                masks_np = np.asarray(masks)
                # normalize dims: expect (N, H, W) or (N,1,H,W)
                if masks_np.ndim == 4 and masks_np.shape[1] == 1:
                    masks_np = masks_np[:, 0]
                if masks_np.ndim == 2:
                    masks_np = masks_np[0:1, ...]
                if masks_np.ndim != 3:
                    masks_np = np.zeros((boxes_np.shape[0], h, w), dtype=bool)
                masks_np = masks_np.astype(bool)

            labels = det.get('labels', [])
            labels_det = [str(l) for l in labels] if len(labels) > 0 else []

            scores = self._to_numpy(det.get('scores', None))
            scores_np = np.zeros((boxes_np.shape[0],), dtype=np.float32) if scores is None else np.asarray(scores, dtype=np.float32).reshape(-1)

            # 検出可視化を作成
            det_image_pil = draw_image(
                image_rgb=pil_image,
                masks=masks_np,
                xyxy=boxes_np,
                probs=scores_np,
                labels=labels_det,
            )
            det_image_cv = cv2.cvtColor(np.array(det_image_pil), cv2.COLOR_RGB2BGR)

            # 検出完了時刻でdet_fpsを更新（スレッドセーフ、共通alphaを使用）および可視化/トラックマージ
            now = time.time()
            with self.state_lock:
                # 更新
                self._update_ema_fps(now, 'det_last_time', 'det_fps')
                # 可視化保存 (コピーして共有)
                self.latest_det_vis = det_image_cv.copy()
                # 既存トラックとIoUマッチングしてCutieへ記銘
                self._merge_detections(
                    cv_image,
                    labels_det,
                    scores_np.tolist(),
                    masks_np
                )
        except Exception as e:
            self.get_logger().error(f'バックグラウンド検出で例外: {e}')
        finally:
            # Future ベースなので特別にフラグを消す必要はないが参照を解放して GC を助ける
            try:
                self.det_future = None
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = LangSamTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # OpenCV ウィンドウを破棄
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()

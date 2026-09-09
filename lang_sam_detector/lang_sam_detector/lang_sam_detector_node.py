#!/usr/bin/env python3
"""LangSAM(GroundingDINO+SAM2)検出専用ノード

リモートPCで動作させることを想定。ローカル(ロボット)側のCutieトラッカーから
detect_requestトピックで届いた1枚の画像に対して検出を行い、結果を
detections(DetectionArray)としてそのまま返す。時刻同期は行わず、
リクエストのheader.stampをそのまま応答へコピーして相関を取る。
"""
import threading
import traceback
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from lang_sam import LangSAM
from lang_sam.utils import draw_image
from lang_sam.models.utils import DEVICE
from lang_sam_msgs.msg import Detection, DetectionArray


class LangSamDetectorNode(Node):
    def __init__(self):
        super().__init__('lang_sam_detector')
        self.logger = self.get_logger()
        self.logger.info('Initializing LangSAM Detector Node...')

        self._setup_parameters()

        self.device = DEVICE
        self.model = LangSAM(sam_type=self.sam_model)
        self.bridge = CvBridge()

        self._warmup()

        # 検出は同時に1件まで(処理中に届いたリクエストは読み捨てる。
        # ステートレスなので送信側のタイマーが次回再送してくれる)
        self.busy_lock = threading.Lock()
        self.busy = False
        self.worker_pool = ThreadPoolExecutor(max_workers=1)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.request_sub = self.create_subscription(
            CompressedImage, self.request_topic, self._on_request, qos)
        self.detections_pub = self.create_publisher(DetectionArray, self.response_topic, qos)

        self.get_logger().info(f'Using device: {self.device}')
        self.get_logger().info(f'Using SAM model: {self.sam_model}')
        self.get_logger().info(f'Using text prompt: {self.text_prompt}')
        self.get_logger().info(f'Request topic: {self.request_topic}')
        self.get_logger().info(f'Response topic: {self.response_topic}')
        self.get_logger().info('LangSAM Detector initialized.')

    def _warmup(self):
        """初回リクエストでCUDAカーネルのJITコンパイル等による遅延が
        出ないよう、起動時にダミー画像で一度推論しておく。失敗しても
        起動は継続する(ウォームアップは無くても動作自体には影響しない)。
        """
        try:
            dummy = PILImage.fromarray(np.zeros((480, 640, 3), dtype=np.uint8))
            with torch.no_grad():
                self.model.predict(
                    [dummy], [self.text_prompt],
                    box_threshold=self.box_threshold,
                    text_threshold=self.text_threshold,
                )
            self.logger.info('ウォームアップ推論が完了しました')
        except Exception:
            self.logger.warning(f'ウォームアップに失敗(起動は継続します):\n{traceback.format_exc()}')

    def _setup_parameters(self):
        self.declare_parameter('sam_model', 'sam2.1_hiera_small')
        self.declare_parameter('text_prompt', 'wheel. car.')
        self.declare_parameter('box_threshold', 0.3)
        self.declare_parameter('text_threshold', 0.25)
        self.declare_parameter('request_topic', '/lang_sam/detect_request')
        self.declare_parameter('response_topic', '/lang_sam/detections')
        self.declare_parameter('visualize', True)

        self.sam_model = self.get_parameter('sam_model').get_parameter_value().string_value
        self.text_prompt = self.get_parameter('text_prompt').get_parameter_value().string_value
        self.box_threshold = self.get_parameter('box_threshold').get_parameter_value().double_value
        self.text_threshold = self.get_parameter('text_threshold').get_parameter_value().double_value
        self.request_topic = self.get_parameter('request_topic').get_parameter_value().string_value
        self.response_topic = self.get_parameter('response_topic').get_parameter_value().string_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value

    @staticmethod
    def _to_numpy(x):
        if x is None:
            return None
        if hasattr(x, 'cpu'):
            try:
                x = x.cpu().numpy()
            except Exception:
                pass
        return np.asarray(x)

    def _on_request(self, msg: CompressedImage):
        with self.busy_lock:
            if self.busy:
                self.logger.debug('検出処理中のためリクエストを読み捨てます')
                return
            self.busy = True
        cv_image = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        self.worker_pool.submit(self._run_detection_job, cv_image, msg.header)

    def _run_detection_job(self, cv_image: np.ndarray, header):
        try:
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            with torch.no_grad():
                results = self.model.predict(
                    [pil_image], [self.text_prompt],
                    box_threshold=self.box_threshold,
                    text_threshold=self.text_threshold,
                )
            det = results[0]

            boxes = self._to_numpy(det.get('boxes', None))
            boxes_np = np.zeros((0, 4), dtype=np.float32) if boxes is None \
                else np.asarray(boxes, dtype=np.float32).reshape(-1, 4)

            h, w, _ = cv_image.shape
            masks = self._to_numpy(det.get('masks', None))
            if masks is None:
                masks_np = np.zeros((boxes_np.shape[0], h, w), dtype=bool)
            else:
                masks_np = np.asarray(masks)
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
            scores_np = np.zeros((boxes_np.shape[0],), dtype=np.float32) if scores is None \
                else np.asarray(scores, dtype=np.float32).reshape(-1)

            if self.visualize:
                try:
                    det_image_pil = draw_image(
                        image_rgb=pil_image, masks=masks_np, xyxy=boxes_np,
                        probs=scores_np, labels=labels_det,
                    )
                    det_image_cv = cv2.cvtColor(np.array(det_image_pil), cv2.COLOR_RGB2BGR)
                    cv2.imshow('LangSAM Detector', det_image_cv)
                    cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().warning(f'draw_image失敗(detection): {e}')

            self._publish_detections(header, labels_det, scores_np, boxes_np, masks_np, h, w)
        except Exception:
            self.get_logger().error(f'検出処理で例外:\n{traceback.format_exc()}')
            try:
                self.model.sam.predictor.reset_predictor()
            except Exception as e:
                self.get_logger().warning(f'predictorのリセットに失敗: {e}')
        finally:
            with self.busy_lock:
                self.busy = False

    def _publish_detections(self, header, labels, scores, boxes, masks_bool, h, w):
        n = masks_bool.shape[0]

        msg = DetectionArray()
        msg.header = header

        idx_mask = np.zeros((h, w), dtype=np.uint16)
        # スコア昇順に塗る(重なりは高スコアが最後に上書きされ優先される)
        order = sorted(range(n), key=lambda i: float(scores[i]))
        for i in order:
            idx_mask[masks_bool[i]] = i + 1

        for i in range(n):
            d = Detection()
            d.label = labels[i] if i < len(labels) else 'obj'
            d.score = float(scores[i])
            if masks_bool[i].any():
                ys, xs = np.where(masks_bool[i])
                d.x_min, d.y_min = int(xs.min()), int(ys.min())
                d.x_max, d.y_max = int(xs.max()), int(ys.max())
            else:
                x1, y1, x2, y2 = boxes[i].tolist()
                d.x_min, d.y_min, d.x_max, d.y_max = int(x1), int(y1), int(x2), int(y2)
            msg.detections.append(d)

        msg.index_mask = self.bridge.cv2_to_imgmsg(idx_mask, encoding='mono16')
        msg.index_mask.header = header
        self.detections_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LangSamDetectorNode()
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()

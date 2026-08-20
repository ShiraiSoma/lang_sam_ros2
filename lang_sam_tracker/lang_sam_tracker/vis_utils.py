"""可視化・デバイス選択ユーティリティ

元々はLangSAM本体(lang_sam.utils / lang_sam.models.utils)に含まれていたが、
LangSAM本体はlang_sam_detectorパッケージ(リモートPC)へ移設したため、
Cutieトラッキング側の可視化に必要な最小限のみここに複製する。
"""
import logging

import numpy as np
import supervision as sv
import torch


def get_device() -> torch.device:
    if torch.cuda.is_available():
        return torch.device('cuda')
    if torch.backends.mps.is_available():
        return torch.device('mps')
    logging.warning('No GPU found, using CPU instead')
    return torch.device('cpu')


def draw_image(image_rgb, masks, xyxy, probs, labels):
    box_annotator = sv.BoxCornerAnnotator()
    label_annotator = sv.LabelAnnotator()
    mask_annotator = sv.MaskAnnotator()
    unique_labels = list(set(labels))
    class_id_map = {label: idx for idx, label in enumerate(unique_labels)}
    class_id = [class_id_map[label] for label in labels]

    detections = sv.Detections(
        xyxy=xyxy,
        mask=masks.astype(bool),
        confidence=probs,
        class_id=np.array(class_id),
    )
    annotated_image = box_annotator.annotate(scene=image_rgb.copy(), detections=detections)
    annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections, labels=labels)
    annotated_image = mask_annotator.annotate(scene=annotated_image, detections=detections)
    return annotated_image

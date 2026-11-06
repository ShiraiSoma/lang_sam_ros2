"""可視化ユーティリティ

元々はLangSAM本体(lang_sam.utils)に含まれていたが、推論はリモート側で行い
描画のみをクライアント(ロボット側PC)で行うため、必要最小限をここに複製する。
torchに依存しないので、クライアント側にGPU・推論環境は不要。
"""
import numpy as np
import supervision as sv


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

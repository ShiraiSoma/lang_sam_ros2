#!/usr/bin/env python3
"""Cutie (VOS) によるマスク伝播トラッカーのラッパー

LangSAM(GroundingDINO+SAM2)の検出マスクを初期値として、
毎フレームのセグメンテーションマスクをCutieのメモリ機構で伝播する。
特徴点(GFTT+KLT)を使わないため、テクスチャレスな対象や変形にも頑健。

注意: sam2がimport時にHydraをグローバル初期化するため、公式の
cutie.utils.get_default_model は使えない(initialize()が衝突する)。
ここではGlobalHydraを手動で退避→クリアしてCutieのcfgを合成し、
終了後にsam2側の状態を復元する。
"""
import os

import cv2
import numpy as np
import torch
from hydra import compose, initialize_config_dir
from hydra.core.global_hydra import GlobalHydra
from omegaconf import open_dict

CUTIE_WEIGHTS_URL = "https://github.com/hkchengrex/Cutie/releases/download/v1.0/cutie-base-mega.pth"
DEFAULT_WEIGHTS_PATH = os.path.expanduser("~/.cache/cutie/cutie-base-mega.pth")


class CutieTracker:
    """マルチオブジェクトのストリーミングマスク伝播

    オブジェクトIDは正の整数(0は背景)。呼び出し側でIDを管理し、
    seed()で初期化/矯正、track()で毎フレーム伝播、delete()で破棄する。
    """

    def __init__(self,
                 device: str = "cuda",
                 weights_path: str | None = None,
                 max_internal_size: int = 480,
                 mem_every: int = 5,
                 use_long_term: bool = True,
                 use_amp: bool = False):
        import cutie as cutie_pkg
        from cutie.inference.inference_core import InferenceCore
        from cutie.model.cutie import CUTIE

        self.device = device
        self.use_amp = use_amp and str(device).startswith("cuda")

        weights_path = weights_path or DEFAULT_WEIGHTS_PATH
        if not os.path.exists(weights_path):
            os.makedirs(os.path.dirname(weights_path), exist_ok=True)
            torch.hub.download_url_to_file(CUTIE_WEIGHTS_URL, weights_path)

        config_dir = os.path.join(list(cutie_pkg.__path__)[0], "config")
        gh = GlobalHydra.instance()
        prev_hydra = gh.hydra if gh.is_initialized() else None
        gh.clear()
        try:
            with initialize_config_dir(version_base="1.3.2", config_dir=config_dir,
                                       job_name="cutie_streaming"):
                cfg = compose(config_name="eval_config")
        finally:
            # sam2等が初期化していたグローバル状態を復元
            # (hydraのコンテキストマネージャはGlobalHydraシングルトン自体を
            #  差し替えるため、instance()を取り直してから復元する)
            gh = GlobalHydra.instance()
            gh.clear()
            if prev_hydra is not None:
                gh.initialize(prev_hydra)
        with open_dict(cfg):
            cfg["weights"] = weights_path
            cfg["max_internal_size"] = int(max_internal_size)
            cfg["mem_every"] = int(mem_every)
            cfg["use_long_term"] = bool(use_long_term)

        self._cfg = cfg
        self._network = CUTIE(cfg).to(device).eval()
        self._network.load_weights(torch.load(weights_path, map_location=device))
        self._InferenceCore = InferenceCore
        self._processor = InferenceCore(self._network, cfg=cfg)

    # --- 内部ユーティリティ ---
    def _frame_to_tensor(self, frame_bgr: np.ndarray) -> torch.Tensor:
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        # (H,W,3) uint8 -> (3,H,W) float 0..1 (ImageNet正規化はCutieモデル内部で実施)
        return torch.from_numpy(rgb).to(self.device).permute(2, 0, 1).float() / 255.0

    def _step(self, image: torch.Tensor, mask=None, objects=None) -> np.ndarray:
        with torch.inference_mode():
            with torch.autocast(device_type="cuda", enabled=self.use_amp):
                prob = self._processor.step(image, mask, objects=objects)
            idx_mask = self._processor.output_prob_to_mask(prob)
        return idx_mask.cpu().numpy().astype(np.int32)

    # --- 公開API ---
    @property
    def object_ids(self) -> list[int]:
        return list(self._processor.object_manager.all_obj_ids)

    def seed(self, frame_bgr: np.ndarray, idx_mask: np.ndarray,
             object_ids: list[int]) -> np.ndarray:
        """このフレームのマスクを正解としてメモリに記銘する(初期化・矯正兼用)

        idx_mask: (H,W) 各画素にオブジェクトID(背景は0)
        object_ids: idx_mask中に存在する有効なID群
        戻り値: 伝播後の(H,W)インデックスマスク
        """
        image = self._frame_to_tensor(frame_bgr)
        mask = torch.from_numpy(idx_mask.astype(np.int64)).to(self.device)
        return self._step(image, mask, objects=[int(i) for i in object_ids])

    def track(self, frame_bgr: np.ndarray) -> np.ndarray:
        """メモリに基づいて現フレームへマスクを伝播する

        戻り値: (H,W)インデックスマスク(背景0)
        """
        image = self._frame_to_tensor(frame_bgr)
        return self._step(image)

    def delete(self, object_ids: list[int]):
        """指定オブジェクトをメモリから破棄する"""
        ids = [int(i) for i in object_ids if int(i) in self._processor.object_manager.all_obj_ids]
        if ids:
            self._processor.delete_objects(ids)

    def reset(self):
        """全オブジェクト・全メモリを破棄して初期状態に戻す"""
        self._processor = self._InferenceCore(self._network, cfg=self._cfg)

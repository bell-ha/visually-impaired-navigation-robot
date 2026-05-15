from __future__ import annotations
from pathlib import Path

import numpy as np
from ultralytics import YOLO


DEFAULT_MODEL = "yolov8n.pt"
PERSON_CLASS_ID = 0
DEFAULT_CONF = 0.5
DEFAULT_IOU = 0.5
MIN_BOX_HEIGHT = 80   # 픽셀 — 이보다 작은 박스는 사람으로 안 봄


class PersonTracker:
    def __init__(
        self,
        model_path: str = DEFAULT_MODEL,
        conf: float = DEFAULT_CONF,
        iou: float = DEFAULT_IOU,
        device: str = "cpu",
    ):
        self.model = YOLO(model_path)
        self.conf = conf
        self.iou = iou
        self.device = device
        self._tracker_cfg = "bytetrack.yaml"

    def update(self, frame: np.ndarray) -> list[tuple[float, float, float, float, int]]:
        results = self.model.track(
            frame,
            persist=True,
            tracker=self._tracker_cfg,
            conf=self.conf,
            iou=self.iou,
            classes=[PERSON_CLASS_ID],
            device=self.device,
            verbose=False,
        )

        tracks = []
        if results and results[0].boxes is not None:
            boxes = results[0].boxes
            if boxes.id is not None:
                xyxy = boxes.xyxy.cpu().numpy()
                ids = boxes.id.cpu().numpy().astype(int)
                for (x1, y1, x2, y2), tid in zip(xyxy, ids):
                    if (y2 - y1) >= MIN_BOX_HEIGHT:
                        tracks.append((float(x1), float(y1), float(x2), float(y2), int(tid)))

        return tracks

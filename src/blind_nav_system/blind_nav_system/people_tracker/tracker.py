from __future__ import annotations

import cv2
import numpy as np
from ultralytics import YOLO


DEFAULT_MODEL = "yolov8n.pt"
PERSON_CLASS_ID = 0
DEFAULT_CONF = 0.5
DEFAULT_IOU = 0.5
MIN_LONG_SIDE = 60    # 픽셀 — bbox 긴 변 기준 (카메라 회전 무관)


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

    def update(self, frame: np.ndarray, rotate_deg: int = 0) -> list[tuple[float, float, float, float, int]]:
        orig_h, orig_w = frame.shape[:2]

        # YOLO 입력을 올바른 방향으로 회전 (사람이 서있는 자세로 보이도록)
        if rotate_deg == 90:
            yolo_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotate_deg == 180:
            yolo_frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotate_deg == 270:
            yolo_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            yolo_frame = frame

        results = self.model.track(
            yolo_frame,
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
                    # 회전된 bbox를 원본 좌표계로 역변환 (depth 샘플링과 좌표 일치)
                    if rotate_deg == 90:
                        ox1, oy1, ox2, oy2 = y1, orig_h - 1 - x2, y2, orig_h - 1 - x1
                    elif rotate_deg == 180:
                        ox1, oy1, ox2, oy2 = orig_w - 1 - x2, orig_h - 1 - y2, orig_w - 1 - x1, orig_h - 1 - y1
                    elif rotate_deg == 270:
                        ox1, oy1, ox2, oy2 = orig_w - 1 - y2, x1, orig_w - 1 - y1, x2
                    else:
                        ox1, oy1, ox2, oy2 = x1, y1, x2, y2

                    if max(oy2 - oy1, ox2 - ox1) >= MIN_LONG_SIDE:
                        tracks.append((float(ox1), float(oy1), float(ox2), float(oy2), int(tid)))

        return tracks

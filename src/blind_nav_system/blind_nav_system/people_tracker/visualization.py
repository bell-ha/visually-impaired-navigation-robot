from __future__ import annotations
import math
import time

import cv2
import numpy as np

from direction import DirectionResult
from utils import get_color, draw_text_with_bg


ARROW_BASE_LEN = 60
ARROW_SPEED_SCALE = 8
ARROW_MAX_LEN = 120
ARROW_THICKNESS = 2
ARROW_TIP_RATIO = 0.35

CROWD_ARROW_LEN = 90
CROWD_ARROW_THICKNESS = 4
CROWD_ARROW_COLOR = (0, 255, 255)


class Visualizer:
    def __init__(self):
        self._fps_times: list[float] = []
        self._fps_window = 30

    def draw(
        self,
        frame: np.ndarray,
        tracks: list[tuple],
        directions: dict[int, DirectionResult],
        crowd_flow: dict | None,
    ) -> np.ndarray:
        t = time.perf_counter()
        out = frame.copy()

        for det in tracks:
            x1, y1, x2, y2, tid = det
            self._draw_person(out, int(x1), int(y1), int(x2), int(y2), int(tid),
                              directions.get(int(tid)))

        if crowd_flow is not None:
            self._draw_crowd_flow(out, crowd_flow)

        self._draw_hud(out, crowd_flow, tracks)
        self._draw_fps(out, t)
        return out

    def _draw_person(self, img, x1, y1, x2, y2, tid, dr):
        color = get_color(tid)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        draw_text_with_bg(img, f"ID:{tid}", (x1, y1 - 6), font_scale=0.55,
                          thickness=2, fg=color, bg=(0, 0, 0))

        if dr is None or not dr.is_moving:
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.circle(img, (cx, cy), 5, (180, 180, 180), -1)
            return

        bx = (x1 + x2) // 2
        by = y2
        arrow_len = min(ARROW_BASE_LEN + dr.spd * ARROW_SPEED_SCALE, ARROW_MAX_LEN)
        norm = math.sqrt(dr.vx ** 2 + dr.vy ** 2) + 1e-6
        ex = int(bx + (dr.vx / norm) * arrow_len)
        ey = int(by + (dr.vy / norm) * arrow_len)

        cv2.arrowedLine(img, (bx, by), (ex, ey), color, ARROW_THICKNESS,
                        tipLength=ARROW_TIP_RATIO, line_type=cv2.LINE_AA)
        draw_text_with_bg(img, f"{dr.angle:.0f}deg  {dr.spd:.1f}px/f",
                          (x1, y2 + 14), font_scale=0.42,
                          fg=(220, 220, 220), bg=(0, 0, 0))

    def _draw_crowd_flow(self, img, cf):
        h, w = img.shape[:2]
        cx, cy = w // 2, h - 60
        angle_rad = math.radians(cf["mean_angle"])
        ex = int(cx + math.cos(angle_rad) * CROWD_ARROW_LEN)
        ey = int(cy - math.sin(angle_rad) * CROWD_ARROW_LEN)
        cv2.arrowedLine(img, (cx, cy), (ex, ey), CROWD_ARROW_COLOR,
                        CROWD_ARROW_THICKNESS, tipLength=0.3, line_type=cv2.LINE_AA)
        draw_text_with_bg(img, f"CROWD FLOW  {cf['mean_angle']:.0f}deg",
                          (cx - 70, cy + 22), font_scale=0.55,
                          fg=CROWD_ARROW_COLOR, bg=(0, 0, 0))

    def _draw_hud(self, img, cf, tracks):
        total = len(tracks)
        moving = cf["n_moving"] if cf else 0
        draw_text_with_bg(img, f"People: {total}  Moving: {moving}",
                          (10, 24), font_scale=0.6, thickness=2,
                          fg=(255, 255, 255), bg=(30, 30, 30))

    def _draw_fps(self, img, start_t):
        self._fps_times.append(time.perf_counter() - start_t)
        if len(self._fps_times) > self._fps_window:
            self._fps_times.pop(0)
        avg = sum(self._fps_times) / len(self._fps_times)
        fps = 1.0 / avg if avg > 0 else 0
        draw_text_with_bg(img, f"FPS:{fps:.1f}", (10, 50),
                          font_scale=0.55, fg=(0, 255, 128), bg=(0, 0, 0))

from __future__ import annotations
import math
from collections import deque

import numpy as np

from direction import DirectionResult
from utils import angle_degrees, speed


N_BINS = 8
TEMPORAL_LEN = 15  # 5fps 기준 ~3초 평균 — 빠르게 접근하는 군중에 빠르게 반응


class CrowdFlowEstimator:
    def __init__(self, temporal_len: int = TEMPORAL_LEN):
        self._history: deque[tuple[float, float]] = deque(maxlen=temporal_len)

    def update(self, directions: list[DirectionResult]) -> dict | None:
        moving = [d for d in directions if d.is_moving]
        if not moving:
            return None

        vxs = np.array([d.vx for d in moving])
        vys = np.array([d.vy for d in moving])

        mean_vx = float(np.mean(vxs))
        mean_vy = float(np.mean(vys))
        self._history.append((mean_vx, mean_vy))

        hist = np.array(self._history)
        smoothed_vx = float(np.mean(hist[:, 0]))
        smoothed_vy = float(np.mean(hist[:, 1]))
        spd = speed(smoothed_vx, smoothed_vy)

        angles = [d.angle for d in moving]
        dominant_angle = self._dominant_angle(angles)

        return {
            "mean_vx": smoothed_vx,
            "mean_vy": smoothed_vy,
            "mean_angle": angle_degrees(smoothed_vx, smoothed_vy),
            "dominant_angle": dominant_angle,
            "mean_speed": spd,
            "n_moving": len(moving),
            "n_total": len(directions),
        }

    @staticmethod
    def _dominant_angle(angles: list[float]) -> float:
        if not angles:
            return 0.0
        bin_size = 360 / N_BINS
        bins = [0] * N_BINS
        for a in angles:
            idx = int(((a + 180) % 360) / bin_size) % N_BINS
            bins[idx] += 1
        max_bin = bins.index(max(bins))
        center = max_bin * bin_size + bin_size / 2 - 180
        return center

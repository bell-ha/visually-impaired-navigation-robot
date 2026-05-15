from __future__ import annotations
import math
from collections import deque, defaultdict
from dataclasses import dataclass, field

import numpy as np

from utils import angle_degrees, speed


HISTORY_LEN = 12
MIN_HISTORY = 3
EMA_ALPHA = 0.35
MIN_SPEED_PX = 0.8
MAX_ANGLE_JUMP_DEG = 60


@dataclass
class TrackState:
    history: deque = field(default_factory=lambda: deque(maxlen=HISTORY_LEN))
    smoothed_vx: float = 0.0
    smoothed_vy: float = 0.0
    prev_angle: float | None = None
    is_moving: bool = False


@dataclass
class DirectionResult:
    track_id: int
    vx: float
    vy: float
    angle: float
    spd: float
    is_moving: bool


class DirectionEstimator:
    def __init__(
        self,
        history_len: int = HISTORY_LEN,
        min_history: int = MIN_HISTORY,
        ema_alpha: float = EMA_ALPHA,
        min_speed_px: float = MIN_SPEED_PX,
        max_angle_jump: float = MAX_ANGLE_JUMP_DEG,
    ):
        self.history_len = history_len
        self.min_history = min_history
        self.ema_alpha = ema_alpha
        self.min_speed_px = min_speed_px
        self.max_angle_jump = max_angle_jump
        self._states: defaultdict[int, TrackState] = defaultdict(TrackState)

    def update(self, track_id: int, cx: float, cy: float) -> DirectionResult:
        state = self._states[track_id]
        state.history.append((cx, cy))

        if len(state.history) < self.min_history:
            return DirectionResult(track_id, 0, 0, 0, 0, False)

        raw_vx, raw_vy = self._regress_velocity(state.history)
        spd = speed(raw_vx, raw_vy)

        a = self.ema_alpha
        state.smoothed_vx = a * raw_vx + (1 - a) * state.smoothed_vx
        state.smoothed_vy = a * raw_vy + (1 - a) * state.smoothed_vy

        smoothed_spd = speed(state.smoothed_vx, state.smoothed_vy)
        is_moving = smoothed_spd >= self.min_speed_px

        raw_angle = angle_degrees(state.smoothed_vx, state.smoothed_vy)
        angle = self._clamp_angle(raw_angle, state.prev_angle)
        if is_moving:
            state.prev_angle = angle

        state.is_moving = is_moving
        return DirectionResult(
            track_id=track_id,
            vx=state.smoothed_vx,
            vy=state.smoothed_vy,
            angle=angle,
            spd=smoothed_spd,
            is_moving=is_moving,
        )

    def cleanup(self, active_ids: set[int]) -> None:
        stale = [tid for tid in self._states if tid not in active_ids]
        for tid in stale:
            del self._states[tid]

    @staticmethod
    def _regress_velocity(history: deque) -> tuple[float, float]:
        pts = np.array(history, dtype=float)
        n = len(pts)
        t = np.arange(n, dtype=float)
        vx = float(np.polyfit(t, pts[:, 0], 1)[0])
        vy = float(np.polyfit(t, pts[:, 1], 1)[0])
        return vx, vy

    def _clamp_angle(self, new_angle: float, prev_angle: float | None) -> float:
        if prev_angle is None:
            return new_angle
        delta = abs(new_angle - prev_angle)
        if delta > 180:
            delta = 360 - delta
        if delta > self.max_angle_jump:
            return prev_angle
        return new_angle

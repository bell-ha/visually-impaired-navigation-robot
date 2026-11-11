"""층별 좌표 찍기 도구들이 같이 쓰는 것 — 지도 읽기, 로봇 모양 벽 검사, 엘리베이터 기준점.

검사 규칙은 make_point_tool.py 가 만드는 웹 도구(JS)와 **같아야** 한다.
한쪽을 바꾸면 다른 쪽도 같이 바꿀 것.
"""

import math
import os

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, "../../.."))          # src/blind_nav_system
MAPS = os.path.join(PKG, "maps")
LOCATION_YAML = os.path.join(PKG, "config/location.yaml")
FLOORS = ["floor1", "floor2", "floor3", "floor4", "floor5"]

# map_server(negate 0) 해석: ≤89 벽, >205 길, 그 사이 모름 (occupied 0.65 / free 0.196)
OCC_MAX = 89
UNKNOWN_MAX = 205

# 로봇 + 뒤따르는 사용자 모양 (로봇 기준 x 앞, y 왼쪽, m).
# nav2_params_human.yaml footprint [[0.175,-0.175],[0.175,0.475],[-0.9,0.475],[-0.9,-0.175]]
# + footprint_padding 0.08. 뒤 0.98 m 꼬리 = 손잡이를 잡고 따라오는 사람 자리.
FP_FRONT, FP_REAR, FP_LEFT, FP_RIGHT = 0.255, -0.98, 0.555, -0.255
WARN_MIN = 0.10      # 모양에서 벽까지 이보다 가까우면 주의
WARN_CENTER = 0.50   # 중심에서 벽까지 이보다 가까우면 주의

# 엘리베이터 기준점 (floor5 좌표계 = 다섯 층 공통, 2026-11-11 이후)
ELEV = [("탑승지점", -47.454, 4.993), ("문앞(②⑥)", -48.228, 5.224),
        ("하차지점", -48.232, 4.932), ("캐빈(④)", -47.819, 7.057)]


def load_map(floor):
    """(배열, origin(x, y), resolution) — yaml 은 origin/resolution 만 읽는다."""
    meta = {}
    for line in open(os.path.join(MAPS, f"{floor}.yaml"), encoding="utf8"):
        if ":" in line:
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    o = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    a = np.array(Image.open(os.path.join(MAPS, meta.get("image", f"{floor}.pgm"))))
    return a, o, float(meta["resolution"])


class WallCheck:
    """벽까지 거리 지도(scipy EDT)를 한 번 만들어 두고 자세마다 검사한다."""

    def __init__(self, floor):
        from scipy.ndimage import distance_transform_edt
        self.a, self.o, self.res = load_map(floor)
        self.h, self.w = self.a.shape
        self.dist = distance_transform_edt(self.a > OCC_MAX) * self.res   # m, 벽 칸 = 0

    def cell(self, x, y):
        c = round((x - self.o[0]) / self.res)
        r = round(self.h - 1 - (y - self.o[1]) / self.res)
        if 0 <= r < self.h and 0 <= c < self.w:
            return r, c
        return None

    def check(self, x, y, yaw_deg):
        """'ok' / 'warn' / 'bad', 모양-벽 최소거리, 중심-벽 거리.
        모양 안을 5 cm 간격으로 훑는다. 지도 밖이나 벽 칸에 닿으면 bad."""
        a = math.radians(yaw_deg)
        ca, sa = math.cos(a), math.sin(a)
        min_d, hit = 999.0, False
        for fx in np.arange(FP_REAR, FP_FRONT + 1e-3, 0.05):
            for fy in np.arange(FP_RIGHT, FP_LEFT + 1e-3, 0.05):
                rc = self.cell(x + fx * ca - fy * sa, y + fx * sa + fy * ca)
                if rc is None:
                    hit = True
                    continue
                d = self.dist[rc]
                if d <= 1e-3:
                    hit = True
                min_d = min(min_d, d)
        rc = self.cell(x, y)
        center = self.dist[rc] if rc else 0.0
        if hit:
            return "bad", 0.0, center
        st = "warn" if (min_d < WARN_MIN or center < WARN_CENTER) else "ok"
        return st, float(min_d), float(center)


def yaw_to_zw(yaw_deg):
    """로봇은 수평 → z = sin(yaw/2), w = cos(yaw/2) (coord_util.py 와 같은 규약)."""
    h = math.radians(yaw_deg) / 2
    return math.sin(h), math.cos(h)

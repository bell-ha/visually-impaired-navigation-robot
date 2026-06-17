from __future__ import annotations
import math
import threading
from collections import defaultdict, deque

import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage, CameraInfo
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from builtin_interfaces.msg import Duration
import tf2_ros
import tf2_geometry_msgs

from direction import DirectionResult
from flow import CrowdFlowEstimator

DEPTH_TOPIC        = "/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC         = "/camera/camera/color/camera_info"
MARKER_TOPIC       = "/people_tracker/markers"
CROWD_FLOW_TOPIC   = "/people_tracker/crowd_flow"
TARGET_FRAME       = "map"

HISTORY_LEN  = 8   # 빠른 방향 반응
MIN_HISTORY  = 2   # 2프레임부터 속도 계산 — 사람 등장 즉시 화살표
MAP_MIN_SPEED = 0.02  # m/frame — 이 이하는 정지로 판정


class PeopleMarkerPublisher(Node):
    """
    Depth + TF2로 사람 위치를 맵 좌표로 변환 후 RViz2 MarkerArray 퍼블리시.
    픽셀 기반 direction이 아니라 실제 3D 이동 벡터를 사용한다.
    """

    def __init__(self):
        super().__init__("people_marker_publisher")

        self._depth: np.ndarray | None = None
        self._frame_id = "camera_color_optical_frame"
        self._fx = self._fy = self._cx = self._cy = None
        self._lock = threading.Lock()

        # 맵 좌표 위치 이력 (3D 속도 계산용)
        self._pos_history: defaultdict[int, deque] = defaultdict(
            lambda: deque(maxlen=HISTORY_LEN)
        )

        self.create_subscription(ROSImage,    DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(CameraInfo,  INFO_TOPIC,  self._info_cb,  10)

        self._pub        = self.create_publisher(MarkerArray,      MARKER_TOPIC,     10)
        self._crowd_pub  = self.create_publisher(Float32MultiArray, CROWD_FLOW_TOPIC, 10)
        self._crowd_flow_est = CrowdFlowEstimator()

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(f"구독: {DEPTH_TOPIC}")
        self.get_logger().info(f"퍼블리시: {MARKER_TOPIC}, {CROWD_FLOW_TOPIC}")

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: ROSImage):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            (msg.height, msg.width))
        with self._lock:
            self._depth   = arr.copy()
            self._frame_id = msg.header.frame_id

    def _info_cb(self, msg: CameraInfo):
        with self._lock:
            K = msg.k
            self._fx, self._cx = K[0], K[2]
            self._fy, self._cy = K[4], K[5]

    # ── 핵심 계산 ──────────────────────────────────────────────────────────────

    def _sample_depth(self, u: int, v: int) -> float | None:
        """픽셀 (u,v) 주변 패치의 중앙값 깊이(m). 7×7 → 15×15 → 25×25 순서로 확장 탐색."""
        with self._lock:
            if self._depth is None:
                return None
            depth = self._depth.copy()
        h, w = depth.shape
        u = max(0, min(u, w - 1))
        v = max(0, min(v, h - 1))
        for r in (3, 7, 12):
            patch = depth[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
            valid = patch[patch > 0]
            if len(valid) >= 5:
                D = float(np.median(valid)) / 1000.0
                return D if 0.1 < D < 6.0 else None  # 6m 초과는 D435i 노이즈 구간
        return None

    def _deproject(self, u: int, v: int, D: float) -> tuple[float, float, float]:
        """픽셀 + 깊이 → 카메라 프레임 3D 좌표."""
        with self._lock:
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
        X = (u - cx) * D / fx
        Y = (v - cy) * D / fy
        return X, Y, D

    def _to_map(self, x: float, y: float, z: float) -> tuple[float, float, float] | None:
        """카메라 프레임 → map 프레임 변환."""
        try:
            pt = PointStamped()
            pt.header.frame_id = self._frame_id
            pt.header.stamp    = self.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = x, y, z
            out = self._tf_buffer.transform(
                pt, TARGET_FRAME,
                timeout=rclpy.duration.Duration(seconds=0.3))
            return out.point.x, out.point.y, out.point.z
        except Exception:
            return None

    def _velocity_2d(self, tid: int, mx: float, my: float) -> tuple[float, float] | None:
        """맵 좌표 이력으로 2D 속도(m/frame) 계산. 이력 부족 시 None."""
        hist = self._pos_history[tid]
        hist.append((mx, my))
        if len(hist) < MIN_HISTORY:
            return None
        pts = np.array(hist, dtype=float)
        t   = np.arange(len(pts), dtype=float)
        vx  = float(np.polyfit(t, pts[:, 0], 1)[0])
        vy  = float(np.polyfit(t, pts[:, 1], 1)[0])
        return vx, vy

    # ── 퍼블리시 ───────────────────────────────────────────────────────────────

    def publish(self, tracks: list, active_ids: set[int]):
        """매 프레임 tracker 결과를 받아 마커를 퍼블리시."""
        # 사라진 트랙 이력 정리
        stale = [tid for tid in self._pos_history if tid not in active_ids]
        for tid in stale:
            del self._pos_history[tid]

        with self._lock:
            ready = self._fx is not None and self._depth is not None
        if not ready:
            return

        now     = self.get_clock().now().to_msg()
        markers = []

        # 이전 마커 전체 삭제
        clr = Marker()
        clr.action      = Marker.DELETEALL
        clr.header.frame_id = TARGET_FRAME
        markers.append(clr)

        uid = 0
        map_dir_results = []
        for x1, y1, x2, y2, tid in tracks:
            # 가슴 위치 (bbox 상단 1/3) — 바닥보다 depth 안정적
            bx = int((x1 + x2) / 2)
            by = int(y1 + (y2 - y1) * 0.33)

            D = self._sample_depth(bx, by)
            if D is None:
                continue

            cam_xyz = self._deproject(bx, by, D)
            map_xyz = self._to_map(*cam_xyz)
            if map_xyz is None:
                continue

            mx, my, _ = map_xyz

            # 사람 실린더
            cyl           = Marker()
            cyl.header.frame_id = TARGET_FRAME
            cyl.header.stamp    = now
            cyl.ns, cyl.id      = "people", uid; uid += 1
            cyl.type            = Marker.CYLINDER
            cyl.action          = Marker.ADD
            cyl.pose.position.x = mx
            cyl.pose.position.y = my
            cyl.pose.position.z = 0.9          # 허리 높이
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = cyl.scale.y = 0.5   # 직경
            cyl.scale.z = 1.8                  # 키
            cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = 0.2, 0.6, 1.0, 0.7
            cyl.lifetime = Duration(sec=1)
            markers.append(cyl)

            # ID 텍스트
            txt           = Marker()
            txt.header.frame_id = TARGET_FRAME
            txt.header.stamp    = now
            txt.ns, txt.id      = "ids", uid; uid += 1
            txt.type            = Marker.TEXT_VIEW_FACING
            txt.action          = Marker.ADD
            txt.pose.position.x = mx
            txt.pose.position.y = my
            txt.pose.position.z = 2.1
            txt.pose.orientation.w = 1.0
            txt.scale.z         = 0.35
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text            = f"ID:{tid}  {D:.1f}m"
            txt.lifetime        = Duration(sec=1)
            markers.append(txt)

            # 방향 화살표 (맵 좌표 기반) + crowd_flow 집계용 DirectionResult 수집
            vel = self._velocity_2d(tid, mx, my)
            if vel is not None:
                vx, vy = vel
                spd = (vx**2 + vy**2) ** 0.5
                is_moving = spd > MAP_MIN_SPEED
                map_dir_results.append(DirectionResult(
                    track_id=tid,
                    vx=vx, vy=vy,
                    angle=math.degrees(math.atan2(vy, vx)),
                    spd=spd,
                    is_moving=is_moving,
                ))
                if spd > 0.02:                 # 정지 판정 임계값 (m/frame)
                    arrow_len = min(0.5 + spd * 10, 2.0)
                    norm = spd
                    ex = mx + (vx / norm) * arrow_len
                    ey = my + (vy / norm) * arrow_len

                    arr           = Marker()
                    arr.header.frame_id = TARGET_FRAME
                    arr.header.stamp    = now
                    arr.ns, arr.id      = "directions", uid; uid += 1
                    arr.type            = Marker.ARROW
                    arr.action          = Marker.ADD
                    arr.points          = [
                        Point(x=mx, y=my, z=0.1),
                        Point(x=ex, y=ey, z=0.1),
                    ]
                    arr.scale.x = 0.06   # 축 지름
                    arr.scale.y = 0.15   # 화살머리 지름
                    arr.color.r, arr.color.g, arr.color.b, arr.color.a = 1.0, 0.5, 0.0, 0.9
                    arr.lifetime = Duration(sec=1)
                    markers.append(arr)

        msg = MarkerArray()
        msg.markers = markers
        self._pub.publish(msg)

        # crowd_flow 계산 후 퍼블리시
        crowd_flow = self._crowd_flow_est.update(map_dir_results)
        if crowd_flow is not None:
            cf = Float32MultiArray()
            cf.data = [
                float(crowd_flow['dominant_angle']),
                float(crowd_flow['n_moving']),
                float(crowd_flow['mean_speed']),
            ]
            self._crowd_pub.publish(cf)

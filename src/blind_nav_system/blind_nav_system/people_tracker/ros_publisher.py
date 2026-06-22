from __future__ import annotations
import json
import math
import threading
from collections import defaultdict, deque

import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from builtin_interfaces.msg import Duration
import tf2_ros
import tf2_geometry_msgs

DEPTH_TOPIC        = "/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC         = "/camera/camera/color/camera_info"
MARKER_TOPIC       = "/people_tracker/markers"
PEOPLE_TOPIC       = "/people_tracker/people"
TARGET_FRAME       = "map"

HISTORY_LEN    = 8    # 빠른 방향 반응
MIN_HISTORY    = 2    # depth 정확 시 최소 이력
MIN_HISTORY_BBOX = 4  # bbox 추정 시 최소 이력 (노이즈 보정)
MAP_MIN_SPEED  = 0.02 # m/frame — 이 이하는 정지로 판정

# bbox 거리 추정 (depth 실패 시 fallback)
PERSON_HEIGHT_M    = 1.7   # 평균 사람 키 (m)
BBOX_DEPTH_MAX     = 9.0   # bbox 추정 허용 최대 거리 (m)
BBOX_MIN_HEIGHT_PX = 15    # 이 픽셀 이하는 너무 멀어서 신뢰 불가


class PeopleMarkerPublisher(Node):
    """
    Depth + TF2로 사람 위치를 맵 좌표로 변환 후 RViz2 MarkerArray 퍼블리시.
    픽셀 기반 direction이 아니라 실제 3D 이동 벡터를 사용한다.
    """

    def __init__(self):
        super().__init__("people_marker_publisher")

        self._depth: np.ndarray | None = None
        self._depth_stamp = None          # depth 메시지 타임스탬프 (TF 조회에 사용)
        self._frame_id = "camera_color_optical_frame"
        self._fx = self._fy = self._cx = self._cy = None
        self._lock = threading.Lock()

        # 맵 좌표 위치 이력 (3D 속도 계산용)
        self._pos_history: defaultdict[int, deque] = defaultdict(
            lambda: deque(maxlen=HISTORY_LEN)
        )

        self.create_subscription(ROSImage,    DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(CameraInfo,  INFO_TOPIC,  self._info_cb,  10)

        self._pub        = self.create_publisher(MarkerArray, MARKER_TOPIC,  10)
        self._people_pub = self.create_publisher(String,      PEOPLE_TOPIC, 10)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(f"구독: {DEPTH_TOPIC}")
        self.get_logger().info(f"퍼블리시: {MARKER_TOPIC}, {PEOPLE_TOPIC}")

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: ROSImage):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            (msg.height, msg.width))
        with self._lock:
            self._depth       = arr.copy()
            self._depth_stamp = msg.header.stamp
            self._frame_id    = msg.header.frame_id

    def _info_cb(self, msg: CameraInfo):
        with self._lock:
            K = msg.k
            self._fx, self._cx = K[0], K[2]
            self._fy, self._cy = K[4], K[5]

    # ── 핵심 계산 ──────────────────────────────────────────────────────────────

    def _bbox_depth_estimate(self, y1: float, y2: float) -> float | None:
        """bbox 높이로 거리 추정. depth 실패 시 fallback."""
        with self._lock:
            fy = self._fy
        if fy is None or fy <= 0:
            return None
        h_px = y2 - y1
        if h_px < BBOX_MIN_HEIGHT_PX:
            return None
        D = (PERSON_HEIGHT_M * fy) / h_px
        return D if 0.5 < D < BBOX_DEPTH_MAX else None

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
            with self._lock:
                stamp    = self._depth_stamp
                frame_id = self._frame_id
            pt = PointStamped()
            pt.header.frame_id = frame_id
            # depth 메시지 timestamp 사용 — 로봇 이동 중 위치 오차 방지
            pt.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = x, y, z
            out = self._tf_buffer.transform(
                pt, TARGET_FRAME,
                timeout=rclpy.duration.Duration(seconds=0.05))
            return out.point.x, out.point.y, out.point.z
        except Exception:
            return None

    def _robot_pose(self) -> tuple[float, float, float] | None:
        """TF2에서 map 기준 로봇 (x, y, yaw_deg) 반환."""
        try:
            trans = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            q  = trans.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return rx, ry, math.degrees(math.atan2(siny, cosy))
        except Exception:
            return None

    def _robot_yaw_deg(self) -> float | None:
        pose = self._robot_pose()
        return pose[2] if pose else None

    def _velocity_2d(self, tid: int, mx: float, my: float,
                     min_hist: int = MIN_HISTORY) -> tuple[float, float] | None:
        """맵 좌표 이력으로 2D 속도(m/frame) 계산. 이력 부족 시 None."""
        hist = self._pos_history[tid]
        if hist:
            px, py = hist[-1]
            if ((mx - px) ** 2 + (my - py) ** 2) ** 0.5 > 1.0:
                # 튀는 값 감지 → 이력 리셋 후 새 기준점으로 시작
                hist.clear()
        hist.append((mx, my))
        if len(hist) < min_hist:
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
            self.get_logger().warn(
                "depth 또는 camera_info 미수신 — 토픽 확인 필요",
                throttle_duration_sec=5.0,
            )
            return

        now     = self.get_clock().now().to_msg()
        markers = []

        # 이전 마커 전체 삭제
        clr = Marker()
        clr.action      = Marker.DELETEALL
        clr.header.frame_id = TARGET_FRAME
        markers.append(clr)

        robot_pose = self._robot_pose()
        robot_yaw  = robot_pose[2] if robot_pose else None
        robot_x    = robot_pose[0] if robot_pose else None
        robot_y    = robot_pose[1] if robot_pose else None

        uid = 0
        people_data = []
        for x1, y1, x2, y2, tid in tracks:
            # 가슴 위치 (bbox 상단 1/3) — 바닥보다 depth 안정적
            bx = int((x1 + x2) / 2)
            by = int(y1 + (y2 - y1) * 0.33)

            # depth 정밀 측정 → 실패 시 bbox 높이로 거리 추정
            D = self._sample_depth(bx, by)
            depth_estimated = D is None
            if depth_estimated:
                D = self._bbox_depth_estimate(y1, y2)
            if D is None:
                continue

            cam_xyz = self._deproject(bx, by, D)
            map_xyz = self._to_map(*cam_xyz)
            if map_xyz is None:
                continue

            mx, my, _ = map_xyz

            # bbox 추정 시 더 많은 이력을 쌓은 후 속도 계산 (노이즈 보정)
            min_hist = MIN_HISTORY_BBOX if depth_estimated else MIN_HISTORY
            vel = self._velocity_2d(tid, mx, my, min_hist)
            vx = vy = spd = 0.0
            if vel is not None:
                vx, vy = vel
                spd = (vx**2 + vy**2) ** 0.5

            # 로봇 위치 기준 거리 (TF 없으면 -1)
            if robot_x is not None and robot_y is not None:
                dist = round(((mx - robot_x)**2 + (my - robot_y)**2) ** 0.5, 2)
            else:
                dist = -1.0

            # 접근자(빨강) / 동행자(초록) / 미분류(파랑) 분류
            if vel is not None and robot_yaw is not None and spd > MAP_MIN_SPEED:
                person_angle = math.degrees(math.atan2(vy, vx))
                diff = abs(person_angle - robot_yaw)
                if diff > 180:
                    diff = 360.0 - diff
                if diff > 90:
                    classification = 'approaching'
                    cr, cg, cb, ca = 1.0, 0.2, 0.2, (0.5 if depth_estimated else 0.8)
                else:
                    classification = 'companion'
                    cr, cg, cb, ca = 0.2, 0.9, 0.2, (0.4 if depth_estimated else 0.7)
            else:
                classification = 'unknown'
                cr, cg, cb, ca = 0.2, 0.6, 1.0, (0.3 if depth_estimated else 0.7)

            people_data.append({
                'id': tid,
                'x': round(mx, 3), 'y': round(my, 3),
                'vx': round(vx, 4), 'vy': round(vy, 4),
                'classification': classification,
                'distance': dist,
                'depth_estimated': depth_estimated,
            })

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
            cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = cr, cg, cb, ca
            cyl.lifetime = Duration(sec=1)
            markers.append(cyl)

            # ID 텍스트 (bbox 추정 시 ~ 표시)
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
            txt.text            = f"ID:{tid}  {'~' if depth_estimated else ''}{D:.1f}m"
            txt.lifetime        = Duration(sec=1)
            markers.append(txt)

            # 방향 화살표
            if vel is not None and spd > MAP_MIN_SPEED:
                arrow_len = min(0.5 + spd * 10, 2.0)
                ex = mx + (vx / spd) * arrow_len
                ey = my + (vy / spd) * arrow_len

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

        # 분류된 사람 정보 퍼블리시 (navigation_modifier가 구독)
        payload = String()
        payload.data = json.dumps({
            'people': people_data,
            'robot_x': round(robot_x, 3) if robot_x is not None else None,
            'robot_y': round(robot_y, 3) if robot_y is not None else None,
            'robot_yaw': round(robot_yaw, 2) if robot_yaw is not None else None,
        })
        self._people_pub.publish(payload)

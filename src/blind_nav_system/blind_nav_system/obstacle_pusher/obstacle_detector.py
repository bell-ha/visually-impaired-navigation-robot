from __future__ import annotations
import json
import math
import threading

import cv2
import numpy as np
import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import tf2_ros
import tf2_geometry_msgs

# ── 토픽 ──────────────────────────────────────────────────────────────────────
DEPTH_TOPIC  = "/camera/camera/aligned_depth_to_color/image_raw"
RGB_TOPIC    = "/camera/camera/color/image_raw"
INFO_TOPIC   = "/camera/camera/color/camera_info"
OBJ_TOPIC    = "/obstacle_pusher/objects"
MARKER_TOPIC = "/obstacle_pusher/markers"
TARGET_FRAME = "map"

# ── YOLO ──────────────────────────────────────────────────────────────────────
YOLO_MODEL     = "yolov8n.pt"
YOLO_CONF      = 0.35
CHAIR_CLASS_ID = 56

# ── ROI (depth 유효 범위) ──────────────────────────────────────────────────────
ROI_FRONT_MIN = 0.3    # m
ROI_FRONT_MAX = 3.0    # m

PUBLISH_RATE = 3.0     # Hz


class ObstacleDetector(Node):
    """
    YOLO로 의자(class 56) 감지 → depth에서 3D 위치 계산 → 퍼블리시.
    Open3D/RANSAC/DBSCAN 없이 경량 동작.
    """

    def __init__(self, rotate_deg: int = 90):
        super().__init__("obstacle_detector")

        self._rotate_deg = rotate_deg
        self._depth: np.ndarray | None = None
        self._rgb:   np.ndarray | None = None
        self._fx = self._fy = self._cx = self._cy = None
        self._frame_id = "camera_color_optical_frame"
        self._lock = threading.Lock()

        try:
            from ultralytics import YOLO as _YOLO
            self._yolo = _YOLO(YOLO_MODEL)
            self.get_logger().info(f"YOLO 로드: {YOLO_MODEL}")
        except Exception as e:
            self._yolo = None
            self.get_logger().warn(f"YOLO 로드 실패 ({e})")

        self.create_subscription(ROSImage,   DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(ROSImage,   RGB_TOPIC,   self._rgb_cb,   10)
        self.create_subscription(CameraInfo, INFO_TOPIC,  self._info_cb,  10)

        self._obj_pub    = self.create_publisher(String,      OBJ_TOPIC,    10)
        self._marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info("ObstacleDetector 시작 (YOLO-first 경량 모드)")

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: ROSImage):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
        with self._lock:
            self._depth    = arr.copy()
            self._frame_id = msg.header.frame_id

    def _rgb_cb(self, msg: ROSImage):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        with self._lock:
            self._rgb = arr.copy()

    def _info_cb(self, msg: CameraInfo):
        with self._lock:
            K = msg.k
            self._fx, self._cx = K[0], K[2]
            self._fy, self._cy = K[4], K[5]

    # ── 공개 API ──────────────────────────────────────────────────────────────

    def process_frame(self) -> np.ndarray | None:
        with self._lock:
            if self._depth is None or self._fx is None or self._rgb is None:
                return None
            depth    = self._depth.copy()
            rgb      = self._rgb.copy()
            fx, fy   = self._fx, self._fy
            cx, cy   = self._cx, self._cy
            frame_id = self._frame_id
        return self._process(depth, rgb, fx, fy, cx, cy, frame_id)

    # ── 핵심 처리 ─────────────────────────────────────────────────────────────

    def _process(self, depth, rgb, fx, fy, cx, cy, frame_id):
        # 1. YOLO 1회 실행 — 의자만 추출
        chair_boxes = self._yolo_detect_chairs(rgb)

        if not chair_boxes:
            self._publish_empty()
            return self._draw_cv2(depth, rgb, [])

        # 2. 각 bbox에서 depth로 3D 위치 계산
        objects   = []
        draw_list = []   # (rx1, ry1, rx2, ry2, dist, conf)
        markers   = []
        clr = Marker()
        clr.action = Marker.DELETEALL
        clr.header.frame_id = TARGET_FRAME
        markers.append(clr)
        now = self.get_clock().now().to_msg()
        uid = 0

        for (rx1, ry1, rx2, ry2, conf) in chair_boxes:
            pos = self._bbox_to_3d(depth, rx1, ry1, rx2, ry2, fx, fy, cx, cy)
            if pos is None:
                continue
            cam_x, cam_y, cam_z, w_m, h_m = pos

            if not (ROI_FRONT_MIN <= cam_z <= ROI_FRONT_MAX):
                continue

            map_xyz = self._to_map(cam_x, cam_y, cam_z, frame_id)
            if map_xyz is None:
                continue
            map_x, map_y, _ = map_xyz

            cam_angle_deg = math.degrees(math.atan2(cam_x, cam_z))
            obj = {
                "map_x":         round(map_x,         3),
                "map_y":         round(map_y,         3),
                "distance":      round(cam_z,         2),
                "cam_angle_deg": round(cam_angle_deg, 1),
                "width":         round(w_m,           3),
                "height":        round(h_m,           3),
                "depth":         round(w_m,           3),
                "volume":        round(w_m * h_m * w_m, 4),
                "pushable":      "unknown",
            }
            objects.append(obj)
            draw_list.append((rx1, ry1, rx2, ry2, cam_z, conf))

            # RViz 마커
            box_m = Marker()
            box_m.header.frame_id = TARGET_FRAME
            box_m.header.stamp    = now
            box_m.ns, box_m.id    = "boxes", uid; uid += 1
            box_m.type            = Marker.CUBE
            box_m.action          = Marker.ADD
            box_m.pose.position.x = map_x
            box_m.pose.position.y = map_y
            box_m.pose.position.z = h_m / 2
            box_m.pose.orientation.w = 1.0
            box_m.scale.x = max(w_m, 0.1)
            box_m.scale.y = max(w_m, 0.1)
            box_m.scale.z = max(h_m, 0.1)
            box_m.color.r, box_m.color.g, box_m.color.b, box_m.color.a = 1.0, 0.9, 0.1, 0.7
            box_m.lifetime = Duration(sec=6)
            markers.append(box_m)

            txt = Marker()
            txt.header.frame_id = TARGET_FRAME
            txt.header.stamp    = now
            txt.ns, txt.id      = "box_labels", uid; uid += 1
            txt.type            = Marker.TEXT_VIEW_FACING
            txt.action          = Marker.ADD
            txt.pose.position.x = map_x
            txt.pose.position.y = map_y
            txt.pose.position.z = h_m + 0.2
            txt.pose.orientation.w = 1.0
            txt.scale.z         = 0.22
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f"[PROBE?] {cam_z:.1f}m  conf:{conf:.2f}"
            txt.lifetime = Duration(sec=6)
            markers.append(txt)

        msg = String()
        msg.data = json.dumps({"objects": objects})
        self._obj_pub.publish(msg)

        ma = MarkerArray(); ma.markers = markers
        self._marker_pub.publish(ma)

        if objects:
            self.get_logger().info(
                f"의자 {len(objects)}개: " +
                ", ".join(f"{o['distance']:.1f}m" for o in objects))

        return self._draw_cv2(depth, rgb, draw_list)

    # ── YOLO 의자 감지 ────────────────────────────────────────────────────────

    def _yolo_detect_chairs(self, rgb) -> list[tuple[int,int,int,int,float]]:
        """RGB → YOLO 1회 → 의자(class 56) bbox 리스트 반환. 좌표는 회전된 이미지 기준."""
        if self._yolo is None or rgb is None:
            return []
        rgb_rot = self._rotate_img(rgb)
        results = self._yolo(rgb_rot, verbose=False, conf=YOLO_CONF)[0]
        chairs = []
        for box in results.boxes:
            if int(box.cls[0]) != CHAIR_CLASS_ID:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            chairs.append((x1, y1, x2, y2, float(box.conf[0])))
        return chairs

    # ── bbox + depth → 3D 위치 ─────────────────────────────────────────────────

    def _bbox_to_3d(self, depth, rx1, ry1, rx2, ry2, fx, fy, cx, cy):
        """
        YOLO bbox (회전된 이미지 좌표) + depth → 카메라 좌표계 3D 위치.
        depth는 원본(비회전) 이미지이므로 bbox 좌표를 역변환 후 샘플링.
        """
        H_orig, W_orig = depth.shape

        if self._rotate_deg == 90:
            # 90° CW 역변환: rotated(x,y) → orig(u,v) = (y, H-1-x)
            u1, u2 = ry1, ry2
            v1, v2 = H_orig - 1 - rx2, H_orig - 1 - rx1
        elif self._rotate_deg == 270:
            # 270° CW 역변환: rotated(x,y) → orig(u,v) = (W-1-y, x)
            u1, u2 = W_orig - 1 - ry2, W_orig - 1 - ry1
            v1, v2 = rx1, rx2
        elif self._rotate_deg == 180:
            u1, u2 = W_orig - 1 - rx2, W_orig - 1 - rx1
            v1, v2 = H_orig - 1 - ry2, H_orig - 1 - ry1
        else:
            u1, v1, u2, v2 = rx1, ry1, rx2, ry2

        u1 = int(np.clip(u1, 0, W_orig - 1))
        u2 = int(np.clip(u2, 0, W_orig - 1))
        v1 = int(np.clip(v1, 0, H_orig - 1))
        v2 = int(np.clip(v2, 0, H_orig - 1))
        if u2 <= u1 or v2 <= v1:
            return None

        roi = depth[v1:v2+1, u1:u2+1].astype(np.float32) / 1000.0
        valid = roi[(roi > ROI_FRONT_MIN) & (roi < ROI_FRONT_MAX)]
        if len(valid) < 10:
            return None

        # 20 퍼센타일 = 의자 앞면 (배경 제외)
        Z = float(np.percentile(valid, 20))
        u_c = (u1 + u2) / 2.0
        v_c = (v1 + v2) / 2.0
        X = (u_c - cx) * Z / fx
        Y = (v_c - cy) * Z / fy

        w_m = abs(u2 - u1) * Z / fx
        h_m = abs(v2 - v1) * Z / fy

        return X, Y, Z, w_m, h_m

    # ── 시각화 ────────────────────────────────────────────────────────────────

    def _draw_cv2(self, depth, rgb, draw_list):
        # Color 프레임
        if rgb is None:
            color_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            color_frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            color_frame = self._rotate_img(color_frame)

        for (rx1, ry1, rx2, ry2, dist, conf) in draw_list:
            cv2.rectangle(color_frame, (rx1, ry1), (rx2, ry2), (50, 210, 230), 2)
            cv2.putText(color_frame, f"CHAIR {dist:.1f}m ({conf:.0%})",
                        (rx1, max(ry1 - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (50, 210, 230), 2)

        # Depth 컬러맵
        if depth is not None:
            d_vis = np.clip(depth.astype(np.float32) / 4000.0 * 255, 0, 255).astype(np.uint8)
            depth_frame = cv2.applyColorMap(d_vis, cv2.COLORMAP_JET)
            depth_frame = self._rotate_img(depth_frame)
            for (rx1, ry1, rx2, ry2, dist, _) in draw_list:
                cv2.rectangle(depth_frame, (rx1, ry1), (rx2, ry2), (255, 255, 255), 2)
        else:
            depth_frame = np.zeros_like(color_frame)

        # 같은 높이로 맞춰 나란히
        ch, cw = color_frame.shape[:2]
        dh, dw = depth_frame.shape[:2]
        if ch != dh:
            depth_frame = cv2.resize(depth_frame, (int(dw * ch / dh), ch))
        combined = np.hstack([color_frame, depth_frame])

        cv2.putText(combined,
                    f"Chairs:{len(draw_list)}  {ROI_FRONT_MIN}-{ROI_FRONT_MAX}m  rot:{self._rotate_deg}",
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        return combined

    # ── 유틸 ──────────────────────────────────────────────────────────────────

    def _rotate_img(self, img: np.ndarray) -> np.ndarray:
        if self._rotate_deg == 90:
            return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        if self._rotate_deg == 270:
            return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self._rotate_deg == 180:
            return cv2.rotate(img, cv2.ROTATE_180)
        return img

    def _to_map(self, x, y, z, frame_id) -> tuple | None:
        try:
            pt = PointStamped()
            pt.header.frame_id = frame_id
            pt.header.stamp    = self.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = float(x), float(y), float(z)
            out = self._tf_buffer.transform(
                pt, TARGET_FRAME,
                timeout=rclpy.duration.Duration(seconds=0.3))
            return out.point.x, out.point.y, out.point.z
        except Exception:
            return None

    def _publish_empty(self):
        msg = String(); msg.data = json.dumps({"objects": []})
        self._obj_pub.publish(msg)
        ma = MarkerArray()
        clr = Marker(); clr.action = Marker.DELETEALL; clr.header.frame_id = TARGET_FRAME
        ma.markers = [clr]
        self._marker_pub.publish(ma)

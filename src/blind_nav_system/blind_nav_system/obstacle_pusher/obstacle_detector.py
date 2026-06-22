from __future__ import annotations
import json
import threading

import cv2
import numpy as np
import open3d as o3d
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

# ── YOLO 파라미터 ─────────────────────────────────────────────────────────────
YOLO_MODEL   = "yolov8n.pt"
YOLO_CONF    = 0.3
YOLO_IOU_THR = 0.15   # depth 클러스터 투영 박스와 YOLO 박스 최소 IoU

# 무겁거나 고정된 물체 → 즉시 우회 (COCO class ID)
NOT_PUSHABLE_IDS = {
    56,   # chair
    57,   # couch
    58,   # potted plant
    59,   # bed
    60,   # dining table
    61,   # toilet
    62,   # tv
    63,   # laptop
    68,   # microwave
    69,   # oven
    71,   # sink
    72,   # refrigerator
}

# 가벼운 물체 → 바로 밀기 (COCO class ID)
PUSHABLE_IDS = {
    32,   # sports ball
    39,   # bottle
    41,   # cup
    45,   # bowl
    65,   # remote
    73,   # book
    77,   # teddy bear
}

# ── ROI 파라미터 ──────────────────────────────────────────────────────────────
ROI_FRONT_MIN = 0.3    # m
ROI_FRONT_MAX = 2.5    # m
ROI_SIDE_MAX  = 0.6    # m

# ── RANSAC 평면 제거 ──────────────────────────────────────────────────────────
PLANE_DIST_THRESH = 0.03
PLANE_RANSAC_N    = 3
PLANE_RANSAC_ITER = 500

# ── 유클리드 클러스터링 ───────────────────────────────────────────────────────
CLUSTER_EPS     = 0.07
CLUSTER_MIN_PTS = 30
CLUSTER_MAX_PTS = 5000

# ── 박스 크기 기준 ────────────────────────────────────────────────────────────
BOX_MIN_SIDE = 0.05   # m
BOX_MAX_SIDE = 0.60   # m
BOX_MAX_VOL  = 0.10   # m³

PUBLISH_RATE = 5.0    # Hz (depth 처리가 무거우므로 5fps)


class ObstacleDetector(Node):
    """
    aligned_depth → open3d 포인트클라우드 → RANSAC 평면 제거
    → 유클리드 클러스터링 → YOLO 분류 → /obstacle_pusher/objects 퍼블리시

    pushable 필드 (push_probe.py가 참조):
      "pushable"     — YOLO가 가벼운 물체로 인식  → probe 없이 바로 밀기
      "not_pushable" — YOLO가 가구/고정물로 인식  → 즉시 우회
      "unknown"      — YOLO 미인식 (박스 등)       → probe push로 물리적 판단
    """

    def __init__(self, rotate_deg: int = 90):
        super().__init__("obstacle_detector")

        self._rotate_deg = rotate_deg   # 화면 표시용 회전 (카메라 물리 장착 각도)
        self._depth: np.ndarray | None = None
        self._rgb: np.ndarray | None   = None
        self._frame_id = "camera_color_optical_frame"
        self._fx = self._fy = self._cx = self._cy = None
        self._lock = threading.Lock()

        # YOLO 로드 실패해도 depth 전용 모드로 동작
        try:
            from ultralytics import YOLO as _YOLO
            self._yolo = _YOLO(YOLO_MODEL)
            self.get_logger().info(f"YOLO 모델 로드: {YOLO_MODEL}")
        except Exception as e:
            self._yolo = None
            self.get_logger().warn(f"YOLO 로드 실패 ({e}) → depth 전용 모드 (모든 물체 unknown)")

        self.create_subscription(ROSImage,   DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(ROSImage,   RGB_TOPIC,   self._rgb_cb,   10)
        self.create_subscription(CameraInfo, INFO_TOPIC,  self._info_cb,  10)

        self._obj_pub    = self.create_publisher(String,      OBJ_TOPIC,    10)
        self._marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info("ObstacleDetector 시작")
        self.get_logger().info(f"ROI: 전방 {ROI_FRONT_MIN}~{ROI_FRONT_MAX}m, 측면 ±{ROI_SIDE_MAX}m")

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: ROSImage):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            (msg.height, msg.width))
        with self._lock:
            self._depth    = arr.copy()
            self._frame_id = msg.header.frame_id

    def _rgb_cb(self, msg: ROSImage):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        with self._lock:
            self._rgb = arr.copy()

    def _info_cb(self, msg: CameraInfo):
        with self._lock:
            K = msg.k
            self._fx, self._cx = K[0], K[2]
            self._fy, self._cy = K[4], K[5]

    # ── 회전 유틸 ─────────────────────────────────────────────────────────────

    def _rotate_img(self, img: np.ndarray) -> np.ndarray:
        """카메라 장착 각도에 맞게 이미지 회전."""
        if self._rotate_deg == 90:
            return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        if self._rotate_deg == 270:
            return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self._rotate_deg == 180:
            return cv2.rotate(img, cv2.ROTATE_180)
        return img

    def _rotate_for_yolo(self, rgb, u_raw, v_raw, H, W):
        """
        RGB 회전 + 클러스터 투영 좌표 변환.
        회전된 이미지 좌표계에서 IoU 매칭이 가능하도록 좌표도 같이 변환.
        """
        rgb_rot = self._rotate_img(rgb)
        if self._rotate_deg == 90:
            # (u, v) → (H-1-v, u)  in rotated frame (W=H_orig, H=W_orig)
            u_r = H - 1 - v_raw
            v_r = u_raw
        elif self._rotate_deg == 270:
            # (u, v) → (v, W-1-u)
            u_r = v_raw
            v_r = W - 1 - u_raw
        elif self._rotate_deg == 180:
            u_r = W - 1 - u_raw
            v_r = H - 1 - v_raw
        else:
            u_r, v_r = u_raw, v_raw
        rH, rW = rgb_rot.shape[:2]
        u_r = np.clip(u_r, 0, rW - 1)
        v_r = np.clip(v_r, 0, rH - 1)
        return rgb_rot, int(u_r.min()), int(v_r.min()), int(u_r.max()), int(v_r.max())

    # ── YOLO 분류 ─────────────────────────────────────────────────────────────

    def _yolo_classify(self, cluster_pts_cam: np.ndarray, fx, fy, cx, cy) -> str:
        """
        클러스터 포인트(카메라 좌표) → 이미지 투영 → YOLO IoU 매칭
        반환: "pushable" | "not_pushable" | "unknown"
        """
        if self._yolo is None:
            return "unknown"
        with self._lock:
            if self._rgb is None:
                return "unknown"
            rgb = self._rgb.copy()

        z = cluster_pts_cam[:, 2]
        valid = z > 0.01
        if valid.sum() < 5:
            return "unknown"
        pts = cluster_pts_cam[valid]
        H, W = rgb.shape[:2]

        # 클러스터 포인트 → 원본 이미지 좌표 투영
        u_raw = np.clip((fx * pts[:, 0] / pts[:, 2] + cx).astype(int), 0, W - 1)
        v_raw = np.clip((fy * pts[:, 1] / pts[:, 2] + cy).astype(int), 0, H - 1)

        # 카메라 회전에 맞게 YOLO용 이미지와 투영 좌표 변환
        rgb_yolo, u1, v1, u2, v2 = self._rotate_for_yolo(
            rgb, u_raw, v_raw, H, W)
        if u2 - u1 < 5 or v2 - v1 < 5:
            return "unknown"

        # YOLO 추론
        results = self._yolo(rgb_yolo, verbose=False, conf=YOLO_CONF)[0]

        best_cls = None
        best_iou = 0.0
        for box in results.boxes:
            bx1, by1, bx2, by2 = map(int, box.xyxy[0])
            # depth 클러스터 투영 박스 vs YOLO 탐지 박스 IoU
            ix1 = max(u1, bx1); iy1 = max(v1, by1)
            ix2 = min(u2, bx2); iy2 = min(v2, by2)
            inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
            if inter == 0:
                continue
            union = (u2-u1)*(v2-v1) + (bx2-bx1)*(by2-by1) - inter
            iou = inter / max(union, 1)
            if iou > best_iou:
                best_iou = iou
                best_cls = int(box.cls[0])

        if best_cls is None or best_iou < YOLO_IOU_THR:
            return "unknown"

        cls_name = results.names.get(best_cls, str(best_cls))
        if best_cls in NOT_PUSHABLE_IDS:
            self.get_logger().info(f"YOLO: {cls_name} → not_pushable")
            return "not_pushable"
        elif best_cls in PUSHABLE_IDS:
            self.get_logger().info(f"YOLO: {cls_name} → pushable")
            return "pushable"
        else:
            self.get_logger().debug(f"YOLO: {cls_name} → unknown (PUSHABLE/NOT_PUSHABLE 목록 외)")
            return "unknown"

    # ── 공개 API (메인 루프에서 호출) ─────────────────────────────────────────

    def get_latest_data(self):
        """메인 루프에서 최신 프레임/파라미터를 꺼낸다. 준비 안 됐으면 None."""
        with self._lock:
            if self._depth is None or self._fx is None:
                return None
            return (
                self._depth.copy(),
                self._rgb.copy() if self._rgb is not None else None,
                self._fx, self._fy, self._cx, self._cy,
                self._frame_id,
            )

    def process_frame(self) -> np.ndarray | None:
        """
        메인 스레드에서 직접 호출.
        최신 depth로 RANSAC+DBSCAN+YOLO 처리 후 렌더링된 BGR 프레임 반환.
        준비 안 됐으면 None 반환.
        """
        data = self.get_latest_data()
        if data is None:
            return None
        depth, rgb, fx, fy, cx, cy, frame_id = data
        return self._process(depth, rgb, fx, fy, cx, cy, frame_id)

    # ── 핵심 처리 (내부) ───────────────────────────────────────────────────────

    def _process(self, depth, rgb, fx, fy, cx, cy, frame_id):
        """실제 처리 — process_frame()에서 호출."""

        # 1. depth → 포인트클라우드
        pcd = self._depth_to_pcd(depth, fx, fy, cx, cy)
        if len(pcd.points) < 100:
            return

        # 2. ROI 필터
        pts = np.asarray(pcd.points)
        mask = (
            (pts[:, 2] >= ROI_FRONT_MIN) &
            (pts[:, 2] <= ROI_FRONT_MAX) &
            (np.abs(pts[:, 0]) <= ROI_SIDE_MAX)
        )
        if mask.sum() < 50:
            return
        roi_pcd = pcd.select_by_index(np.where(mask)[0])

        # 3. RANSAC 평면 제거 (바닥 + 벽)
        remaining = roi_pcd
        for _ in range(3):
            if len(remaining.points) < 50:
                break
            _, inliers = remaining.segment_plane(
                distance_threshold=PLANE_DIST_THRESH,
                ransac_n=PLANE_RANSAC_N,
                num_iterations=PLANE_RANSAC_ITER,
            )
            if len(inliers) / len(remaining.points) > 0.6:
                remaining = remaining.select_by_index(inliers, invert=True)
            else:
                break

        if len(remaining.points) < CLUSTER_MIN_PTS:
            self._publish_empty()
            return

        # 4. 유클리드 클러스터링
        labels = np.array(
            remaining.cluster_dbscan(
                eps=CLUSTER_EPS,
                min_points=CLUSTER_MIN_PTS,
                print_progress=False,
            )
        )

        objects   = []
        markers   = []
        draw_list = []   # (cluster_pts_cam, pushable, dist, volume, label_text)
        uid = 0
        clr = Marker(); clr.action = Marker.DELETEALL; clr.header.frame_id = TARGET_FRAME
        markers.append(clr)
        now = self.get_clock().now().to_msg()

        for label in set(labels):
            if label < 0:
                continue
            cluster_pts = np.asarray(remaining.points)[labels == label]
            n = len(cluster_pts)
            if not (CLUSTER_MIN_PTS <= n <= CLUSTER_MAX_PTS):
                continue

            mn, mx_pt = cluster_pts.min(axis=0), cluster_pts.max(axis=0)
            dims       = mx_pt - mn
            center_cam = (mn + mx_pt) / 2

            w, h, d = abs(dims[0]), abs(dims[1]), abs(dims[2])
            volume  = w * h * d

            # 크기 필터
            sides = sorted([w, h, d])
            if sides[0] < BOX_MIN_SIDE or sides[2] > BOX_MAX_SIDE or volume > BOX_MAX_VOL:
                continue

            # 맵 좌표 변환
            map_xyz = self._to_map(*center_cam, frame_id)
            if map_xyz is None:
                continue
            map_x, map_y, _ = map_xyz

            # 5. YOLO 분류
            pushable = self._yolo_classify(cluster_pts, fx, fy, cx, cy)

            label_text = {
                "pushable":     "PUSH",
                "not_pushable": "DETOUR",
                "unknown":      "PROBE?",
            }.get(pushable, "?")

            obj = {
                "map_x":    round(map_x,   3),
                "map_y":    round(map_y,   3),
                "distance": round(float(center_cam[2]), 2),
                "width":    round(w, 3),
                "height":   round(h, 3),
                "depth":    round(d, 3),
                "volume":   round(volume, 4),
                "n_pts":    int(n),
                "pushable": pushable,
            }
            objects.append(obj)
            draw_list.append((cluster_pts, pushable, float(center_cam[2]), volume, label_text))

            # RViz: 초록=pushable, 빨강=not_pushable, 노랑=unknown
            color_map = {
                "pushable":     (0.2, 1.0, 0.4),
                "not_pushable": (1.0, 0.3, 0.2),
                "unknown":      (1.0, 0.9, 0.1),
            }
            cr, cg, cb = color_map.get(pushable, (1.0, 0.9, 0.1))

            box_m = Marker()
            box_m.header.frame_id = TARGET_FRAME
            box_m.header.stamp    = now
            box_m.ns, box_m.id    = "boxes", uid; uid += 1
            box_m.type            = Marker.CUBE
            box_m.action          = Marker.ADD
            box_m.pose.position.x = map_x
            box_m.pose.position.y = map_y
            box_m.pose.position.z = h / 2
            box_m.pose.orientation.w = 1.0
            box_m.scale.x = w; box_m.scale.y = d; box_m.scale.z = h
            box_m.color.r, box_m.color.g, box_m.color.b, box_m.color.a = cr, cg, cb, 0.7
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
            txt.pose.position.z = h + 0.2
            txt.pose.orientation.w = 1.0
            txt.scale.z         = 0.22
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f"[{label_text}] {volume*1000:.1f}L  {float(center_cam[2]):.1f}m"
            txt.lifetime = Duration(sec=6)
            markers.append(txt)

        # 퍼블리시
        msg = String()
        msg.data = json.dumps({"objects": objects})
        self._obj_pub.publish(msg)

        ma = MarkerArray(); ma.markers = markers
        self._marker_pub.publish(ma)

        if objects:
            summary = ", ".join(
                f"{o['pushable']}@{o['distance']:.1f}m" for o in objects)
            self.get_logger().info(f"박스 후보 {len(objects)}개: {summary}")

        return self._draw_cv2(rgb, draw_list, fx, fy, cx, cy)

    # ── OpenCV 시각화 ──────────────────────────────────────────────────────────

    def _draw_cv2(self, rgb, draw_list, fx, fy, cx, cy):
        if rgb is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "No RGB", (20, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 100, 100), 2)
        else:
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # BGR 색상: 초록=push, 빨강=detour, 노랑=probe
        color_bgr = {
            "pushable":     (50,  220, 50),
            "not_pushable": (50,  50,  220),
            "unknown":      (50,  210, 230),
        }

        for cluster_pts, pushable, dist, volume, label in draw_list:
            Z = cluster_pts[:, 2]
            valid = Z > 0.01
            if valid.sum() < 3:
                continue
            pts = cluster_pts[valid]
            u = np.clip((fx * pts[:, 0] / pts[:, 2] + cx).astype(int), 0, frame.shape[1] - 1)
            v = np.clip((fy * pts[:, 1] / pts[:, 2] + cy).astype(int), 0, frame.shape[0] - 1)
            u1, u2 = int(u.min()), int(u.max())
            v1, v2 = int(v.min()), int(v.max())

            color = color_bgr.get(pushable, (50, 210, 230))
            cv2.rectangle(frame, (u1, v1), (u2, v2), color, 2)
            cv2.putText(frame, f"{label}  {dist:.1f}m  {volume*1000:.1f}L",
                        (u1, max(v1 - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # 박스 다 그린 뒤 카메라 장착 각도에 맞게 회전 (people_tracker와 동일)
        frame = self._rotate_img(frame)

        # 상단 상태 표시 (회전 후 좌표 기준)
        obj_count = len(draw_list)
        status = f"Objects: {obj_count}  ROI: {ROI_FRONT_MIN}-{ROI_FRONT_MAX}m  rot:{self._rotate_deg}"
        cv2.putText(frame, status, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

        return frame

    # ── 유틸 ──────────────────────────────────────────────────────────────────

    def _depth_to_pcd(self, depth: np.ndarray, fx, fy, cx, cy) -> o3d.geometry.PointCloud:
        h, w = depth.shape
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))
        Z = depth.astype(float) / 1000.0
        valid = (Z > 0.1) & (Z < 5.0)
        Z = Z[valid]
        u = u_grid[valid]; v = v_grid[valid]
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        pts = np.stack([X, Y, Z], axis=1)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        return pcd

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

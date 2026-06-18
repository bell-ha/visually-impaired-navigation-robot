from __future__ import annotations
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import argparse
import threading

import cv2
import numpy as np

from tracker import PersonTracker
from direction import DirectionEstimator
from visualization import Visualizer
from utils import bottom_center_of_box, draw_text_with_bg

# ROS2 optional — only needed when source == "ros"
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Image as ROSImage
    from ros_publisher import PeopleMarkerPublisher
    _ROS_OK = True
except ImportError:
    _ROS_OK = False

WINDOW = "Human Direction MVP"
SEEKBAR = "position"

ROS_TOPIC = "/camera/camera/color/image_raw"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Human Direction Estimation MVP")
    p.add_argument("source", nargs="?", default="ros",
                   help="'ros'=머리 카메라(기본), 영상 파일, 또는 웹캠 인덱스(예: 6=그리퍼)")
    p.add_argument("--model", default="yolov8n.pt")
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--iou", type=float, default=0.5)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--save", type=str, default="")
    p.add_argument("--device", default="cpu")
    p.add_argument("--rotate", type=int, default=90, choices=[0, 90, 180, 270],
                   help="프레임 회전: 90, 180, 270 (기본값: 90)")
    return p.parse_args()


# ── ROS2 camera source ────────────────────────────────────────────────────────

class _RosFrameNode(Node):
    def __init__(self):
        super().__init__("people_tracker_capture")
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self.create_subscription(ROSImage, ROS_TOPIC, self._cb, 10)
        print(f"[ROS2] 구독 시작: {ROS_TOPIC}")

    def _cb(self, msg: ROSImage):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, -1))
            if msg.encoding == "rgb8":
                arr = arr[:, :, ::-1]  # RGB → BGR for cv2
            with self._lock:
                self._frame = arr.copy()
        except Exception as e:
            self.get_logger().warn(f"프레임 오류: {e}")

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None


class RosCapture:
    """cv2.VideoCapture처럼 read() 인터페이스를 제공하는 ROS2 래퍼."""

    def __init__(self):
        if not _ROS_OK:
            print("[오류] rclpy가 없습니다. source=ros 사용 불가.", file=sys.stderr)
            sys.exit(1)
        rclpy.init()
        self._node      = _RosFrameNode()
        self.publisher  = PeopleMarkerPublisher()

        # 두 노드를 MultiThreadedExecutor로 함께 spin
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor.add_node(self.publisher)
        self._thread = threading.Thread(
            target=self._executor.spin, daemon=True)
        self._thread.start()

        print("[ROS2] 첫 프레임 대기 중...", end="", flush=True)
        import time
        for _ in range(100):
            if self._node.get_frame() is not None:
                print(" 연결됨!")
                break
            time.sleep(0.1)
        else:
            print(" 타임아웃 — ROS2 카메라 드라이버가 실행 중인지 확인하세요.")

    def read(self):
        frame = self._node.get_frame()
        return (frame is not None), frame

    def get(self, prop):
        frame = self._node.get_frame()
        if frame is None:
            return 0
        h, w = frame.shape[:2]
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return w
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return h
        if prop == cv2.CAP_PROP_FPS:
            return 30
        if prop == cv2.CAP_PROP_FRAME_COUNT:
            return 0
        return 0

    def set(self, prop, val):
        pass

    def isOpened(self):
        return True

    def release(self):
        self._executor.shutdown()
        rclpy.shutdown()


def open_source(source_str: str):
    if source_str == "ros":
        return RosCapture()
    src = int(source_str) if source_str.isdigit() else source_str
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"[ERROR] 소스를 열 수 없습니다: {source_str}", file=sys.stderr)
        sys.exit(1)
    return cap


def fmt_time(seconds: float) -> str:
    s = max(0, int(seconds))
    return f"{s // 60:02d}:{s % 60:02d}"


def draw_seekbar(frame: np.ndarray, cur: int, total: int, fps: float) -> np.ndarray:
    h, w = frame.shape[:2]
    BAR_H = 28
    out = frame.copy()
    overlay = out.copy()
    cv2.rectangle(overlay, (0, h - BAR_H), (w, h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.7, out, 0.3, 0, out)
    ratio = cur / max(total - 1, 1)
    filled = int(w * ratio)
    cv2.rectangle(out, (0, h - BAR_H + 6), (filled, h - 8), (0, 200, 255), -1)
    cv2.rectangle(out, (0, h - BAR_H + 6), (w, h - 8), (80, 80, 80), 1)
    cx = max(8, min(filled, w - 8))
    cy = h - BAR_H // 2
    cv2.circle(out, (cx, cy), 7, (0, 220, 255), -1)
    cur_t = fmt_time(cur / fps) if fps > 0 else "--:--"
    tot_t = fmt_time(total / fps) if fps > 0 else "--:--"
    draw_text_with_bg(out, f"{cur_t} / {tot_t}", (w - 130, h - 8),
                      font_scale=0.48, fg=(220, 220, 220), bg=(20, 20, 20))
    return out


def main() -> None:
    args = parse_args()
    # ros / webcam index → live stream, file path → seekable video
    is_video = not str(args.source).isdigit() and args.source != "ros"

    cap = open_source(args.source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if is_video else 0
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    if is_video and total_frames > 1:
        cv2.createTrackbar(SEEKBAR, WINDOW, 0, total_frames - 1, lambda x: None)

    tracker = PersonTracker(model_path=args.model, conf=args.conf,
                            iou=args.iou, device=args.device)
    estimator = DirectionEstimator()
    visualizer = Visualizer()

    writer = None
    if args.save:
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.save, fourcc, src_fps, (w, h))
        print(f"[INFO] 결과를 저장합니다: {args.save}")

    print("[INFO] 시작합니다.")
    print("       Space: 일시정지/재생   q/ESC: 종료   r: 시계 방향 90도 회전" +
          ("   시크바로 구간 이동 가능" if is_video else ""))

    paused = False
    last_bar_pos = 0
    last_rendered = None
    rotate_step = args.rotate  # 현재 회전 누적값 (0/90/180/270)

    while True:
        wait_ms = 30 if paused else 1
        key = cv2.waitKey(wait_ms) & 0xFF

        if key in (ord("q"), 27):
            break
        if key == ord(" "):
            paused = not paused
            print("[INFO]", "일시정지" if paused else "재생")
        if key == ord("r"):
            rotate_step = (rotate_step + 90) % 360
            print(f"[INFO] 회전: {rotate_step}도")

        if is_video and total_frames > 1:
            bar_pos = cv2.getTrackbarPos(SEEKBAR, WINDOW)
            cur_frame = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
            if abs(bar_pos - last_bar_pos) > 1 and abs(bar_pos - cur_frame) > 2:
                cap.set(cv2.CAP_PROP_POS_FRAMES, bar_pos)
                estimator.cleanup(set())
            last_bar_pos = bar_pos

        if paused:
            if last_rendered is not None:
                cv2.imshow(WINDOW, last_rendered)
            continue

        ret, frame = cap.read()
        if not ret:
            if is_video:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            break

        cur_frame = int(cap.get(cv2.CAP_PROP_POS_FRAMES))

        if is_video and total_frames > 1:
            cv2.setTrackbarPos(SEEKBAR, WINDOW, min(cur_frame, total_frames - 1))
            last_bar_pos = min(cur_frame, total_frames - 1)

        # YOLO는 회전된 프레임으로 (서있는 자세), bbox는 원본 좌표로 역변환
        tracks = tracker.update(frame, rotate_step)

        active_ids: set[int] = set()
        direction_results: dict = {}
        for x1, y1, x2, y2, tid in tracks:
            cx, cy = bottom_center_of_box(x1, y1, x2, y2)
            direction_results[tid] = estimator.update(tid, cx, cy)
            active_ids.add(tid)

        estimator.cleanup(active_ids)

        # RViz2 마커 퍼블리시 — 원본 좌표 기준 (depth와 동일 좌표계)
        if hasattr(cap, "publisher"):
            cap.publisher.publish(tracks, active_ids)

        # 원본 좌표로 그린 뒤 마지막에 회전 → bbox/화살표 위치도 정확
        rendered = visualizer.draw(frame, tracks, direction_results, None)
        if rotate_step == 90:
            rendered = cv2.rotate(rendered, cv2.ROTATE_90_CLOCKWISE)
        elif rotate_step == 270:
            rendered = cv2.rotate(rendered, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif rotate_step == 180:
            rendered = cv2.rotate(rendered, cv2.ROTATE_180)

        if is_video and total_frames > 1:
            rendered = draw_seekbar(rendered, cur_frame, total_frames, src_fps)

        last_rendered = rendered
        cv2.imshow(WINDOW, rendered)

        if writer is not None:
            writer.write(rendered)

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    print("[INFO] 종료되었습니다.")


if __name__ == "__main__":
    main()

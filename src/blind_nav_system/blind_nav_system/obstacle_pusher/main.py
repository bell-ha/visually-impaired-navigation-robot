#!/usr/bin/env python3
"""
obstacle_pusher 진입점

실행:
  source /opt/ros/humble/setup.bash
  source ~/GitHub/visually-impaired-navigation-robot/src/blind_nav_system/venv/bin/activate
  cd .../blind_nav_system/obstacle_pusher
  python3 main.py
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import time
import threading 
import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from obstacle_detector import ObstacleDetector
from push_probe import PushProbe

WINDOW = "obstacle_pusher"


def main():
    rclpy.init()

    detector = ObstacleDetector()
    probe    = PushProbe()

    # ROS2 executor는 백그라운드 daemon 스레드 — people_tracker와 동일 패턴
    executor = MultiThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(probe)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("[obstacle_pusher] 시작")
    print("  /obstacle_pusher/objects  — 박스 후보 (RViz: 큐브)")
    print("  /obstacle_pusher/decision — push / detour 결정")
    print("  /obstacle_pusher/markers  — RViz 시각화")
    print("  r: 회전 전환 (90→180→270→0)   q/ESC: 종료")

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    # 메인 스레드에서 처리 + imshow — OpenCV GUI는 반드시 메인 스레드
    rotate = 90   # 기본값: 카메라 90° 장착
    PROCESS_HZ = 3          # RANSAC+DBSCAN+YOLO를 초당 3회만 실행
    interval   = 1.0 / PROCESS_HZ
    last_proc  = 0.0
    last_frame = None

    # 카메라 대기 화면
    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(placeholder, "Waiting for camera...",
                (80, 230), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 120, 120), 2)
    cv2.putText(placeholder, "Check: ros2 topic hz /camera/camera/color/image_raw",
                (20, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1)

    try:
        while rclpy.ok():
            now = time.monotonic()
            if now - last_proc >= interval:
                last_proc = now
                try:
                    frame = detector.process_frame()
                    if frame is not None:
                        last_frame = frame
                except Exception as e:
                    print(f"[ERROR] process_frame: {e}")

            cv2.imshow(WINDOW, last_frame if last_frame is not None else placeholder)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                rotate = (rotate + 90) % 360
                detector._rotate_deg = rotate
                print(f"[INFO] 회전: {rotate}도")
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        executor.shutdown()
        rclpy.shutdown()
        print("[obstacle_pusher] 종료")


if __name__ == "__main__":
    main()

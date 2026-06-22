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

import threading
import cv2
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
    try:
        while rclpy.ok():
            frame = detector.process_frame()   # RANSAC + DBSCAN + YOLO + 렌더링
            if frame is not None:
                cv2.imshow(WINDOW, frame)

            key = cv2.waitKey(1) & 0xFF
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

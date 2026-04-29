"""
Visual Servo Node — 목표 버튼을 카메라 중앙에 오도록 팔을 조정.

실행:
  ros2 run blind_nav_system visual_servo_node --ros-args -p target_button:=3

제어 방식:
  error_x (좌우) → wrist_yaw 증감
  error_y (상하) → joint_lift 증감
  오차 < dead_zone 이면 정지
"""

import os
import json
import subprocess
import tempfile

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

VENV_PYTHON  = os.path.expanduser("~/venv_ocr/bin/python3")
INFER_SCRIPT = os.path.join(os.path.dirname(__file__), "ocr_rcnn_infer.py")

IMAGE_W, IMAGE_H = 640, 480
CENTER_X, CENTER_Y = IMAGE_W // 2, IMAGE_H // 2
DEAD_ZONE = 40   # pixels — 이 안에 들어오면 정지


class VisualServoNode(Node):
    def __init__(self):
        super().__init__("visual_servo_node")

        # ── 파라미터 ──────────────────────────────────────────────
        self.target_text = self.declare_parameter("target_button", "3").value
        self.camera_topic = self.declare_parameter(
            "camera_topic", "/gripper_camera/color/image_raw"
        ).value
        # 비례 게인 (튜닝 필요시 파라미터로 조정)
        self.kp_yaw  = self.declare_parameter("kp_yaw",  0.0008).value  # rad/pixel
        self.kp_lift = self.declare_parameter("kp_lift", 0.0003).value  # m/pixel

        # ── 상태 ─────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.current_lift = None
        self.current_yaw  = None
        self._processing  = False
        self._goal_done   = True
        self._centered    = False
        self._last_frame  = None   # 시각화용 최신 프레임

        # ── ROS 인터페이스 ────────────────────────────────────────
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )
        self.img_sub = self.create_subscription(
            Image, self.camera_topic, self._on_image, 10
        )
        self.status_pub = self.create_publisher(String, "/elevator/servo_status", 10)
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory"
        )

        self.get_logger().info(
            f"Visual servo started | target='{self.target_text}' | cam={self.camera_topic}"
        )

    # ── Joint state 수신 ─────────────────────────────────────────
    def _on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name == "joint_lift":
                self.current_lift = pos
            elif name == "joint_wrist_yaw":
                self.current_yaw = pos

    # ── 이미지 수신 → 추론 → 제어 ────────────────────────────────
    def _on_image(self, msg: Image):
        if self._processing or not self._goal_done:
            return
        if self.current_lift is None or self.current_yaw is None:
            self.get_logger().warn("Joint states not received yet")
            return
        if self._centered:
            return

        self._processing = True
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # 시각화용 원본 프레임 저장 (640x480)
            self._last_frame = cv2.resize(cv_img, (IMAGE_W, IMAGE_H))

            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as f:
                tmp = f.name
            cv2.imwrite(tmp, cv_img)

            result = subprocess.run(
                [VENV_PYTHON, INFER_SCRIPT, "--image", tmp],
                capture_output=True, text=True, timeout=15,
            )
            os.unlink(tmp)

            if result.returncode != 0:
                self.get_logger().error(f"Infer error: {result.stderr[:100]}")
                return

            data = json.loads(result.stdout.strip().splitlines()[-1])
            detections = data.get("detections", [])
            self._draw_debug(detections)
            self._servo_step(detections)

        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
        finally:
            self._processing = False

    # ── 디버그 화면 ───────────────────────────────────────────────
    def _draw_debug(self, detections):
        if self._last_frame is None:
            return
        vis = self._last_frame.copy()

        # 중앙 십자선
        cv2.line(vis, (CENTER_X - 30, CENTER_Y), (CENTER_X + 30, CENTER_Y), (255, 255, 0), 1)
        cv2.line(vis, (CENTER_X, CENTER_Y - 30), (CENTER_X, CENTER_Y + 30), (255, 255, 0), 1)
        cv2.circle(vis, (CENTER_X, CENTER_Y), DEAD_ZONE, (255, 255, 0), 1)

        for d in detections:
            box = d["box"]
            x1, y1, x2, y2 = int(box["x1"]), int(box["y1"]), int(box["x2"]), int(box["y2"])
            btn_cx, btn_cy = (x1 + x2) // 2, (y1 + y2) // 2
            is_target = (d["text"] == self.target_text)

            if is_target:
                # 목표 버튼: 빨간 박스 + 두꺼운 테두리
                color = (0, 0, 255)
                thickness = 3
                # 중앙까지 화살표
                cv2.arrowedLine(vis, (CENTER_X, CENTER_Y), (btn_cx, btn_cy),
                                (0, 0, 255), 2, tipLength=0.2)
                # 오차 텍스트
                ex = btn_cx - CENTER_X
                ey = btn_cy - CENTER_Y
                cv2.putText(vis, f"err({ex:+d},{ey:+d})",
                            (10, IMAGE_H - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 255), 2)
            else:
                color = (0, 200, 80)
                thickness = 2

            cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness)
            label = f"{d['text']} {d['score']:.2f}"
            cv2.rectangle(vis, (x1, y1 - 18), (x1 + len(label) * 9, y1), color, -1)
            cv2.putText(vis, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # 상태 배너
        state = "CENTERED" if self._centered else f"TRACKING: '{self.target_text}'"
        banner_color = (0, 180, 0) if self._centered else (30, 30, 30)
        cv2.rectangle(vis, (0, 0), (IMAGE_W, 26), banner_color, -1)
        cv2.putText(vis, state, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("Visual Servo", vis)
        cv2.waitKey(1)

    # ── 서보 제어 한 스텝 ─────────────────────────────────────────
    def _servo_step(self, detections):
        # 목표 버튼 찾기
        target = next(
            (d for d in detections if d["text"] == self.target_text), None
        )

        if target is None:
            self.get_logger().warn(
                f"Button '{self.target_text}' not found "
                f"(detected: {[d['text'] for d in detections]})"
            )
            self._publish_status("NOT_FOUND")
            return

        box = target["box"]
        btn_x = (box["x1"] + box["x2"]) / 2.0
        btn_y = (box["y1"] + box["y2"]) / 2.0

        error_x = btn_x - CENTER_X   # 양수: 버튼이 오른쪽
        error_y = btn_y - CENTER_Y   # 양수: 버튼이 아래

        self.get_logger().info(
            f"Button '{self.target_text}' | center=({btn_x:.0f},{btn_y:.0f}) "
            f"| error=({error_x:+.0f},{error_y:+.0f})"
        )

        if abs(error_x) < DEAD_ZONE and abs(error_y) < DEAD_ZONE:
            self.get_logger().info("CENTERED! Stopping servo.")
            self._centered = True
            self._publish_status("CENTERED")
            return

        # 비례 제어: 오차 → 관절 증분
        delta_yaw  = -self.kp_yaw  * error_x   # 오른쪽 오차 → yaw 감소
        delta_lift = -self.kp_lift * error_y    # 아래 오차   → lift 감소

        new_yaw  = float(self.current_yaw)  + delta_yaw
        new_lift = float(self.current_lift) + delta_lift

        # 관절 범위 클램프
        new_yaw  = max(-2.88, min(1.67, new_yaw))
        new_lift = max(0.15,  min(1.10, new_lift))

        self._send_joint_goal(new_lift, new_yaw)
        self._publish_status(f"TRACKING error=({error_x:+.0f},{error_y:+.0f})")

    # ── Joint trajectory 전송 ─────────────────────────────────────
    def _send_joint_goal(self, lift: float, yaw: float):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server not available")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_lift", "joint_wrist_yaw"]

        pt = JointTrajectoryPoint()
        pt.positions = [lift, yaw]
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]

        self._goal_done = False
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

        self.get_logger().info(f"→ lift={lift:.3f}  yaw={yaw:.3f}")

    def _on_goal_response(self, future):
        handle = future.result()
        if handle.accepted:
            handle.get_result_async().add_done_callback(self._on_goal_done)
        else:
            self._goal_done = True

    def _on_goal_done(self, future):
        self._goal_done = True

    def _publish_status(self, msg: str):
        m = String()
        m.data = msg
        self.status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

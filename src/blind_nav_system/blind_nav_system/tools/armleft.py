#!/usr/bin/env python3
import time
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class PoseHold(Node):
    """
    Stretch SE3 고정형:
      - /stretch_controller/follow_joint_trajectory 로
      - joint_lift + joint_arm_l0~l3 + joint_wrist_yaw 를 목표 위치로 보내고
      - 주기적으로 재전송해서(hold) 리프트가 스르륵 내려가는 걸 방지
      - wrist_yaw = -1.57 rad → 매니퓰레이터가 로봇 몸통 안쪽(정면)을 향함
    """

    def __init__(self):
        super().__init__("pose_hold_fixed")

        # ====== 원하는 목표값 ======
        self.lift_target      = 0.90   # m  (너무 높으면 0.7~0.8부터)
        self.arm_target       = 0.00   # m  (완전 수축)
        self.wrist_yaw_target = +3.4  # rad (-π/2 ≈ -90°, 몸통 안쪽 방향)
        #   ※ 방향이 반대이면 +1.57(+90°)로 바꿔보세요.
        #   ※ SE3 wrist_yaw 가동 범위: 약 [-2.88, 1.67] rad
        self.gripper_target   = 0.00   # m — 그리퍼 닫힘. 손목을 안쪽으로 돌릴 때
        #   그리퍼가 열려 있으면 손가락이 몸에 닿아 손목이 못 돌고, armleft가 2초마다
        #   계속 밀어붙여 wrist_yaw 서보가 과부하(overload) 남 → 닫아서 방지.

        # ====== 홀드 재전송 주기 ======
        self.resend_period = 2.0  # 초 (1.0~3.0 사이 권장)

        # ====== 고정된 액션/조인트 ======
        self.joint_state_topic = "/joint_states"
        self.action_name = "/stretch_controller/follow_joint_trajectory"
        self.joints = [
            "joint_lift",
            "joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3",
            "joint_wrist_yaw",     # 안쪽 방향 고정
            "gripper_aperture",    # ← 그리퍼 닫힘 유지 (손목 회전 시 충돌·과부하 방지)
        ]
        # joint_states 준비 확인용 — gripper_aperture 는 /joint_states 에 발행되지
        # 않는 명령전용 가상관절이므로 여기서 제외 (넣으면 준비확인이 영영 안 됨)
        self.state_joints = [
            "joint_lift", "joint_arm_l0", "joint_arm_l1", "joint_arm_l2",
            "joint_arm_l3", "joint_wrist_yaw",
        ]

        # 내부 상태
        self.got_relevant_joint_state = False
        self.last_send_time = 0.0

        # 구독 (조인트 상태가 실제로 들어오는지 확인용)
        self.create_subscription(JointState, self.joint_state_topic, self._on_joint_states, 10)

        # 액션 클라이언트
        self.client = ActionClient(self, FollowJointTrajectory, self.action_name)

        self.get_logger().info(f"Waiting for action server: {self.action_name}")
        self.create_timer(0.2, self._tick)

    def _on_joint_states(self, msg: JointState):
        names = set(msg.name)
        needed = set(self.state_joints)
        if needed.issubset(names):
            self.got_relevant_joint_state = True

    def _build_goal(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints

        p = JointTrajectoryPoint()
        # joint_lift,  arm_l0~l3 (각 0.0),  wrist_yaw
        positions = (
            [float(self.lift_target)]
            + [float(self.arm_target)] * 4   # arm_l0 ~ arm_l3
            + [float(self.wrist_yaw_target)]
            + [float(self.gripper_target)]   # gripper_aperture (닫힘)
        )
        p.positions = positions
        p.time_from_start.sec = 2  # 2초에 걸쳐 부드럽게 이동

        goal.trajectory.points = [p]
        return goal, positions

    def _tick(self):
        # 1) 액션 서버 대기
        if not self.client.wait_for_server(timeout_sec=0.0):
            return

        # 2) joint_states 수신 확인
        if not self.got_relevant_joint_state:
            self.get_logger().info(
                f"Waiting for {self.joint_state_topic} to include required joints ..."
            )
            return

        # 3) 일정 주기로 같은 목표 재전송 (hold)
        now = time.time()
        if now - self.last_send_time < self.resend_period:
            return

        goal, positions = self._build_goal()
        self.get_logger().info(
            f"Hold pose -> lift={self.lift_target:.3f}, arm={self.arm_target:.3f}, "
            f"wrist_yaw={self.wrist_yaw_target:.3f} rad | pos={positions}"
        )
        self.client.send_goal_async(goal)
        self.last_send_time = now


def main():
    rclpy.init()
    node = PoseHold()
    # SIGTERM(대시보드 종료/pkill)으로 죽어도 아래 finally 정리가 돌게 함 → 서보 깨끗이
    def _on_term(signum, frame):
        raise KeyboardInterrupt()
    try:
        signal.signal(signal.SIGTERM, _on_term)
    except Exception:
        pass
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
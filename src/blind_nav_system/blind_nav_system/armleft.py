#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class PoseHold(Node):
    """
    Stretch SE3 (현재 네 환경) 고정형:
      - /stretch_controller/follow_joint_trajectory 로
      - joint_lift + joint_arm_l0~l3 를 목표 위치로 보내고
      - 주기적으로 재전송해서(hold) 리프트가 스르륵 내려가는 걸 방지
    """

    def __init__(self):
        super().__init__("pose_hold_fixed")

        # ====== 원하는 목표값 ======
        self.lift_target = 0.90   # m (너무 높으면 0.7~0.8부터)
        self.arm_target = 0.00    # m (완전 수축)

        # ====== 홀드 재전송 주기 ======
        self.resend_period = 2.0  # 초 (1.0~3.0 사이 권장)

        # ====== 고정된 액션/조인트 ======
        self.joint_state_topic = "/joint_states"
        self.action_name = "/stretch_controller/follow_joint_trajectory"
        self.joints = [
            "joint_lift",
            "joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3",
        ]

        # 내부 상태
        self.got_relevant_joint_state = False
        self.last_send_time = 0.0

        # 구독(조인트 상태가 실제로 들어오는지 확인용)
        self.create_subscription(JointState, self.joint_state_topic, self._on_joint_states, 10)

        # 액션 클라이언트
        self.client = ActionClient(self, FollowJointTrajectory, self.action_name)

        self.get_logger().info(f"Waiting for action server: {self.action_name}")
        self.create_timer(0.2, self._tick)

    def _on_joint_states(self, msg: JointState):
        # joint_states 안에 우리가 필요한 조인트가 실제로 포함되는지 확인
        names = set(msg.name)
        needed = set(self.joints)
        if needed.issubset(names):
            self.got_relevant_joint_state = True

    def _build_goal(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints

        p = JointTrajectoryPoint()
        positions = [float(self.lift_target)] + [float(self.arm_target)] * (len(self.joints) - 1)
        p.positions = positions
        p.time_from_start.sec = 2  # 2초에 걸쳐 부드럽게 이동

        goal.trajectory.points = [p]
        return goal, positions

    def _tick(self):
        # 1) 액션 서버 대기
        if not self.client.wait_for_server(timeout_sec=0.0):
            return

        # 2) joint_states가 제대로 들어오는지 확인(초기화 타이밍 대비)
        if not self.got_relevant_joint_state:
            self.get_logger().info(f"Waiting for {self.joint_state_topic} to include {self.joints} ...")
            return

        # 3) 일정 주기로 같은 목표 재전송(hold)
        now = time.time()
        if now - self.last_send_time < self.resend_period:
            return

        goal, positions = self._build_goal()
        self.get_logger().info(f"Hold pose -> lift={self.lift_target:.3f}, arm={self.arm_target:.3f} | pos={positions}")
        self.client.send_goal_async(goal)
        self.last_send_time = now


def main():
    rclpy.init()
    node = PoseHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
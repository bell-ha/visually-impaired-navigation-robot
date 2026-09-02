#!/usr/bin/env python3
"""엘리베이터 캐빈 내부 기록 — 원본만 남긴다 (렌더는 cabin_render.py).

무엇을 하나
  all.pgm의 좌표계 그대로 캐빈 내부를 추가로 찍기 위한 '원본 기록'이다.
  slam_toolbox 재개는 불가능하다(.posegraph가 없다). 대신 AMCL이 주는 map 좌표를
  앵커로 삼아 /scan을 그대로 파일에 남기고, 격자를 칠하는 일은 전부 오프라인으로
  미룬다.

왜 기록과 렌더를 나눴나
  라이브로 칠하면 필터 값(섹터 각도·박스 크기·로그오즈 임계)을 엘리베이터 앞에
  서서 고쳐야 한다. 사용자가 현장에 서 있는 시간이 이 작업에서 제일 비싼 자원이다.
  캡처는 원본만 남기고, 렌더는 나중에 몇 번이든 다시 돌린다.

안전
  이 스크립트는 cmd_vel을 발행하지 않는다. 팔·리프트도 명령하지 않는다.
  구독만 한다. 이동은 100% 사람(게임패드)이 한다 — 기록 전용이라는 것이
  이 스크립트의 유일한 안전 근거다. /initialpose도 발행하지 않는다.
  대시보드(8080)·엘베앱(5000)에 의존하지 않는다.

의존
  /scan, /tf(+/tf_static), /stretch/joint_states 셋뿐.

═══ 사용 절차 ═══
  1) 런치는 켜둔 채로 둔다(끄지 마라). AMCL이 살아 있어야 앵커를 얻는다.
  2) 게임패드:
         ros2 run joy joy_node
         ros2 run teleop_twist_joy teleop_node \
             --ros-args --params-file <xbox.config.yaml> \
             -r /cmd_vel:=/stretch/cmd_vel
     ⛔ stretch_gamepad_teleop.py 는 절대 쓰지 마라 — 바디 락을 물어 런치를 죽인다.
  3) 로봇을 엘베 문 앞(맵에 잘 찍힌 자리)에 두고 이 스크립트를 시작한다.
     시작 순간의 map→odom을 얼리므로, 이때 AMCL이 제대로 물려 있어야 한다.
  4) 캐빈 안으로 들어가 제자리에서 360° 1~2바퀴. 천천히 — 화면이 "너무 빠릅니다"를
     띄우면 그 스캔은 버려진다(다음 바퀴에 다시 들어오니 손실은 없다).
  5) 운영자는 캐빈 밖 '문 옆'에 서서 움직이지 않는다(문 정면 금지).
  6) Ctrl+C로 끝낸다. 그다음 cabin_render.py를 돌린다.

앵커를 얼리는 이유
  정확도가 아니라 내부 일관성 때문이다. 캐빈은 맵에서 unknown이라 AMCL이 회전 중
  입자가 퍼지거나 열린 문 너머 홀 기하에 잘못 물려 포즈가 점프한다. 회전 중 한 번의
  점프가 맵 전체를 전단(shear)시킨다. 실시간 AMCL 포즈도 같이 기록하므로,
  "AMCL이 실제로 얼마나 튀었나"는 오프라인에서 공짜로 비교된다.

라이다 각도 보정에 대하여
  엘베앱의 LASER_YAW_OFFSET(π)을 여기로 베끼지 마라. 그건 TF를 안 쓰고 손으로
  맞춘 해킹이다. 이 스크립트는 TF의 laser 프레임을 쓰므로 장착 회전이 이미 들어
  있다. 베끼면 정확히 180° 뒤집힌 맵이 나온다. 대신 base_link 포즈도 같이
  기록해서, '로봇 정면' 기준 섹터 필터를 렌더가 오프셋 상수 없이 계산할 수 있게 한다.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from sensor_msgs.msg import LaserScan, JointState

# /stretch/joint_states 실측으로 확인한 이름이다(추측 아님).
# name: [wrist_extension, joint_lift, joint_arm_l3..l0, joint_head_*, joint_wrist_*, ...]
ARM_EXT_JOINT = "wrist_extension"
LIFT_JOINT    = "joint_lift"
ARM_STOW_MAX  = 0.05     # m — 이보다 뻗어 있으면 '수납되지 않음'으로 본다

# 번짐 e = r·ω·t_scan. t_scan 실측 0.122s, 캐빈 벽 ~0.8m,
# 격자 절반(2.5cm)을 목표로 하면 ω ≤ 0.25 rad/s가 권장, 0.30이 상한이다.
# (nav2 max_vel_theta=0.5는 정확히 5cm — 격자 한 칸이라 마진이 없다.)
OMEGA_MAX = 0.30


def yaw_of(q):
    """쿼터니언 → yaw(rad). 평면 주행이라 z축 회전만 쓴다."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """map←odom(a) ∘ odom←X(b) = map←X. 둘 다 (x, y, yaw) 평면 변환."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap(ayaw + byaw))


def xf_to_xyyaw(tf_msg):
    t = tf_msg.transform.translation
    return (t.x, t.y, yaw_of(tf_msg.transform.rotation))


class CabinCapture(Node):
    def __init__(self, args):
        super().__init__("cabin_capture")
        self.args = args
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.anchor = None          # 얼린 map→odom (x, y, yaw)
        self.joints = None          # (arm_ext, lift, stamp)
        self.started = False
        self.fh = None
        self.n_kept = 0
        self.n_fast = 0
        self.n_notf = 0
        self.n_arm_warn = 0
        self.prev = None            # (t, yaw) — ω 계산용
        self.last_print = 0.0
        self.last_omega = 0.0

        self.create_subscription(JointState, args.joints_topic,
                                 self._on_joints, qos_profile_sensor_data)
        self.create_subscription(LaserScan, args.scan_topic,
                                 self._on_scan, qos_profile_sensor_data)

    # ── 구독 (읽기 전용) ──────────────────────────────────────────────────
    def _on_joints(self, msg):
        d = dict(zip(msg.name, msg.position))
        self.joints = (d.get(ARM_EXT_JOINT), d.get(LIFT_JOINT), time.monotonic())

    def _on_scan(self, scan):
        if not self.started:
            return
        stamp = scan.header.stamp
        try:
            # 반드시 '스캔 시각'으로 조회한다. 최신(Time())으로 조회하면 회전 중
            # 스캔과 포즈가 어긋나 맵이 통째로 밀린다(이 저장소에 최신 조회를 쓰는
            # 곳이 이미 있다 — 같은 실수를 반복하지 않는다).
            t_ol = self.buf.lookup_transform(self.args.odom_frame, scan.header.frame_id,
                                             stamp, timeout=Duration(seconds=0.1))
            t_ob = self.buf.lookup_transform(self.args.odom_frame, self.args.base_frame,
                                             stamp, timeout=Duration(seconds=0.1))
        except Exception:
            self.n_notf += 1        # 보간하지 않는다 — 없으면 버린다
            return

        pose_laser = compose(self.anchor, xf_to_xyyaw(t_ol))
        pose_base  = compose(self.anchor, xf_to_xyyaw(t_ob))

        t = stamp.sec + stamp.nanosec * 1e-9
        omega = 0.0
        if self.prev is not None:
            dt = t - self.prev[0]
            if dt > 1e-3:
                omega = wrap(pose_base[2] - self.prev[1]) / dt
        self.prev = (t, pose_base[2])
        self.last_omega = omega

        if abs(omega) > self.args.omega_max:
            self.n_fast += 1        # 다음 바퀴에 다시 들어온다 — 손실이 아니다
            self._status()
            return

        arm_ext = lift = None
        if self.joints:
            arm_ext, lift, _ = self.joints
            if arm_ext is not None and arm_ext > ARM_STOW_MAX:
                self.n_arm_warn += 1     # 기록 중에는 경고만 — 거부해도 팔은 안 들어가고
                                         # 데이터만 잃는다

        # 실시간 AMCL 포즈(비교용). 없으면 null — 기록을 막지 않는다.
        pose_amcl = None
        try:
            t_ml = self.buf.lookup_transform(self.args.map_frame, scan.header.frame_id,
                                             stamp, timeout=Duration(seconds=0.05))
            pose_amcl = list(xf_to_xyyaw(t_ml))
        except Exception:
            pass

        ranges = [None if not math.isfinite(r) else round(float(r), 3)
                  for r in scan.ranges]
        rec = {"t": t,
               "angle_min": scan.angle_min,
               "angle_increment": scan.angle_increment,
               "range_min": scan.range_min,
               "range_max": scan.range_max,
               "pose_laser": list(pose_laser),   # 앵커 기준 (렌더가 쓰는 것)
               "pose_base":  list(pose_base),    # '로봇 정면' 기준 섹터 필터용
               "pose_amcl":  pose_amcl,          # 실시간 AMCL (튐 비교용)
               "omega": omega,
               "arm_ext": arm_ext,
               "lift": lift,
               "ranges": ranges}
        self.fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self.n_kept += 1
        self._status()

    # ── 화면 ──────────────────────────────────────────────────────────────
    def _status(self):
        now = time.monotonic()
        if now - self.last_print < 0.4:
            return
        self.last_print = now
        warn = ""
        if abs(self.last_omega) > self.args.omega_max:
            warn = "  ⚠ 너무 빠릅니다 — 이 스캔은 버렸습니다"
        elif abs(self.last_omega) > self.args.omega_max * 0.8:
            warn = "  · 조금 느리게"
        sys.stdout.write(
            f"\r기록 {self.n_kept}  버림(빠름) {self.n_fast}  "
            f"TF없음 {self.n_notf}  |ω| {abs(self.last_omega):.2f} rad/s{warn}   ")
        sys.stdout.flush()

    # ── 시작 관문 ────────────────────────────────────────────────────────
    def try_start(self):
        """앵커를 얼리고 팔을 확인한 뒤 기록을 연다. 성공하면 True."""
        try:
            tf_mo = self.buf.lookup_transform(self.args.map_frame, self.args.odom_frame,
                                              rclpy.time.Time(),
                                              timeout=Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f"map→odom 아직 없음 — AMCL 대기 중 ({e})")
            return False
        if self.joints is None:
            self.get_logger().warn(f"{self.args.joints_topic} 아직 없음 — 팔 상태를 "
                                   "확인할 수 없어 시작하지 않는다")
            return False
        arm_ext = self.joints[0]
        if arm_ext is None:
            self.get_logger().error(f"joint_states에 {ARM_EXT_JOINT}가 없다 — 시작 거부")
            return False
        if arm_ext > ARM_STOW_MAX:
            # 이 순간의 거부는 사용자가 팔을 수납하게 만든다 = 결과를 바꾼다.
            self.get_logger().error(
                f"⛔ 팔이 뻗어 있다({ARM_EXT_JOINT}={arm_ext:.3f}m > {ARM_STOW_MAX}m) — "
                "기록을 시작하지 않는다. 팔을 수납한 뒤 다시 실행하라.")
            return False

        self.anchor = xf_to_xyyaw(tf_mo)
        self.fh = open(self.args.out, "w", encoding="utf-8")
        head = {"_header": 1,
                "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                "anchor_map_odom": list(self.anchor),
                "map_frame": self.args.map_frame, "odom_frame": self.args.odom_frame,
                "base_frame": self.args.base_frame,
                "scan_topic": self.args.scan_topic,
                "omega_max": self.args.omega_max,
                "arm_stow_max": ARM_STOW_MAX,
                "note": "pose_* 는 전부 map 좌표계 (x, y, yaw). "
                        "pose_laser/pose_base 는 얼린 앵커 기준, pose_amcl 은 실시간."}
        self.fh.write(json.dumps(head, ensure_ascii=False) + "\n")
        self.started = True
        self.get_logger().info(
            f"앵커 고정 map→odom = ({self.anchor[0]:+.3f}, {self.anchor[1]:+.3f}, "
            f"{math.degrees(self.anchor[2]):+.1f}°) — 기록 시작: {self.args.out}")
        self.get_logger().info("캐빈 안에서 제자리 360° 1~2바퀴. 끝내려면 Ctrl+C.")
        return True

    def close(self):
        if self.fh:
            self.fh.close()
            self.fh = None


def main():
    ap = argparse.ArgumentParser(description="엘베 캐빈 기록(원본만). 로봇을 움직이지 않는다.")
    ap.add_argument("--out", default=time.strftime("cabin_%Y%m%d_%H%M%S.jsonl"),
                    help="출력 JSONL 경로")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--joints-topic", default="/stretch/joint_states")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--omega-max", type=float, default=OMEGA_MAX,
                    help="이보다 빠르게 돌면 그 스캔은 버린다 (rad/s)")
    ap.add_argument("--wait", type=float, default=30.0,
                    help="앵커·팔 상태를 기다리는 최대 시간(초)")
    args = ap.parse_args()

    if os.path.exists(args.out):
        print(f"⛔ 이미 있는 파일이다: {args.out} — 덮어쓰지 않는다.")
        return 1

    rclpy.init()
    node = CabinCapture(args)
    print("TF·joint_states 대기 중...")
    t0 = time.monotonic()
    try:
        while rclpy.ok() and not node.started:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() - t0 > args.wait:
                node.get_logger().error(
                    "시작 조건을 못 갖췄다 — 런치(AMCL)가 떠 있는지, "
                    f"{args.joints_topic}가 나오는지 확인하라. 기록하지 않는다.")
                break
            node.try_start()
        if node.started:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if node.started:
            print(f"기록 종료: {args.out}\n"
                  f"  기록 {node.n_kept}장 · 빠름으로 버림 {node.n_fast} · "
                  f"TF없음으로 버림 {node.n_notf} · 팔 경고 {node.n_arm_warn}회")
            if node.n_kept == 0:
                print("  ⚠ 한 장도 못 남겼다 — 다시 찍어라.")
            elif node.n_arm_warn:
                print("  ⚠ 팔이 뻗은 채로 찍힌 스캔이 있다. 렌더 결과에 팔이 벽으로 보일 수 있다.")
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

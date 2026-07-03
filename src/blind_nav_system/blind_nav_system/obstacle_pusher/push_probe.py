from __future__ import annotations
import json
import math
import time
import threading

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger
import tf2_ros
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatusArray, GoalStatus as GoalStatusMsg
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# ── 토픽 ──────────────────────────────────────────────────────────────────────
OBJECTS_TOPIC           = "/obstacle_pusher/objects"
CMDVEL_TOPIC            = "/stretch/cmd_vel"
ODOM_TOPIC              = "/odom"
RESULT_TOPIC            = "/obstacle_pusher/decision"
MARKER_TOPIC            = "/obstacle_pusher/markers"
SCAN_TOPIC              = "/scan"
PUSH_ENABLE_FLAG        = "/tmp/obstacle_push_enabled"
LOCAL_COSTMAP_CLEAR_SVC  = "/local_costmap/clear_entirely_local_costmap"
GLOBAL_COSTMAP_CLEAR_SVC = "/global_costmap/clear_entirely_global_costmap"
PROBE_COOLDOWN           = 30.0   # s — heavy 판정 후 재탐침 금지 시간
PROBE_ANY_COOLDOWN       =  8.0   # s — probe 완료 후 (light/heavy 무관) 재탐침 금지

# ── Probe Push 파라미터 ────────────────────────────────────────────────────────
CAMERA_CONE_DEG    = 40.0   # ° — 카메라 광축 기준 이 각도 이내 의자만 감지/접근
APPROACH_START_DIST= 3.0    # m — 의자 감지 즉시 직접 접근 시작 (카메라 최대 감지거리)
APPROACH_SPEED     = 0.15   # m/s — 직접 접근 속도
APPROACH_STEER_K   = 0.50   # 조향 비례계수 (cam_angle_rad → angular.z rad/s)
PROBE_TRIGGER_DIST = 0.55   # m   — 접근 후 probe 시작 거리 (카메라 최소거리 0.3m 여유)
PROBE_SPEED        = 0.08   # m/s — 탐침 속도 (느리게, 접촉 감지용)
PROBE_DURATION     = 1.5    # s   — 탐침 밀기 (가볍/무거움 판정용)
PUSH_DURATION      = 5.0    # s   — 가벼움 확정 후 실제로 치우는 밀기 시간
PUSH_SPEED         = 0.12   # m/s — 실제 밀기 속도
PROBE_MOVE_THR     = 0.04   # m   — odom fallback: 이 이상이면 "가벼움"
APPROACH_DIST      = 0.30   # m   — 의자에서 이 거리까지 접근 후 probe
LIDAR_DELTA_THR    = 0.05   # m   — LiDAR 박스 이동량 threshold (조정 대상)
COSTMAP_CLEAR_RATE = 0.40   # s   — costmap clearing 주기
LIDAR_SEARCH_R     = 0.40   # m   — probe 전후 LiDAR 클러스터 탐색 반경


class PushProbe(Node):
    """
    /obstacle_pusher/objects 구독 → pushable 필드에 따라 3경로 분기:

    "pushable"     — YOLO 확정 가벼운 물체 → 바로 밀기 (probe 없이)
    "not_pushable" — YOLO 확정 가구/고정물 → 즉시 우회 결정 퍼블리시
    "unknown"      — YOLO 미인식 (박스 등) → costmap 해제 후 접근 → probe push
                     probe 결과는 로그로 출력 → LIDAR_DELTA_THR 수동 조정
    """

    def __init__(self):
        super().__init__("push_probe")

        self._objects: list[dict] = []
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0
        self._scan: LaserScan | None = None
        self._lock              = threading.Lock()
        self._probing           = False
        self._approaching       = False  # 직접 cmd_vel 접근 중 (Nav2 취소 상태)
        self._clearing          = False
        self._heavy_until       = 0.0   # heavy 판정 후 재탐침 금지 시각
        self._any_probe_until   = 0.0   # probe 완료 후 일반 쿨다운
        self._nav2_active       = False # Nav2 active goal 여부
        self._last_goal_pose: PoseStamped | None = None  # RViz 2D Goal Pose 마지막 목표

        self.create_subscription(String,         OBJECTS_TOPIC,                       self._objects_cb,    10)
        self.create_subscription(Odometry,        ODOM_TOPIC,                          self._odom_cb,       10)
        self.create_subscription(LaserScan,       SCAN_TOPIC,                          self._scan_cb,       10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',  self._nav2_status_cb,  10)
        self.create_subscription(PoseStamped,     '/goal_pose',                        self._goal_pose_cb,    10)

        self._cmd_pub    = self.create_publisher(Twist,       CMDVEL_TOPIC,  10)
        self._result_pub = self.create_publisher(String,      RESULT_TOPIC,  10)
        self._marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC,  10)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._costmap_cli        = self.create_client(Trigger,     LOCAL_COSTMAP_CLEAR_SVC)
        self._global_costmap_cli = self.create_client(Trigger,     GLOBAL_COSTMAP_CLEAR_SVC)
        self._nav2_cancel_cli    = self.create_client(CancelGoal,  '/navigate_to_pose/_action/cancel_goal')
        self._nav2_action_cli    = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(0.2, self._check)
        self.get_logger().info("PushProbe 노드 시작")
        self.get_logger().info(
            f"  LiDAR threshold: {LIDAR_DELTA_THR}m  |  odom fallback threshold: {PROBE_MOVE_THR}m")

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _objects_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._objects = data.get("objects", [])
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        with self._lock:
            self._odom_x   = msg.pose.pose.position.x
            self._odom_y   = msg.pose.pose.position.y
            self._odom_yaw = yaw

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            self._scan = msg

    def _nav2_status_cb(self, msg: GoalStatusArray):
        active = any(s.status == GoalStatusMsg.STATUS_EXECUTING for s in msg.status_list)
        with self._lock:
            self._nav2_active = active

    def _goal_pose_cb(self, msg: PoseStamped):
        with self._lock:
            self._last_goal_pose = msg

    # ── 방향 필터 ─────────────────────────────────────────────────────────────

    def _in_camera_cone(self, obj: dict) -> bool:
        """
        카메라 광축 기준 ±CAMERA_CONE_DEG 이내 의자만 probe.
        odom/map 좌표가 아닌 카메라에서 직접 계산한 각도를 사용 —
        Nav2가 우회 경로를 잡아도 YOLO가 탐지했다면 카메라 FOV 안에 있음.
        cam_angle_deg 미제공 시(구버전 msg) 통과 처리.
        """
        cam_angle = obj.get("cam_angle_deg")
        if cam_angle is None:
            return True   # 하위 호환: 필드 없으면 통과
        return abs(cam_angle) <= CAMERA_CONE_DEG

    # ── 주기 체크 & 분기 ──────────────────────────────────────────────────────

    def _push_enabled(self) -> bool:
        # 1) 대시보드 토글 확인
        try:
            from pathlib import Path
            if Path(PUSH_ENABLE_FLAG).read_text().strip() != "1":
                self.get_logger().warn(
                    f"[PUSH_DISABLED] 대시보드 '장애물 밀기' 꺼짐",
                    throttle_duration_sec=10.0)
                return False
        except FileNotFoundError:
            self.get_logger().warn(
                f"[PUSH_DISABLED] {PUSH_ENABLE_FLAG} 없음 — 대시보드 실행 필요",
                throttle_duration_sec=10.0)
            return False
        except Exception as e:
            self.get_logger().warn(f"[PUSH_DISABLED] 오류: {e}", throttle_duration_sec=10.0)
            return False

        # 2) Nav2 active goal 확인 (RViz 2D Goal Pose 포함)
        with self._lock:
            nav_active = self._nav2_active
        if not nav_active:
            self.get_logger().info(
                "[PUSH_DISABLED] Nav2 goal 없음 — RViz에서 2D Goal Pose 설정 필요",
                throttle_duration_sec=5.0)
            return False

        return True

    def _check(self):
        if self._probing:
            return

        with self._lock:
            approaching = self._approaching

        # 접근 중이 아닐 때만 Nav2 goal 확인 (접근 중엔 Nav2를 이미 취소했음)
        if not approaching:
            if not self._push_enabled():
                if self._clearing:
                    self._clearing = False
                return
        else:
            # 접근 중: 대시보드가 꺼지면 즉시 중단
            try:
                from pathlib import Path
                if Path(PUSH_ENABLE_FLAG).read_text().strip() != "1":
                    self._stop_approach("대시보드 꺼짐")
                    return
            except Exception:
                self._stop_approach("대시보드 확인 실패")
                return

        with self._lock:
            objects = list(self._objects)

        # 카메라 콘 내, APPROACH_START_DIST 이내 의자만 추림
        visible = [
            o for o in objects
            if self._in_camera_cone(o) and o.get("distance", 999) <= APPROACH_START_DIST
        ]

        if not visible:
            if approaching:
                self._stop_approach("의자 시야에서 사라짐")
            elif self._clearing:
                self._clearing = False
            return

        target = min(visible, key=lambda o: o["distance"])
        dist   = target["distance"]

        # heavy 쿨다운 — 무거운 장애물 판정 후 일정 시간 재진입 금지
        if time.time() < self._heavy_until:
            if approaching:
                self._stop_approach("heavy 쿨다운 중")
            return

        # probe 완료 후 일반 쿨다운 (왔다리갔다리 방지)
        if time.time() < self._any_probe_until:
            if approaching:
                self._stop_approach("probe 쿨다운 중")
            return

        if dist <= PROBE_TRIGGER_DIST:
            # probe 거리 도달 → 접근 중단 → probe 시작
            if approaching:
                self._cmd_pub.publish(Twist())  # 즉시 정지
                with self._lock:
                    self._approaching = False
            self.get_logger().info(
                f"의자 {dist:.2f}m (cam={target.get('cam_angle_deg','?')}°) → probe 시작")
            self._probing = True
            threading.Thread(target=self._run_probe, args=(target,), daemon=True).start()
        else:
            # 직접 접근 단계
            if not approaching:
                self._start_approach(target)
            else:
                self._send_approach_cmd(target)

    # ── 직접 접근 (카메라 각도 기반 조향) ────────────────────────────────────────

    def _start_approach(self, target: dict):
        """Nav2 취소 후 의자를 향해 직접 cmd_vel 접근 시작."""
        dist      = target.get("distance", 0.0)
        angle_deg = target.get("cam_angle_deg", 0.0)
        self.get_logger().info(
            f"[APPROACH] 의자 {dist:.1f}m / cam={angle_deg:.1f}° "
            f"→ Nav2 취소, 직접 접근 시작")
        if self._nav2_cancel_cli.service_is_ready():
            self._nav2_cancel_cli.call_async(CancelGoal.Request())
        # costmap 클리어도 병행 — probe 완료 후 Nav2가 바로 경로 잡도록
        if not self._clearing:
            self._clearing = True
            threading.Thread(target=self._clear_loop, daemon=True).start()
        with self._lock:
            self._approaching = True
        self._send_approach_cmd(target)

    def _send_approach_cmd(self, target: dict):
        """의자를 향해 비례 조향 + 전진. _check()에서 0.2s마다 갱신."""
        angle_deg = target.get("cam_angle_deg", 0.0)
        angle_rad = math.radians(angle_deg)
        # 각도가 클수록 전진 속도 줄임 (cos 가중, 최소 40% 유지)
        linear_x  = APPROACH_SPEED * max(0.4, math.cos(angle_rad))
        # 우향 양수(+) → 로봇 우회전(ROS 좌표계: 음수 angular.z)
        angular_z = -angle_rad * APPROACH_STEER_K
        angular_z = max(-0.5, min(0.5, angular_z))  # ±0.5 rad/s 클리핑
        twist = Twist()
        twist.linear.x  = linear_x
        twist.angular.z = angular_z
        self._cmd_pub.publish(twist)

    def _stop_approach(self, reason: str = ""):
        """직접 접근 중단 — 로봇 정지 + costmap clearing 중단 + Nav2 재시작."""
        with self._lock:
            self._approaching = False
        self._clearing = False
        self._cmd_pub.publish(Twist())  # 정지
        if reason:
            self.get_logger().info(f"[APPROACH] 중단: {reason}")
        self._resend_nav2_goal(f"접근중단({reason})")

    # ── costmap clearing ──────────────────────────────────────────────────────

    def _clear_loop(self):
        """local + global costmap을 주기적으로 클리어 — 의자 위치를 Nav2 경로에서 해제."""
        while self._clearing:
            for cli in (self._costmap_cli, self._global_costmap_cli):
                if cli.service_is_ready():
                    cli.call_async(Trigger.Request())
            time.sleep(COSTMAP_CLEAR_RATE)

    # ── LiDAR 클러스터 centroid ───────────────────────────────────────────────

    def _lidar_centroid_near(self, map_x: float, map_y: float) -> tuple[float, float] | None:
        """
        /scan 포인트를 map frame으로 변환 후, (map_x, map_y) 반경 LIDAR_SEARCH_R 내
        클러스터 centroid 반환. LiDAR가 박스를 감지하지 못하면 None.
        """
        with self._lock:
            scan = self._scan
        if scan is None:
            return None

        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges, dtype=float)
        valid  = (
            np.isfinite(ranges) &
            (ranges > scan.range_min) &
            (ranges < scan.range_max)
        )
        if valid.sum() < 5:
            return None

        xs = (ranges * np.cos(angles))[valid]
        ys = (ranges * np.sin(angles))[valid]

        try:
            t = self._tf_buffer.lookup_transform(
                "map", scan.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().debug(f"LiDAR TF 실패: {e}")
            return None

        tx  = t.transform.translation.x
        ty  = t.transform.translation.y
        q   = t.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        c, s = math.cos(yaw), math.sin(yaw)

        xs_m = xs * c - ys * s + tx
        ys_m = xs * s + ys * c + ty

        dists = np.sqrt((xs_m - map_x)**2 + (ys_m - map_y)**2)
        near  = dists < LIDAR_SEARCH_R
        if near.sum() < 5:
            return None

        return float(xs_m[near].mean()), float(ys_m[near].mean())

    # ── Probe Push ────────────────────────────────────────────────────────────

    def _run_probe(self, target: dict):
        # _probing은 _check()에서 이미 True로 설정됨
        # _clear_loop은 _check()에서 이미 시작됨

        try:
            map_x = target.get("map_x",    0.0)
            map_y = target.get("map_y",    0.0)
            dist  = target.get("distance", 999.0)

            # ① Nav2 goal 취소 (RViz 2D Goal Pose 포함) + navigation_client 신호
            if self._nav2_cancel_cli.service_is_ready():
                self._nav2_cancel_cli.call_async(CancelGoal.Request())
                self.get_logger().info("Nav2 goal 취소 → probe 직접 제어 시작")
            self._publish_result("probe_start", "탐침 시작 — Nav2 일시 중단", 0.0, target)
            time.sleep(0.5)

            # ② APPROACH_DIST 앞까지 직접 접근 (빠른 속도)
            approach_m = max(0.0, dist - APPROACH_DIST)
            if approach_m > 0.02:
                self.get_logger().info(
                    f"→ 접근: {approach_m:.2f}m ({APPROACH_SPEED}m/s)")
                with self._lock:
                    x_s, y_s = self._odom_x, self._odom_y
                cmd = Twist(); cmd.linear.x = APPROACH_SPEED
                t_end = time.time() + approach_m / APPROACH_SPEED
                while time.time() < t_end:
                    self._cmd_pub.publish(cmd)
                    time.sleep(0.05)
                self._cmd_pub.publish(Twist())
                time.sleep(0.3)
            else:
                with self._lock:
                    x_s, y_s = self._odom_x, self._odom_y

            # ③ probe 전 LiDAR 위치 기록
            p_before = self._lidar_centroid_near(map_x, map_y)
            if p_before:
                self.get_logger().info(
                    f"[PROBE] P_before (map): ({p_before[0]:.3f}, {p_before[1]:.3f})")
            else:
                self.get_logger().warn("[PROBE] P_before LiDAR 측정 실패")

            with self._lock:
                x0, y0 = self._odom_x, self._odom_y

            # ④ 천천히 전진 (probe push)
            self.get_logger().info(
                f"→ probe 전진 {PROBE_SPEED} m/s × {PROBE_DURATION}s")
            cmd = Twist(); cmd.linear.x = PROBE_SPEED
            t_end = time.time() + PROBE_DURATION
            while time.time() < t_end:
                self._cmd_pub.publish(cmd)
                time.sleep(0.05)
            self._cmd_pub.publish(Twist())
            time.sleep(0.3)

            # ⑤ probe 후 LiDAR 위치 + odom 측정
            p_after = self._lidar_centroid_near(map_x, map_y)
            with self._lock:
                x1, y1 = self._odom_x, self._odom_y
            odom_moved = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)

            # ⑥ 판단 + 수치 출력
            if p_before is not None and p_after is not None:
                lidar_delta = math.sqrt(
                    (p_after[0] - p_before[0])**2 + (p_after[1] - p_before[1])**2)
                self.get_logger().info(
                    f"[PROBE] 박스 이동(LiDAR): {lidar_delta:.3f}m"
                    f"  | 로봇(odom): {odom_moved:.3f}m"
                    f"  | 기준: lidar>{LIDAR_DELTA_THR}m 또는 odom>{PROBE_MOVE_THR}m → 가벼움")
                # LiDAR가 크게 움직였거나, 로봇이 자유롭게 이동(닿지 않았거나 밀림) → 가벼움
                # 로봇도 못 움직이고(odom 작음) LiDAR도 안 움직임 → 무거운 장애물
                if lidar_delta >= LIDAR_DELTA_THR:
                    heavy = False   # 의자가 확실히 이동
                elif odom_moved >= PROBE_MOVE_THR:
                    heavy = False   # 로봇이 자유롭게 이동 → 닿지 않았거나 가볍게 밀림
                else:
                    heavy = True    # 로봇 막힘 + 의자 안 움직임 → 무거운 장애물
                used  = "LiDAR+odom"

            elif p_before is not None and p_after is None:
                # 박스가 LiDAR 시야에서 사라짐 → 크게 밀렸음
                lidar_delta = None
                self.get_logger().info(
                    f"[PROBE] 박스 LiDAR에서 사라짐 → 이동 확정"
                    f"  | 로봇(odom): {odom_moved:.3f}m")
                heavy = False
                used  = "LiDAR(사라짐)"

            else:
                # LiDAR P_before 실패 → odom fallback
                lidar_delta = None
                self.get_logger().warn(
                    f"[PROBE] LiDAR 측정 실패 → odom fallback"
                    f"  | 로봇(odom): {odom_moved:.3f}m"
                    f"  | threshold: PROBE_MOVE_THR={PROBE_MOVE_THR}m")
                heavy = odom_moved < PROBE_MOVE_THR
                used  = "odom(fallback)"

            # ⑦ 판정 로그
            if not heavy:
                action = "push"
                reason = (
                    f"가벼움 [{used}] lidar={lidar_delta:.3f}m odom={odom_moved:.3f}m"
                    if lidar_delta is not None
                    else f"가벼움 [{used}] odom={odom_moved:.3f}m")
                self.get_logger().info(f"판정: 밀고 통과 — {reason}")
            else:
                action = "detour"
                reason = (
                    f"무거움 [{used}] lidar={lidar_delta:.3f}m odom={odom_moved:.3f}m"
                    if lidar_delta is not None
                    else f"무거움 [{used}] odom={odom_moved:.3f}m")
                self.get_logger().info(f"판정: 우회 — {reason}")
                # costmap 클리어 중단 → LiDAR가 다시 채움 → Nav2 우회 경로 계획
                self._clearing = False
                self._heavy_until = time.time() + PROBE_COOLDOWN
                self.get_logger().info(f"costmap 클리어 중단 → Nav2 우회 경로 계획")

            if not heavy:
                # ⑧ 가벼움 → 후퇴 없이 치고 통과
                self.get_logger().info(
                    f"→ 치고 통과: {PUSH_SPEED}m/s × {PUSH_DURATION}s (후퇴 없음)")
                cmd = Twist(); cmd.linear.x = PUSH_SPEED
                t_end = time.time() + PUSH_DURATION
                while time.time() < t_end:
                    self._cmd_pub.publish(cmd)
                    time.sleep(0.05)
                self._cmd_pub.publish(Twist())
                time.sleep(0.3)
                self._publish_result("push_through", reason, odom_moved, target)
                # 치고 통과 완료 → 현재 위치에서 원래 목표로 Nav2 재시작
                self._resend_nav2_goal("치고 통과 완료")

            else:
                # ⑧ 무거움 → 후퇴 없이 Nav2에 우회 경로 위임
                self._publish_result(action, reason, odom_moved, target)
                self._resend_nav2_goal("무거움 판정 → Nav2 우회")

        finally:
            self._probing         = False
            self._any_probe_until = time.time() + PROBE_ANY_COOLDOWN
            # _clearing은 heavy일 때만 위에서 False로 설정 — push면 계속 클리어

    def _resend_nav2_goal(self, reason: str = ""):
        """probe 완료 후 RViz에서 받은 마지막 목표를 Nav2에 재전송."""
        with self._lock:
            pose = self._last_goal_pose
        if pose is None:
            self.get_logger().warn("저장된 목표 없음 — RViz에서 다시 2D Goal Pose 설정 필요")
            return
        if not self._nav2_action_cli.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 action server 미응답")
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = pose.header.frame_id or "map"
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose            = pose.pose
        self._nav2_action_cli.send_goal_async(goal)
        self.get_logger().info(
            f"[{reason}] Nav2 목표 재전송 → "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")

    def _retreat_by(self, dist_m: float):
        if dist_m < 0.01:
            return
        cmd = Twist()
        cmd.linear.x = -PROBE_SPEED
        t_end = time.time() + dist_m / PROBE_SPEED
        while time.time() < t_end:
            self._cmd_pub.publish(cmd)
            time.sleep(0.05)
        self._cmd_pub.publish(Twist())
        time.sleep(0.2)

    # ── 결과 퍼블리시 (공통) ──────────────────────────────────────────────────

    def _publish_result(self, action: str, reason: str, moved_m: float, target: dict):
        result = String()
        result.data = json.dumps({
            "action":   action,
            "reason":   reason,
            "moved_m":  round(moved_m, 3),
            "map_x":    target.get("map_x", 0.0),
            "map_y":    target.get("map_y", 0.0),
            "pushable": target.get("pushable", "unknown"),
        })
        self._result_pub.publish(result)
        self._publish_decision_marker(action, target)

    # ── RViz 마커 ─────────────────────────────────────────────────────────────

    def _publish_decision_marker(self, action: str, target: dict):
        ma  = MarkerArray()
        now = self.get_clock().now().to_msg()

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp    = now
        m.ns, m.id        = "decision", 0
        m.type            = Marker.TEXT_VIEW_FACING
        m.action          = Marker.ADD
        m.pose.position.x = target.get("map_x", 0.0)
        m.pose.position.y = target.get("map_y", 0.0)
        m.pose.position.z = 1.2
        m.pose.orientation.w = 1.0
        m.scale.z         = 0.35

        pushable = target.get("pushable", "unknown")
        if action in ("push", "push_through"):
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.2, 1.0
            m.text = "치고 통과 ✓" if action == "push_through" else "밀고 통과"
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.2, 1.0
            src = "YOLO" if pushable == "not_pushable" else "탐침"
            m.text = f"우회 [{src}]"

        m.lifetime = Duration(sec=5)
        ma.markers.append(m)
        self._marker_pub.publish(ma)

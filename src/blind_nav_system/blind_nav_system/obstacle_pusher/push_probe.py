from __future__ import annotations
import json
import math
import time
import threading

import rclpy
import rclpy.duration
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

# ── 토픽 ──────────────────────────────────────────────────────────────────────
OBJECTS_TOPIC   = "/obstacle_pusher/objects"
CMDVEL_TOPIC    = "/stretch/cmd_vel"
ODOM_TOPIC      = "/odom"
RESULT_TOPIC    = "/obstacle_pusher/decision"
MARKER_TOPIC    = "/obstacle_pusher/markers"
PUSH_ENABLE_FLAG = "/tmp/obstacle_push_enabled"

# ── Probe Push 파라미터 ────────────────────────────────────────────────────────
PROBE_SPEED        = 0.08   # m/s
PROBE_DURATION     = 1.5    # s
PROBE_MOVE_THR     = 0.04   # m — 이 이상 이동했으면 "가벼움"
PROBE_TRIGGER_DIST = 1.2    # m — 이 거리 이내일 때만 probe 시작
DIRECT_PUSH_SPEED  = 0.12   # m/s — YOLO "pushable" 확정 시 직접 밀기 속도
DIRECT_PUSH_TIME   = 1.2    # s


class PushProbe(Node):
    """
    /obstacle_pusher/objects 구독 → pushable 필드에 따라 3경로 분기:

    "pushable"     — YOLO 확정 가벼운 물체 → 바로 밀기 (probe 없이)
    "not_pushable" — YOLO 확정 가구/고정물 → 즉시 우회 결정 퍼블리시
    "unknown"      — YOLO 미인식 (박스 등) → probe push 로 물리적 판단
    """

    def __init__(self):
        super().__init__("push_probe")

        self._objects: list[dict] = []
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._lock    = threading.Lock()
        self._probing = False

        self.create_subscription(String,   OBJECTS_TOPIC, self._objects_cb, 10)
        self.create_subscription(Odometry, ODOM_TOPIC,    self._odom_cb,    10)

        self._cmd_pub    = self.create_publisher(Twist,       CMDVEL_TOPIC,  10)
        self._result_pub = self.create_publisher(String,      RESULT_TOPIC,  10)
        self._marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC,  10)

        self.create_timer(0.2, self._check)
        self.get_logger().info("PushProbe 노드 시작")

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _objects_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._objects = data.get("objects", [])
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom_x = msg.pose.pose.position.x
            self._odom_y = msg.pose.pose.position.y

    # ── 주기 체크 & 분기 ──────────────────────────────────────────────────────

    def _push_enabled(self) -> bool:
        try:
            from pathlib import Path
            return Path(PUSH_ENABLE_FLAG).read_text().strip() == "1"
        except Exception:
            return True

    def _check(self):
        if self._probing:
            return
        if not self._push_enabled():
            return

        with self._lock:
            objects = list(self._objects)

        near = [o for o in objects if o.get("distance", 999) <= PROBE_TRIGGER_DIST]
        if not near:
            return

        target   = min(near, key=lambda o: o["distance"])
        pushable = target.get("pushable", "unknown")

        if pushable == "not_pushable":
            # YOLO가 의자/가구 확정 → 접촉 없이 즉시 우회
            self.get_logger().info(
                f"YOLO 확정 밀기불가 ({target['distance']:.2f}m) → 즉시 우회")
            self._publish_result("detour", "YOLO: 가구/고정물", 0.0, target)
            return

        if pushable == "pushable":
            # YOLO가 가벼운 물체 확정 → probe 없이 바로 밀기
            self.get_logger().info(
                f"YOLO 확정 가벼운물체 ({target['distance']:.2f}m) → 직접 밀기")
            threading.Thread(
                target=self._run_direct_push, args=(target,), daemon=True
            ).start()
            return

        # unknown → probe push (물리적 판단)
        self.get_logger().info(
            f"미인식 물체 ({target['distance']:.2f}m) → probe push 시작")
        threading.Thread(
            target=self._run_probe, args=(target,), daemon=True
        ).start()

    # ── 직접 밀기 (YOLO "pushable" 확정) ─────────────────────────────────────

    def _run_direct_push(self, target: dict):
        """YOLO가 가벼운 물체로 확정 → 물리 접촉 없이 바로 navigation에 위임."""
        self._probing = True
        try:
            self.get_logger().info("YOLO 확정 → navigation에 side-waypoint 전달")
            self._publish_result("push", "YOLO: 가벼운 물체 확정", 0.0, target)
        finally:
            self._probing = False

    # ── Probe Push (unknown → 물리적 판단) ───────────────────────────────────

    def _run_probe(self, target: dict):
        self._probing = True
        try:
            # ① navigation 일시 중단 요청 (Nav2와 cmd_vel 충돌 방지)
            self._publish_result("probe_start", "탐침 시작 — Nav2 일시 중단", 0.0, target)
            time.sleep(0.5)

            with self._lock:
                x0, y0 = self._odom_x, self._odom_y

            # ② 천천히 전진 (물체 저항 테스트)
            self.get_logger().info(
                f"→ 탐침 전진 {PROBE_SPEED} m/s × {PROBE_DURATION}s")
            cmd = Twist()
            cmd.linear.x = PROBE_SPEED
            t_end = time.time() + PROBE_DURATION
            while time.time() < t_end:
                self._cmd_pub.publish(cmd)
                time.sleep(0.05)

            # ③ 정지
            self._cmd_pub.publish(Twist())
            time.sleep(0.3)

            # ④ 실제 이동량 측정
            with self._lock:
                x1, y1 = self._odom_x, self._odom_y
            moved    = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
            expected = PROBE_SPEED * PROBE_DURATION
            self.get_logger().info(
                f"이동량: {moved:.3f}m / 예상: {expected:.3f}m")

            if moved >= PROBE_MOVE_THR:
                # 물체가 밀렸음 → navigation이 side-waypoint로 통과
                action = "push"
                reason = f"탐침: 가벼움 (이동 {moved:.2f}m)"
                self.get_logger().info(f"판정: 밀고 통과 — {reason}")
                self._retreat()   # 탐침 전진분 복귀 후 Nav2에 위임
            else:
                action = "detour"
                reason = f"탐침: 무거움 (이동 {moved:.2f}m < {PROBE_MOVE_THR}m)"
                self.get_logger().info(f"판정: 우회 — {reason}")
                self._retreat()

            self._publish_result(action, reason, moved, target)

        finally:
            self._probing = False

    def _retreat(self):
        cmd = Twist()
        cmd.linear.x = -PROBE_SPEED
        for _ in range(6):
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
        if action == "push":
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.2, 1.0
            src = "YOLO" if pushable == "pushable" else "탐침"
            m.text = f"밀고 통과 [{src}]"
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.2, 1.0
            src = "YOLO" if pushable == "not_pushable" else "탐침"
            m.text = f"우회 [{src}]"

        m.lifetime = Duration(sec=5)
        ma.markers.append(m)
        self._marker_pub.publish(ma)

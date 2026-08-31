import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from pathlib import Path as FilePath
import yaml
import os
import time
import math

from navigation_modifier import NavigationModifier

SOCIAL_NAV_FLAG  = '/tmp/social_nav_enabled'
OBSTACLE_TOPIC   = '/obstacle_pusher/decision'

REPLAN_INTERVAL  = 1.0   # 재평가 주기 (초)
REPLAN_MIN_DIST  = 0.8   # 목표까지 이 거리 이하면 재계획 안 함 (m)
PUSH_SIDE_OFFSET = 0.20  # m — 박스 중심에서 옆으로 스쳐 지나갈 오프셋


class NavigationClient(Node):
    def __init__(self, target_key):
        super().__init__(f'nav_client_{int(time.time())}')
        self._action_client        = ActionClient(self, NavigateToPose,       'navigate_to_pose')
        self._through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.stop_publisher = self.create_publisher(Twist,       '/stretch/cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path,        '/plan',            10)
        self._wp_pub        = self.create_publisher(MarkerArray, '/nav/waypoints',   10)

        self.vel_sub = self.create_subscription(Twist, '/stretch/cmd_vel', self.vel_callback, 10)

        self.last_turn_announcement = 0
        self.turn_threshold = 0.4

        self.target_key  = target_key
        self.goal_handle = None
        self.is_arrived  = False
        self.yaml_path   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/location.yaml")

        self._modifier          = NavigationModifier(self)
        self._goal_x            = 0.0
        self._goal_y            = 0.0
        self._goal_w            = 1.0
        self._navigating        = False
        # 1-B: Nav2 abort 감지·안내용 상태
        self.nav_failed         = False   # interface _watch가 폴링하는 실패 플래그
        self._failed_announced  = False   # 재진입 방어 (start_navigation에서 리셋)
        self._goal_gen          = 0       # goal 세대번호 (취소 후 낡은 콜백 필터)
        self._nav_fail_status   = 0
        self._last_had_wp       = False   # 이전 주기에 경유지 있었는지
        self._replan_timer      = None
        self._obstacle_push_wp: tuple[float, float] | None = None  # 박스 side-waypoint
        self._probing           = False   # probe 중 Nav2 일시 중단 플래그
        self._social_nav_flag_logged = False   # SOCIAL_NAV_FLAG 부재/실패 로그 1회 제한

        self.create_subscription(
            StringMsg, OBSTACLE_TOPIC, self._obstacle_cb, 10)

    # ── 위치 로드 ──────────────────────────────────────────────────────────────

    def load_location(self):
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            return data['locations'].get(self.target_key)
        except:
            return None

    # ── 내비게이션 시작 ────────────────────────────────────────────────────────

    def start_navigation(self):
        loc = self.load_location()
        if not loc:
            print(f"[NAV] 목적지 '{self.target_key}' 을 location.yaml에서 찾을 수 없음", flush=True)
            return False

        for srv in ['/global_costmap/clear_entirely_global_costmap',
                    '/local_costmap/clear_entirely_local_costmap']:
            cli = self.create_client(Trigger, srv)
            if cli.wait_for_service(timeout_sec=0.5):
                cli.call_async(Trigger.Request())

        self._goal_x    = float(loc['x'])
        self._goal_y    = float(loc['y'])
        self._goal_w    = float(loc.get('w', 1.0))
        # z: 도착 방향의 쿼터니언 z 성분 — w만으로는 방향이 표현되지 않음
        # (yaw 쿼터니언은 z·w 쌍). yaml에 z가 없으면 0.0 = 기존 동작 유지.
        self._goal_z    = float(loc.get('z', 0.0))
        self._navigating = True
        self.is_arrived  = False
        # 1-B: 새 주행 = 실패 상태·재진입 플래그 리셋 (새 goal = 새 안내 자격)
        self.nav_failed        = False
        self._failed_announced = False
        FilePath("/tmp/navigation_active").write_text("1")

        # 1-B: 서버 대기는 여기서 1회만 블로킹. 콜백(_replan_check·_obstacle_cb)에서
        # blocking wait를 부르면 단일스레드 executor가 최대 10초 멈춰 실패 안내까지
        # 지연되므로, 대기를 여기로 분리하고 _send_to_nav2는 논블로킹 검사만 한다.
        self._action_client.wait_for_server(timeout_sec=10.0)
        self._through_poses_client.wait_for_server(timeout_sec=2.0)

        # 초기 전송
        waypoints = self._compute_waypoints_if_enabled(self._social_nav_enabled())
        ok = self._send_to_nav2(waypoints)
        if not ok:
            self._navigating = False
            return False

        # 이동 중 주기적 재평가 타이머
        self._replan_timer = self.create_timer(REPLAN_INTERVAL, self._replan_check)
        return True

    # ── 주기적 재평가 ──────────────────────────────────────────────────────────

    # ── 장애물 결정 수신 ──────────────────────────────────────────────────────

    def _obstacle_cb(self, msg: StringMsg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        action = data.get("action")

        if action == "probe_start":
            # 탐침 시작 → Nav2 일시 중단 (cmd_vel 충돌 방지)
            self._probing = True
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            print("[NAV] 탐침 시작 — 내비게이션 일시 중단", flush=True)
            return

        self._probing = False

        if action == "push_through":
            # 이미 치고 통과 → side-waypoint 없이 현재 위치에서 목적지 직행
            self._obstacle_push_wp = None
            print("[NAV] 치고 통과 완료 → 현재 위치에서 목적지 직행", flush=True)
        elif action == "push":
            box_x = data.get("map_x", 0.0)
            box_y = data.get("map_y", 0.0)
            self._obstacle_push_wp = self._box_side_waypoint(box_x, box_y)
            sx, sy = self._obstacle_push_wp
            print(f"[NAV] 밀기 결정 → 박스 옆 경유지 ({sx:.2f}, {sy:.2f})", flush=True)
        elif action == "detour":
            self._obstacle_push_wp = None
            print("[NAV] 우회 결정 → 기존 경로 유지", flush=True)

        # 내비게이션 중이면 즉시 재계획 (social 여부와 무관하게 장애물 대응은 온전해야 함)
        if self._navigating and not self.is_arrived:
            waypoints = self._compute_waypoints_if_enabled(self._social_nav_enabled())
            if not self._send_to_nav2(waypoints):
                if self.goal_handle is None:      # 살아있는 goal 없음 = 진짜 막다른 길
                    self._nav_failed(-1, "resend_failed")
                else:                             # 기존 goal 유효 → 경로 갱신만 실패, 계속 주행
                    print("[NAV] 재전송 실패 — 기존 goal 유지, 계속 주행", flush=True)

    def _box_side_waypoint(self, box_x: float, box_y: float) -> tuple[float, float]:
        """
        로봇 현재 위치 → 박스 방향 기준 오른쪽으로 PUSH_SIDE_OFFSET 이동.
        로봇이 박스 가장자리를 스쳐 지나가면서 자연스럽게 밀어냄.
        """
        rx = self._modifier._robot_x
        ry = self._modifier._robot_y
        dx = box_x - rx
        dy = box_y - ry
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 0.01:
            return box_x, box_y
        # 진행방향 기준 오른쪽 (시계방향 90도)
        right_x = dy / dist
        right_y = -dx / dist
        return (
            box_x + right_x * PUSH_SIDE_OFFSET,
            box_y + right_y * PUSH_SIDE_OFFSET,
        )

    # ── 주기적 재평가 ──────────────────────────────────────────────────────────

    def _replan_check(self):
        if not self._navigating or self.is_arrived or self._probing:
            return

        # 목표 근처면 재계획 생략
        rx = self._modifier._robot_x
        ry = self._modifier._robot_y
        dist_to_goal = ((rx - self._goal_x)**2 + (ry - self._goal_y)**2) ** 0.5
        if dist_to_goal < REPLAN_MIN_DIST:
            return

        # 사이클당 1회만 평가 — waypoints 계산과 approaching 판정이 각자 파일을
        # 다시 읽으면 그 사이 토글돼 반쪽상태(레이스)가 생길 수 있었음(#24)
        social_on = self._social_nav_enabled()
        waypoints   = self._compute_waypoints_if_enabled(social_on)
        now_has_wp  = len(waypoints) > 1

        # 접근자가 있으면 매 주기 경유지 갱신 (사람이 다가올수록 회피 방향 재계산).
        # social OFF면 애초에 회피 자체를 안 하므로 접근자 유무로 강제 재계획하면
        # 안 됨(게이트 우회 — social 끄고도 매초 재계획 로그가 찍히던 버그).
        approaching_present = social_on and any(
            p.get('classification') == 'approaching'
            for p in self._modifier._people
        )

        if now_has_wp == self._last_had_wp and not approaching_present:
            return

        self._last_had_wp = now_has_wp
        reason = "접근자 경유지 갱신" if approaching_present else "사람 상황 변화 감지"
        print(f"[NAV] {reason} → 경로 재계획", flush=True)

        # 현재 goal 취소 후 재전송
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        if not self._send_to_nav2(waypoints):
            self._nav_failed(-1, "resend_failed")

    # ── 헬퍼 ──────────────────────────────────────────────────────────────────

    def _social_nav_enabled(self) -> bool:
        """'/tmp/social_nav_enabled' 파일로 ON/OFF 확인. 없거나 읽기 실패하면
        OFF(fail-closed) — ON을 기본으로 두면 대시보드 없이 뜨거나 /tmp 청소
        직후에 UI 표시 없이 회피가 몰래 켜진다. 도배 방지로 로그는 1회만."""
        try:
            return FilePath(SOCIAL_NAV_FLAG).read_text().strip() == '1'
        except Exception:
            if not self._social_nav_flag_logged:
                self._social_nav_flag_logged = True
                print(f"[NAV] {SOCIAL_NAV_FLAG} 없음/읽기실패 → social nav OFF(fail-closed)",
                      flush=True)
            return False

    def _compute_waypoints_if_enabled(self, social_on: bool) -> list[tuple[float, float]]:
        if social_on:
            waypoints = self._modifier.compute_waypoints(self._goal_x, self._goal_y)
        else:
            waypoints = [(self._goal_x, self._goal_y)]

        # 박스 side-waypoint를 경로 앞쪽에 삽입
        if self._obstacle_push_wp:
            waypoints = [self._obstacle_push_wp] + waypoints

        return waypoints

    def _publish_waypoint_markers(self, waypoints: list[tuple[float, float]]):
        """중간 경유지를 RViz에 노란 구체로 표시."""
        ma = MarkerArray()
        clr = Marker()
        clr.action = Marker.DELETEALL
        clr.header.frame_id = 'map'
        ma.markers.append(clr)

        now = self.get_clock().now().to_msg()
        for i, (wx, wy) in enumerate(waypoints[:-1]):  # 최종 목표 제외
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp    = now
            sphere.ns, sphere.id   = 'wp_sphere', i
            sphere.type            = Marker.SPHERE
            sphere.action          = Marker.ADD
            sphere.pose.position.x = wx
            sphere.pose.position.y = wy
            sphere.pose.position.z = 0.5
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.45
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 1.0, 0.0, 0.9
            sphere.lifetime = Duration(sec=5)
            ma.markers.append(sphere)

            lbl = Marker()
            lbl.header.frame_id = 'map'
            lbl.header.stamp    = now
            lbl.ns, lbl.id      = 'wp_label', i + 100
            lbl.type            = Marker.TEXT_VIEW_FACING
            lbl.action          = Marker.ADD
            lbl.pose.position.x = wx
            lbl.pose.position.y = wy
            lbl.pose.position.z = 0.95
            lbl.pose.orientation.w = 1.0
            lbl.scale.z         = 0.28
            lbl.color.r = lbl.color.g = lbl.color.b = lbl.color.a = 1.0
            lbl.text            = f"경유지 {i+1}"
            lbl.lifetime        = Duration(sec=5)
            ma.markers.append(lbl)

        self._wp_pub.publish(ma)

    # ── Nav2 전송 (공통) ───────────────────────────────────────────────────────

    def _send_to_nav2(self, waypoints: list[tuple[float, float]]) -> bool:
        now = self.get_clock().now().to_msg()

        if len(waypoints) > 1:
            # 경유지 포함 → NavigateThroughPoses
            if not self._through_poses_client.server_is_ready():
                print("[NAV] navigate_through_poses 서버 없음 — NavigateToPose 폴백", flush=True)
                waypoints = [(self._goal_x, self._goal_y)]
            else:
                goal_msg = NavigateThroughPoses.Goal()
                goal_msg.poses = []
                for i, (wx, wy) in enumerate(waypoints):
                    ps = PoseStamped()
                    ps.header.frame_id = "map"
                    ps.header.stamp    = now
                    ps.pose.position.x = wx
                    ps.pose.position.y = wy
                    # 마지막 포즈(최종 목표)만 원래 방향 유지
                    _last = (i == len(waypoints) - 1)
                    ps.pose.orientation.w = self._goal_w if _last else 1.0
                    ps.pose.orientation.z = self._goal_z if _last else 0.0
                    goal_msg.poses.append(ps)
                print(f"[NAV] 경유지 {len(waypoints)-1}개 포함 경로 전송", flush=True)
                self._publish_waypoint_markers(waypoints)
                self._goal_gen += 1
                _gen = self._goal_gen
                self._through_poses_client.send_goal_async(goal_msg).add_done_callback(
                    lambda f, g=_gen: self.goal_response_callback(f, g))
                return True

        # 경유지 없음 → NavigateToPose
        if not self._action_client.server_is_ready():
            print("[NAV] navigate_to_pose 서버 응답 없음", flush=True)
            return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp    = now
        goal_msg.pose.pose.position.x = self._goal_x
        goal_msg.pose.pose.position.y = self._goal_y
        goal_msg.pose.pose.orientation.w = self._goal_w
        goal_msg.pose.pose.orientation.z = self._goal_z
        print("[NAV] 직행 경로 전송", flush=True)
        self._goal_gen += 1
        _gen = self._goal_gen
        self._action_client.send_goal_async(goal_msg).add_done_callback(
            lambda f, g=_gen: self.goal_response_callback(f, g))
        return True

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def goal_response_callback(self, future, gen=0):
        # 낡은(취소된) goal의 응답이 뒤늦게 오면 무시 — 새 주행 오염 방지
        if gen != self._goal_gen:
            return
        try:
            gh = future.result()
        except Exception as e:   # rejected가 아니라 통신 실패 경로
            print(f"[NAV] goal 응답 예외: {type(e).__name__}: {e}", flush=True)
            self._nav_failed(-1, "goal_response_exception")
            return
        if gh is None or not gh.accepted:
            print("[NAV] goal 거절됨", flush=True)
            self._nav_failed(-1, "rejected")
            return
        self.goal_handle = gh
        self.goal_handle.get_result_async().add_done_callback(
            lambda f, g=gen: self.get_result_callback(f, g))

    def get_result_callback(self, future, gen=0):
        # 취소 후 재전송 경합: 낡은 goal의 결과는 무시
        if gen != self._goal_gen:
            return
        try:
            status = future.result().status
        except Exception as e:
            print(f"[NAV] 결과 수신 예외: {type(e).__name__}: {e}", flush=True)
            self._nav_failed(-1, "result_exception")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.is_arrived  = True
            self._navigating = False
            self._stop_replan_timer()
            FilePath("/tmp/navigation_active").write_text("0")
        elif status == GoalStatus.STATUS_CANCELED:
            # 재계획·장애물 회피가 일부러 취소한 것 → 실패 아님, 조용히 무시
            return
        else:
            # ABORTED(6) 등 진짜 실패 → 사용자에게 안내
            self._nav_failed(status, "aborted")

    def _nav_failed(self, status, reason):
        # 재진입 방어: 같은 주행에서 여러 경로(응답예외+결과ABORTED)로 2번 들어와도 1회만
        if self._failed_announced:
            return
        self._failed_announced = True
        self._nav_fail_status  = status
        self._navigating       = False
        self._stop_replan_timer()
        try:
            FilePath("/tmp/navigation_active").write_text("0")
        except Exception:
            pass
        # status 정수 그대로 로그 → robot_diag_nav의 /rosout과 시각 정렬 가능
        print(f"[NAV] 목표 실패 status={status} reason={reason}", flush=True)
        # print는 대시보드 메모리 로그(휘발)뿐 → /rosout에도 남겨 블랙박스에 영속
        self.get_logger().error(f"[NAV] 목표 실패 status={status} reason={reason}")
        # 마지막에 플래그 세움 — interface _watch가 이걸 보고 안내 (부분상태 노출 방지)
        self.nav_failed = True

    def vel_callback(self, msg):
        current_time = time.time()
        angular_z = msg.angular.z
        if abs(angular_z) > self.turn_threshold:
            if current_time - self.last_turn_announcement > 5.0:
                direction   = "왼쪽" if angular_z > 0 else "오른쪽"
                deg_per_sec = abs(math.degrees(angular_z))
                self.get_logger().info(f"[동작 안내] {direction}으로 회전 중 ({deg_per_sec:.1f}°/s)")
                self.last_turn_announcement = current_time

    # ── 정리 ──────────────────────────────────────────────────────────────────

    def _stop_replan_timer(self):
        if self._replan_timer:
            self._replan_timer.cancel()
            self._replan_timer = None

    def cleanup(self):
        self._navigating = False
        self._stop_replan_timer()
        FilePath("/tmp/navigation_active").write_text("0")
        try:
            empty_path = Path()
            empty_path.header.frame_id = "map"
            self.path_publisher.publish(empty_path)
            stop_msg = Twist()
            for _ in range(5):
                self.stop_publisher.publish(stop_msg)
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
        except:
            pass

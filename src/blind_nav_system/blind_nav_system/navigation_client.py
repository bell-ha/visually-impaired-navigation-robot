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

from tf2_ros import Buffer, TransformListener

from navigation_modifier import NavigationModifier

SOCIAL_NAV_FLAG  = '/tmp/social_nav_enabled'
SOCIAL_NAV_SUSPEND = '/tmp/social_nav_suspend'   # 여정이 구간별로 잠시 끈다(대시보드가 씀)
OBSTACLE_TOPIC   = '/obstacle_pusher/decision'

REPLAN_INTERVAL  = 1.0   # 재평가 주기 (초)
REPLAN_MIN_DIST  = 0.8   # 목표까지 이 거리 이하면 재계획 안 함 (m)
# 🔴 2026-10-14 — 경유지가 생겼다 사라졌다 하면 **매초 경로 종류가 바뀌고**, 전환마다
#   이전 goal 을 취소하므로 그때마다 컨트롤러가 로봇을 세운다(실측 17:00:55~17:01:08
#   사이 전환 5회). 사람이 조금 움직여 코스트맵 비용이 임계를 넘나든 것이 원인이었다.
#   그래서 '경유지 있음 → 없음' 방향만 이만큼 붙잡는다. 반대 방향(회피 시작)은
#   즉시 반응해야 하므로 지연을 두지 않는다.
WP_HOLD_SEC      = 3.0
PUSH_SIDE_OFFSET = 0.20  # m — 박스 중심에서 옆으로 스쳐 지나갈 오프셋

# 🔴 경유지 경로 전용 BT (2026-10-14 안전). nav2 기본 through_poses BT 에는
#   <Spin 1.57rad> 과 <BackUp 0.30m> 복구가 들어 있는데, 시각장애인이 로봇 **뒤에서**
#   손잡이를 잡고 걷는다 — 후진은 그대로 충돌이다. navigate_to_pose 쪽은 e4d48f0 에서
#   이미 뺐지만 through_poses 쪽은 yaml 에 설정이 없어 기본 BT 가 쓰이고 있었다.
#   드러나지 않은 이유: 경유지 경로가 좌표계 버그로 한 번도 실행된 적이 없었다(326ae76).
#   ⚠ yaml 이 아니라 **goal 의 behavior_tree 필드**로 넘긴다 — nav2 재시작이 필요 없다.
THROUGH_POSES_BT = ("/home/hello-robot/ament_ws/src/stretch_ros2/stretch_nav2/"
                    "behavior_trees/navigate_through_poses_no_backup.xml")


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

        # 🔴 2026-10-14 — local_costmap 은 **odom 프레임**인데(global_frame: odom,
        #   rolling_window) 경유지 좌표는 **map 프레임**이다. 그대로 조회하면 셀 번호가
        #   음수가 되어 '범위 밖 = 점령'으로 떨어졌고, 회피 경유지가 **매번 조용히
        #   폐기**됐다. 9/04~10/14 Nav2 목표 72건이 전부 '직행'이었던 진짜 원인이다.
        #   변환에 쓸 TF 버퍼. map→odom 은 AMCL 이 늘 쏜다.
        # 지금 떠 있는 goal 이 어느 액션 서버의 것인가 — 'pose' | 'poses' | None.
        # bt_navigator 는 BT 엔진이 **하나**라 서버가 다르면 새 goal 을 거부한다(아래).
        self._active_server = None
        self._pending_wp    = None   # 취소 완료를 기다리는 중인 경유지 목록
        self._pending_ts    = 0.0
        self._wp_hold_until = 0.0   # 이 시각까지는 '경유지 없음' 으로 되돌리지 않는다
        self._tf_buf      = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
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

        # 서버 전환 취소 응답을 기다리는 중 — 3초 넘게 안 오면 강제로 보낸다.
        # (취소 콜백이 유실되면 살아있는 goal 없이 영원히 서 있게 된다)
        if self._pending_wp is not None:
            if time.monotonic() - self._pending_ts < 3.0:
                return
            print("[NAV] 전환 취소 응답 지연 — 강제 전송", flush=True)
            self._send_after_cancel()
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

        # 깜빡임 억제 — 위 WP_HOLD_SEC 주석 참조. 경유지가 잠깐 안 잡혀도 바로
        # 직행으로 되돌리지 않는다(되돌릴 때마다 goal 이 취소돼 로봇이 선다).
        if now_has_wp:
            self._wp_hold_until = time.monotonic() + WP_HOLD_SEC
        elif self._last_had_wp and time.monotonic() < self._wp_hold_until:
            return

        # 접근자가 있으면 매 주기 경유지 갱신 (사람이 다가올수록 회피 방향 재계산).
        # social OFF면 애초에 회피 자체를 안 하므로 접근자 유무로 강제 재계획하면
        # 안 됨(게이트 우회 — social 끄고도 매초 재계획 로그가 찍히던 버그).
        # 🔴 2026-10-14 — **경유지 생성과 같은 조건을 쓴다**(approaching_in_range).
        #   종전에는 여기만 거리 조건이 없어서, 7m 밖 접근자 하나로 재계획을
        #   강제하면서 정작 경유지는 안 만들었다. 트리거와 결과가 어긋나면
        #   '멈추기만 하고 얻는 것이 없는' 구간이 생긴다.
        approaching_present = social_on and bool(self._modifier.approaching_in_range())

        if now_has_wp == self._last_had_wp and not approaching_present:
            return

        self._last_had_wp = now_has_wp
        reason = "접근자 경유지 갱신" if approaching_present else "사람 상황 변화 감지"
        print(f"[NAV] {reason} → 경로 재계획", flush=True)

        # 🔴 2026-10-14 — **취소하지 않고 그냥 보낸다.** nav2 가 새 goal 로 선점하고,
        #   밀려난 goal 의 결과는 _goal_gen 가드와 STATUS_CANCELED 분기가 이미 걸러낸다.
        #   종전에는 cancel_goal_async() 직후 곧바로 보냈는데, bt_navigator 가 아직
        #   이전 BT 를 정리하는 중이라 **새 goal 이 24ms 만에 ABORTED 로 죽었다**:
        #       15:19:00.275 취소 → 직행 전송 / 15:19:00.299 status=6 aborted
        #   그 뒤 살아있는 goal 이 없어 로봇은 20.7m 를 남긴 채 그대로 섰다.
        #   같은 파일의 _obstacle_cb 는 처음부터 취소 없이 재전송했다 — 그 패턴을 따른다.
        if not self._send_to_nav2(waypoints):
            if self.goal_handle is None:      # 살아있는 goal 없음 = 진짜 막다른 길
                self._nav_failed(-1, "resend_failed")
            else:                             # 기존 goal 유효 → 경로 갱신만 실패, 계속 주행
                print("[NAV] 재전송 실패 — 기존 goal 유지, 계속 주행", flush=True)

    # ── 헬퍼 ──────────────────────────────────────────────────────────────────

    def _social_nav_enabled(self) -> bool:
        """'/tmp/social_nav_enabled' 파일로 ON/OFF 확인. 없거나 읽기 실패하면
        OFF(fail-closed) — ON을 기본으로 두면 대시보드 없이 뜨거나 /tmp 청소
        직후에 UI 표시 없이 회피가 몰래 켜진다. 도배 방지로 로그는 1회만."""
        # 🔴 2026-10-14 — 여정이 엘리베이터 구간에 들어가면 대시보드가 suspend 를 켠다.
        #   사용자 토글(SOCIAL_NAV_FLAG)은 **건드리지 않는다** — 토글을 직접 뒤집으면
        #   화면과 파일이 또 갈라진다(674683f 에서 고친 사고). 파일 없음 = 중단 아님.
        try:
            if FilePath(SOCIAL_NAV_SUSPEND).read_text().strip() == '1':
                return False
        except Exception:
            pass
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
        """경로를 보낸다. **액션 서버가 바뀌는 경우에만** 기존 goal 을 먼저 끝낸다.

        🔴 2026-10-14 실측 — bt_navigator 는 BT 엔진이 하나라 다른 네비게이터가 돌고
        있으면 새 goal 을 **거부**한다:
            16:41:24.301 [NAV] 경유지 1개 포함 경로 전송
            16:41:24.308 [ERROR][bt_navigator] Requested navigation from
                         navigate_through_poses while another navigator is processing,
                         rejecting request.
            16:41:24.309 목표 실패 status=6 aborted      ← 8ms
        같은 서버끼리는 선점이 된다(같은 날 16:41:04 'Received goal preemption request').
        그래서 **종류가 바뀔 때만** 취소하고, 취소가 끝난 뒤에 보낸다."""
        want = ("poses" if (len(waypoints) > 1 and self._through_poses_client.server_is_ready())
                else "pose")
        if self.goal_handle is not None and self._active_server not in (None, want):
            gh, self.goal_handle = self.goal_handle, None
            self._pending_wp = list(waypoints)
            self._pending_ts = time.monotonic()
            try:
                # 🔴 취소 **응답**에 전송을 매달지 않는다. cancel_goal_async 의 future 는
                #    '취소 요청을 접수했다' 에서 완료되고, goal 이 실제로 끝나는 것은 그
                #    뒤다. 거기에 매달았다가 같은 밀리초에 보내 또 거부당했다:
                #      16:57:56.487 전환 → 경유지 전송 / .491 rejecting / .508 Goal canceled
                #    보낼 자리는 **이전 goal 의 결과가 도착한 시점**(get_result_callback)이다.
                gh.cancel_goal_async()
                print(f"[NAV] 경로 종류 전환({self._active_server}→{want}) — 기존 goal 종료 대기",
                      flush=True)
                return True
            except Exception as e:
                self._pending_wp = None
                print(f"[NAV] 전환 취소 요청 실패({type(e).__name__}) — 그대로 전송", flush=True)
        return self._send_now(waypoints)

    def _send_after_cancel(self):
        """취소가 **끝난 뒤** 미뤄 둔 경로를 보낸다."""
        wp, self._pending_wp = self._pending_wp, None
        if not wp or not self._navigating or self.is_arrived:
            return
        if not self._send_now(wp):
            self._nav_failed(-1, "resend_failed")

    def _send_now(self, waypoints: list[tuple[float, float]]) -> bool:
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
                goal_msg.behavior_tree = THROUGH_POSES_BT   # 후진·회전 없는 복구 BT
                print(f"[NAV] 경유지 {len(waypoints)-1}개 포함 경로 전송", flush=True)
                self._publish_waypoint_markers(waypoints)
                self._active_server = "poses"
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
        self._active_server = "pose"
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
        # 🔴 종류 전환으로 미뤄 둔 경로가 있으면 **여기가** 보낼 자리다 — 이전 goal 이
        #    terminal 에 도달했으므로 bt_navigator 가 비어 있다. 도착(SUCCEEDED)이면
        #    보낼 이유가 없으니 버리고 정상 처리로 내려간다.
        if self._pending_wp is not None and status != GoalStatus.STATUS_SUCCEEDED:
            self._active_server = None
            print(f"[NAV] 이전 goal 종료(status={status}) — 미뤄 둔 경로 전송", flush=True)
            self._send_after_cancel()
            return
        self._pending_wp = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._active_server = None
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
        self._active_server    = None
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

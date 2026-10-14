from __future__ import annotations
import json
import math

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

PEOPLE_TOPIC          = '/people_tracker/people'
COSTMAP_TOPIC         = '/local_costmap/costmap'

APPROACH_DIST_MAX     = 7.0   # m — 이 거리 안의 접근자만 반응
COMPANION_DIST_MAX    = 4.0   # m — 이 거리 안의 동행자만 힌트로 사용
AVOIDANCE_OFFSET      = 0.6   # m — 좌우 오프셋 거리
AVOIDANCE_LOOKAHEAD   = 2.0   # m — 회피 경유지를 앞으로 얼마나 투영할지
COMPANION_LOOKAHEAD   = 1.5   # m — 동행자 힌트 경유지 앞쪽 거리
COMPANION_BLEND       = 0.4   # 동행자 방향 가중치 (0=목표만, 1=동행자만)
OCCUPIED_THRESHOLD    = 65    # costmap 비용 기준 (이 이상이면 통행 불가)
RIGHT_PREFERENCE_BIAS = 20    # 오른쪽 선호 마진 — 오른쪽 비용이 이 값 이하 더 높아도 오른쪽 선택


class NavigationModifier:
    """
    /people_tracker/people 토픽을 구독해서
    접근자/동행자에 따라 Nav2 경유지를 수정한다.

    - 접근자: costmap 기반으로 더 열린 쪽(기본 오른쪽)으로 오프셋 경유지 삽입
    - 동행자: 속도 방향을 목표 방향과 블렌딩해서 경유지 삽입
    """

    def __init__(self, node):
        self._node = node
        self._people: list[dict] = []
        self._robot_x    = 0.0
        self._robot_y    = 0.0
        self._robot_yaw  = 0.0
        self._costmap: OccupancyGrid | None = None

        node.create_subscription(String,       PEOPLE_TOPIC,  self._people_cb,  10)
        node.create_subscription(OccupancyGrid, COSTMAP_TOPIC, self._costmap_cb, 10)

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _people_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._people    = data.get('people', [])
            self._robot_x   = data.get('robot_x')  or 0.0
            self._robot_y   = data.get('robot_y')  or 0.0
            self._robot_yaw = data.get('robot_yaw') or 0.0
        except Exception:
            pass

    def _costmap_cb(self, msg: OccupancyGrid):
        self._costmap = msg

    # ── 공개 API ───────────────────────────────────────────────────────────────

    def _is_ahead(self, p: dict) -> bool:
        """로봇 **앞쪽**에 있나. 뒤에 있는 사람은 회피·동행 계산에서 뺀다.

        🔴 2026-10-14 사용자 확인: **안내받는 시각장애인은 로봇 뒤에 서서 손잡이를
        잡고 걷는다.** 그 사람은 정의상 로봇과 같은 방향으로 움직이므로 완벽한
        '동행자'로 분류되는데, 그러면 로봇이 **자기가 데려가는 사람의 방향을 따라
        경로를 휘는** 되먹임이 된다. 뒤따라오는 사람은 길을 알려 주지 않는다.
        접근자도 같다 — 뒤에서 반대 방향으로 가는 사람은 우리에게서 **멀어지는**
        중이라 피할 대상이 아닌데, 종전에는 그들도 재계획을 깨웠다.

        ⚠ 로봇 자세를 모르면(추적기가 TF 를 못 얻어 distance 를 -1 로 보낸 경우)
        **앞쪽으로 친다**(fail-open). 모른다고 회피를 조용히 꺼 버리는 것이 더 나쁘다."""
        d = p.get('distance')
        if d is None or d < 0:
            return True                      # 로봇 자세 미상 — 거르지 않는다
        yaw = math.radians(self._robot_yaw)
        dx = (p.get('x') or 0.0) - self._robot_x
        dy = (p.get('y') or 0.0) - self._robot_y
        return (dx * math.cos(yaw) + dy * math.sin(yaw)) > 0.0

    def approaching_in_range(self) -> list[dict]:
        """회피 대상이 되는 접근자. **재계획 트리거와 경유지 생성이 같은 조건을
        쓰게 하는 단일 출처다.**

        🔴 2026-10-14 사고: 트리거(navigation_client._replan_check)가 거리 조건을
        안 보고 `classification == 'approaching'` 만 봤다. 그래서 7m 밖 접근자
        하나로 **매 초 goal 을 취소·재전송하면서 경유지는 만들지 않았다**
        — 얻는 것 없이 로봇만 섰다. 실측:
            15:19:00.275 [NAV] 접근자 경유지 갱신 → 경로 재계획
            15:19:00.275 [NAV] 직행 경로 전송      ← 경유지 0개
            15:19:00.299 [NAV] 목표 실패 status=6 aborted
        그 뒤 승차지점까지 20.7m 를 남긴 채 멈춰 있었다.
        조건을 바꿀 일이 생기면 **여기만** 고친다."""
        return [p for p in self._people
                if p['classification'] == 'approaching'
                and p['distance'] < APPROACH_DIST_MAX
                and self._is_ahead(p)]

    def compute_waypoints(self, goal_x: float, goal_y: float) -> list[tuple[float, float]]:
        """
        목표 (goal_x, goal_y)로 가는 경유지 목록 반환.
        마지막 원소는 항상 원래 목표. 경유지가 없으면 [(goal_x, goal_y)].
        """
        try:
            approaching = self.approaching_in_range()
            companions  = [p for p in self._people
                           if p['classification'] == 'companion'
                           and p['distance'] < COMPANION_DIST_MAX
                           and self._is_ahead(p)]

            waypoints: list[tuple[float, float]] = []

            if approaching:
                wp = self._avoidance_waypoint(approaching)
                if wp and self._is_navigable(*wp):
                    waypoints.append(wp)
                    self._node.get_logger().info(
                        f"[Modifier] 접근자 {len(approaching)}명 감지 → 회피 경유지 ({wp[0]:.2f}, {wp[1]:.2f})")
                else:
                    # 🔴 **조용히 버리지 않는다.** 이 자리가 로그를 안 남겨서, 트리거는
                    #   걸리는데 경유지가 안 생기는 상태를 몇 주 동안 못 봤다.
                    self._node.get_logger().warn(
                        f"[Modifier] 접근자 {len(approaching)}명인데 회피 경유지 폐기 — "
                        + ("계산 실패" if not wp
                           else f"({wp[0]:.2f}, {wp[1]:.2f}) 비용 {self._costmap_cost(*wp)}"))

            # 접근자 처리 중에는 동행자 힌트 생략 (경유지 충돌 방지)
            if companions and not waypoints:
                wp = self._companion_waypoint(goal_x, goal_y, companions)
                if wp and self._is_navigable(*wp):
                    waypoints.append(wp)
                    self._node.get_logger().info(
                        f"[Modifier] 동행자 {len(companions)}명 감지 → 경로 힌트 ({wp[0]:.2f}, {wp[1]:.2f})")
                else:
                    self._node.get_logger().warn(
                        f"[Modifier] 동행자 {len(companions)}명인데 경로 힌트 폐기 — "
                        + ("계산 실패(정지·목표 근접·90°↑ 어긋남)" if not wp
                           else f"({wp[0]:.2f}, {wp[1]:.2f}) 비용 {self._costmap_cost(*wp)}"))

            waypoints.append((goal_x, goal_y))
            return waypoints
        except Exception as e:
            # people_tracker가 키 빠진 dict를 한 번만 내도(예: classification/distance
            # 누락) 여기서 죽으면 _replan_check(1초 타이머)가 매초 재발시켜 스핀
            # 스레드를 계속 위협함 — 회피 없이 목표만 반환해 주행은 계속되게 한다.
            self._node.get_logger().error(
                f"[Modifier] compute_waypoints 예외 → 회피 없이 목표만 반환: {e!r}")
            return [(goal_x, goal_y)]

    # ── 내부 계산 ──────────────────────────────────────────────────────────────

    def _avoidance_waypoint(self, approaching: list[dict]) -> tuple[float, float] | None:
        """접근자가 있을 때 더 열린 쪽으로 오프셋 경유지 계산."""
        yaw_rad = math.radians(self._robot_yaw)

        # 로봇 진행 방향 / 오른쪽 / 왼쪽 단위벡터 (map frame)
        forward = ( math.cos(yaw_rad), math.sin(yaw_rad))
        right   = ( math.cos(yaw_rad - math.pi / 2),
                    math.sin(yaw_rad - math.pi / 2))
        left    = (-right[0], -right[1])

        # 좌우 결정은 로봇 바로 옆에서 — 벽/장애물 감지 확실
        right_near = (self._robot_x + right[0] * AVOIDANCE_OFFSET,
                      self._robot_y + right[1] * AVOIDANCE_OFFSET)
        left_near  = (self._robot_x + left[0]  * AVOIDANCE_OFFSET,
                      self._robot_y + left[1]  * AVOIDANCE_OFFSET)

        right_cost = self._costmap_cost(*right_near)
        left_cost  = self._costmap_cost(*left_near)

        # 오른쪽 선호 (사회적 규범), 오른쪽 비용이 크게 높으면 왼쪽
        side = right if right_cost <= left_cost + RIGHT_PREFERENCE_BIAS else left

        # 경유지는 앞으로 투영 — 로봇이 복도 한쪽을 유지하며 자연스럽게 통과
        return (self._robot_x + forward[0] * AVOIDANCE_LOOKAHEAD + side[0] * AVOIDANCE_OFFSET,
                self._robot_y + forward[1] * AVOIDANCE_LOOKAHEAD + side[1] * AVOIDANCE_OFFSET)

    def _companion_waypoint(self, goal_x: float, goal_y: float,
                            companions: list[dict]) -> tuple[float, float] | None:
        """동행자 속도 방향과 목표 방향을 블렌딩한 경유지 계산."""
        avg_vx = sum(p['vx'] for p in companions) / len(companions)
        avg_vy = sum(p['vy'] for p in companions) / len(companions)
        spd = (avg_vx**2 + avg_vy**2) ** 0.5
        if spd < 0.01:
            return None

        comp_dx, comp_dy = avg_vx / spd, avg_vy / spd

        dx = goal_x - self._robot_x
        dy = goal_y - self._robot_y
        dist = (dx**2 + dy**2) ** 0.5
        if dist < 0.5:
            return None
        goal_dx, goal_dy = dx / dist, dy / dist

        # 🔴 2026-10-14 — **정합도로 감쇠한다.** 종전에는 동행자가 어디로 가든
        #   무조건 40% 를 섞었다. 그런데 '동행자' 판정은 **로봇이 바라보는 방향
        #   ±90°**(= 180° 부채꼴)라서, 목표와 89° 어긋난 사람도 동행자다. 그 40%
        #   가 그대로 들어가면 경로가 33.4° 휘고 1.5m 앞에서 **83cm** 벗어난다
        #   (복도에서는 벽에 붙는 거리다). 실제로 따라갈 가치는 '그 사람이 우리
        #   목표 쪽으로 가고 있는 정도'에 비례하므로 cos 을 곱한다.
        #     θ=0° → 0.40 (종전과 동일)   45° → 0.28   75° → 0.10   90°↑ → 따라가지 않음
        #   임계로 끊지 않고 연속 감쇠인 이유: 경계에서 켜졌다 꺼졌다 하면
        #   now_has_wp 가 매 초 뒤집혀 재계획이 폭주한다(위 트리거와 같은 사고).
        align = goal_dx * comp_dx + goal_dy * comp_dy      # = cos θ (둘 다 단위벡터)
        if align <= 0:
            return None        # 목표와 90° 이상 어긋난 동행자는 따라가지 않는다
        blend = COMPANION_BLEND * align
        bx = (1 - blend) * goal_dx + blend * comp_dx
        by = (1 - blend) * goal_dy + blend * comp_dy
        bs = (bx**2 + by**2) ** 0.5
        if bs < 0.01:
            return None

        return (self._robot_x + (bx / bs) * COMPANION_LOOKAHEAD,
                self._robot_y + (by / bs) * COMPANION_LOOKAHEAD)

    def _to_costmap_frame(self, x: float, y: float):
        """map 좌표를 코스트맵 프레임으로 옮긴다. 못 옮기면 None.

        🔴 2026-10-14 — **여기가 보행 흐름 주행이 한 번도 작동하지 않은 이유였다.**
        local_costmap 은 `global_frame: odom` 에 rolling_window 라 header.frame_id 가
        odom 인데, 경유지 좌표는 map 프레임이다(사람추적이 map 으로 보낸다).
        변환 없이 넣으면 셀 번호가 음수가 되어 '범위 밖' 으로 떨어졌다. 실측:
            costmap origin odom(-20.65, -6.90) · 경유지 map(-41, +3)
            cx = (-41 - (-20.65)) / 0.05 = -407  → 범위 밖 → 100(점령) → 폐기
        9/04~10/14 Nav2 목표 72건이 **전부** '직행' 이었던 진짜 원인이다.
        트리거는 제대로 걸렸고(로그에 '접근자 경유지 갱신'이 11번) 마지막 관문에서
        **로그 한 줄 없이** 버려졌다."""
        cm = self._costmap
        if cm is None:
            return None
        frame = (cm.header.frame_id or "map").lstrip("/")
        if frame == "map":
            return (x, y)
        try:
            import rclpy.time
            tr = self._node._tf_buf.lookup_transform(frame, "map", rclpy.time.Time())
        except Exception:
            return None                      # TF 미준비 — 모른다(아래에서 fail-open)
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cs, sn = math.cos(yaw), math.sin(yaw)
        return (t.x + x * cs - y * sn, t.y + x * sn + y * cs)

    def _costmap_cost(self, x: float, y: float) -> int:
        """map 좌표 (x, y)의 costmap 비용값. **모르면 0(통행 가능)으로 친다.**

        🔴 fail-open 이다. 종전에는 범위 밖을 100(점령)으로 쳤는데, 이 함수는
        '경유지를 둘 자리인가' 라는 **힌트**를 묻는 자리지 안전 게이트가 아니다
        (벽으로 돌진하는 것은 Nav2 가 막는다). 모름을 점령으로 치면 코스트맵이
        없거나·프레임을 못 옮기거나·7m 창 밖인 모든 경우에 회피가 통째로 죽는다 —
        실제로 그렇게 죽어 있었다."""
        cm = self._costmap
        if cm is None:
            return 0
        pt = self._to_costmap_frame(x, y)
        if pt is None:
            return 0                         # 변환 불가 — 막지 않는다
        info = cm.info
        cx = int((pt[0] - info.origin.position.x) / info.resolution)
        cy = int((pt[1] - info.origin.position.y) / info.resolution)
        if not (0 <= cx < info.width and 0 <= cy < info.height):
            return 0                         # 7m 창 밖 = 정보 없음 ≠ 점령
        v = int(cm.data[cy * info.width + cx])
        return 0 if v < 0 else v             # -1(미지)도 막지 않는다

    def _is_navigable(self, x: float, y: float) -> bool:
        return self._costmap_cost(x, y) < OCCUPIED_THRESHOLD

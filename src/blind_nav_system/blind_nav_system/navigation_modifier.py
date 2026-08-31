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

    def compute_waypoints(self, goal_x: float, goal_y: float) -> list[tuple[float, float]]:
        """
        목표 (goal_x, goal_y)로 가는 경유지 목록 반환.
        마지막 원소는 항상 원래 목표. 경유지가 없으면 [(goal_x, goal_y)].
        """
        try:
            approaching = [p for p in self._people
                           if p['classification'] == 'approaching'
                           and p['distance'] < APPROACH_DIST_MAX]
            companions  = [p for p in self._people
                           if p['classification'] == 'companion'
                           and p['distance'] < COMPANION_DIST_MAX]

            waypoints: list[tuple[float, float]] = []

            if approaching:
                wp = self._avoidance_waypoint(approaching)
                if wp and self._is_navigable(*wp):
                    waypoints.append(wp)
                    self._node.get_logger().info(
                        f"[Modifier] 접근자 {len(approaching)}명 감지 → 회피 경유지 ({wp[0]:.2f}, {wp[1]:.2f})")

            # 접근자 처리 중에는 동행자 힌트 생략 (경유지 충돌 방지)
            if companions and not waypoints:
                wp = self._companion_waypoint(goal_x, goal_y, companions)
                if wp and self._is_navigable(*wp):
                    waypoints.append(wp)
                    self._node.get_logger().info(
                        f"[Modifier] 동행자 {len(companions)}명 감지 → 경로 힌트 ({wp[0]:.2f}, {wp[1]:.2f})")

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

        # 목표 방향 (1-blend) + 동행자 방향 blend
        bx = (1 - COMPANION_BLEND) * goal_dx + COMPANION_BLEND * comp_dx
        by = (1 - COMPANION_BLEND) * goal_dy + COMPANION_BLEND * comp_dy
        bs = (bx**2 + by**2) ** 0.5
        if bs < 0.01:
            return None

        return (self._robot_x + (bx / bs) * COMPANION_LOOKAHEAD,
                self._robot_y + (by / bs) * COMPANION_LOOKAHEAD)

    def _costmap_cost(self, x: float, y: float) -> int:
        """map 좌표 (x, y)의 costmap 비용값 반환. costmap 없으면 0."""
        if self._costmap is None:
            return 0
        info = self._costmap.info
        cx = int((x - info.origin.position.x) / info.resolution)
        cy = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= cx < info.width and 0 <= cy < info.height):
            return 100  # 범위 밖 = 점령
        return int(self._costmap.data[cy * info.width + cx])

    def _is_navigable(self, x: float, y: float) -> bool:
        return self._costmap_cost(x, y) < OCCUPIED_THRESHOLD

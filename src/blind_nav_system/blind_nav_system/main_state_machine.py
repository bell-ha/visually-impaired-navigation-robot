#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_state_machine.py

interface.py의 NavSimulator를 대체하는 ROS2 실제 이동 모듈.

interface.py에서 사용하는 API (NavSimulator와 완전 동일):
    nav = NavSimulator(on_arrive=콜백)
    nav.start(dest_key: str) -> bool
    nav.stop()

내부 동작:
    start(dest_key)
        → NavigationClient(dest_key) 생성
        → costmap clear
        → NavigateToPose goal 전송
        → is_arrived 감시 스레드 시작
        → 성공 시 True, 실패 시 False

    stop()
        → /stretch/cmd_vel 에 Twist() 1초 퍼블리시
        → navigate_to_pose goal cancel
        → NavigationClient 노드 정리

    도착 감지
        → NavigationClient.is_arrived == True 이면 on_arrive() 호출
"""
from __future__ import annotations

import threading
import time
import sys
import importlib.util
from pathlib import Path
from typing import Callable, Optional

# ──────────────────────────────────────────────
# ROS2 import (없으면 ImportError → interface.py가 폴백 처리)
# ──────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from unique_identifier_msgs.msg import UUID
from builtin_interfaces.msg import Time as BuiltinTime

# ──────────────────────────────────────────────
# NavigationClient 동적 로드
# (같은 디렉터리의 navigation_client.py)
# ──────────────────────────────────────────────
_THIS_DIR = Path(__file__).resolve().parent

def _load_nav_client():
    p = _THIS_DIR / "navigation_client.py"
    if not p.exists():
        raise ImportError(f"navigation_client.py not found: {p}")
    spec = importlib.util.spec_from_file_location("navigation_client", str(p))
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.NavigationClient


CMD_VEL_TOPIC          = "/stretch/cmd_vel"
CANCEL_NAV_SRV         = "/navigate_to_pose/_action/cancel_goal"
NAV_STOP_DURATION_SEC  = 1.0   # 정지 명령을 퍼블리시할 시간
ARRIVE_POLL_SEC        = 0.1   # is_arrived 폴링 간격


# ══════════════════════════════════════════════
# _NavCore – ROS2 노드 (executor에 추가되는 단일 노드)
# ══════════════════════════════════════════════
class _NavCore(Node):
    """
    /stretch/cmd_vel 퍼블리셔와 goal cancel 서비스 클라이언트를 가진
    단일 ROS2 노드. NavigationClient 노드와 함께 같은 executor에서 동작.
    """
    def __init__(self):
        super().__init__("nav_state_machine_core")
        self.cmd_pub    = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.cancel_cli = self.create_client(CancelGoal, CANCEL_NAV_SRV)

    def publish_stop(self, duration_sec: float = NAV_STOP_DURATION_SEC) -> None:
        """정지 Twist를 duration_sec 동안 퍼블리시 (블로킹)."""
        msg    = Twist()
        end_t  = time.time() + duration_sec
        rate   = 10.0
        period = 1.0 / rate
        while time.time() < end_t:
            self.cmd_pub.publish(msg)
            time.sleep(period)

    def cancel_all_goals(self, timeout_sec: float = 1.0) -> None:
        """NavigateToPose goal 전체 취소 (비동기, 결과 무시)."""
        if not self.cancel_cli.service_is_ready():
            self.cancel_cli.wait_for_service(timeout_sec=0.3)
        if not self.cancel_cli.service_is_ready():
            return
        req = CancelGoal.Request()
        req.goal_info           = GoalInfo()
        req.goal_info.goal_id   = UUID(uuid=[0] * 16)   # match-all
        req.goal_info.stamp     = BuiltinTime(sec=0, nanosec=0)
        self.cancel_cli.call_async(req)


# ══════════════════════════════════════════════
# NavSimulator (실제 이름 유지 – interface.py 무수정)
# ══════════════════════════════════════════════
class NavSimulator:
    """
    interface.py의 NavSimulator와 완전히 동일한 인터페이스.
    내부적으로는 ROS2 + NavigationClient로 실제 로봇을 제어.

    사용법 (interface.py 쪽에서 변경 없음):
        from main_state_machine import NavSimulator
        nav = NavSimulator(on_arrive=self._on_arrive)
        nav.start("편의점")   # → bool
        nav.stop()
    """

    def __init__(
        self,
        on_arrive: Callable[[], None],
        debug: bool = False,
    ):
        self._on_arrive = on_arrive
        self._debug     = debug

        # ROS2 초기화
        if not rclpy.ok():
            rclpy.init()

        self._executor  = SingleThreadedExecutor()
        self._core      = _NavCore()
        self._executor.add_node(self._core)

        # executor spin 전용 스레드
        self._spin_thr = threading.Thread(
            target=self._executor.spin, daemon=True, name="nav_spin")
        self._spin_thr.start()

        # 현재 NavigationClient 노드
        self._nav_node: Optional[object] = None
        self._NavClient = None   # 지연 로드

        # 도착 감시 제어
        self._stop_watch = threading.Event()
        self._watch_thr: Optional[threading.Thread] = None

        # 중복 호출 방지 락
        self._lock = threading.Lock()

        self._log("ROS2 NavSimulator 준비 완료")

    # ──────────────────────────────────────────
    # public API
    # ──────────────────────────────────────────
    def start(self, dest_key: str) -> bool:
        """
        목적지(dest_key)로 이동 시작.
        성공 → True, 실패(좌표 없음·서버 미응답 등) → False.
        """
        with self._lock:
            self._stop_internal()          # 이전 이동이 있으면 먼저 정리
            return self._start_internal(dest_key)

    def stop(self) -> None:
        """이동 정지 및 정리."""
        with self._lock:
            self._stop_internal()

    def shutdown(self) -> None:
        """프로세스 종료 시 호출."""
        self.stop()
        try:
            self._executor.remove_node(self._core)
            self._core.destroy_node()
            self._executor.shutdown()
        except Exception:
            pass

    # ──────────────────────────────────────────
    # 내부 구현
    # ──────────────────────────────────────────
    def _start_internal(self, dest_key: str) -> bool:
        # NavigationClient 클래스 로드 (최초 1회)
        if self._NavClient is None:
            try:
                self._NavClient = _load_nav_client()
            except Exception as e:
                self._log(f"NavigationClient 로드 실패: {e}", err=True)
                return False

        # 노드 생성 및 executor 등록
        try:
            self._nav_node = self._NavClient(dest_key)
            self._executor.add_node(self._nav_node)
        except Exception as e:
            self._log(f"NavigationClient 생성 실패: {e}", err=True)
            self._nav_node = None
            return False

        # 회전 안내 패치 (vel_callback → 콘솔 로그만, TTS는 interface.py 담당 아님)
        # NavigationClient 내부 get_logger()로 이미 출력되므로 별도 패치 없음

        # goal 전송
        try:
            ok = bool(self._nav_node.start_navigation())
        except Exception as e:
            self._log(f"start_navigation 예외: {e}", err=True)
            ok = False

        if not ok:
            self._log("start_navigation 실패 → READY 유지")
            self._cleanup_nav_node()
            return False

        self._log(f"이동 시작 → {dest_key}")

        # 도착 감시 스레드
        self._stop_watch = threading.Event()
        stop_ev   = self._stop_watch
        nav_node  = self._nav_node
        on_arrive = self._on_arrive

        def _watch():
            while not stop_ev.is_set():
                if getattr(nav_node, "is_arrived", False):
                    self._log("도착 감지 → on_arrive 호출")
                    on_arrive()
                    return
                time.sleep(ARRIVE_POLL_SEC)

        self._watch_thr = threading.Thread(
            target=_watch, daemon=True, name="nav_arrive_watch")
        self._watch_thr.start()
        return True

    def _stop_internal(self) -> None:
        # 1) 도착 감시 중단
        self._stop_watch.set()

        # 2) 정지 명령 + goal cancel (별도 스레드로 – 블로킹 방지)
        if self._nav_node is not None:
            def _do_stop():
                try:
                    self._core.publish_stop(NAV_STOP_DURATION_SEC)
                except Exception:
                    pass
                try:
                    self._core.cancel_all_goals()
                except Exception:
                    pass
            threading.Thread(target=_do_stop, daemon=True).start()

        # 3) NavigationClient cleanup + 노드 제거
        self._cleanup_nav_node()
        self._log("이동 정지")

    def _cleanup_nav_node(self) -> None:
        if self._nav_node is None:
            return
        try:
            self._nav_node.cleanup()
        except Exception:
            pass
        try:
            self._executor.remove_node(self._nav_node)
        except Exception:
            pass
        try:
            self._nav_node.destroy_node()
        except Exception:
            pass
        self._nav_node = None

    def _log(self, msg: str, err: bool = False) -> None:
        tag = "[NAV-ERR]" if err else "[NAV]"
        out = sys.stderr if err else sys.stdout
        print(f"{tag} {msg}", file=out)
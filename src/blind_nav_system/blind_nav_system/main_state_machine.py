#!/usr/bin/env python3
"""
main_state_machine.py (thread-safe, non-blocking)

목표
- 이 파일은 '상태 전이 + 네비게이션 제어'만 담당
- 버튼/음성/키보드 등 입력은 외부에서 처리하고,
  여기 제공하는 public 메서드만 호출해서 상태를 바꾼다.
- autospin=True여도 안전하게 동작하도록:
  외부 호출은 "명령 큐"에 넣기만 하고,
  ROS 객체 접근은 executor(spin) 스레드에서만 수행한다.

상태
- LOCKED : 비활성/대기
- READY  : 목적지 입력/확인 대기
- NAV    : 이동 중
- PAUSED : 일시정지(Goal cancel + 정지)

외부 사용 예
    rclpy.init()
    sm = GuidanceStateMachine(autospin=True)

    sm.activate()
    sm.start_navigation("524")
    sm.pause()
    sm.resume()
    sm.abort()
"""

from __future__ import annotations

import time
import threading
import queue
from enum import Enum, auto
from dataclasses import dataclass
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from unique_identifier_msgs.msg import UUID
from builtin_interfaces.msg import Time as BuiltinTime

from navigation_client import NavigationClient

# ====== 환경에 맞게 고정 ======
CMD_VEL_TOPIC = "/stretch/cmd_vel"
CANCEL_NAV_TO_POSE_SRV = "/navigate_to_pose/_action/cancel_goal"


class State(Enum):
    LOCKED = auto()
    READY = auto()
    NAV = auto()
    PAUSED = auto()


TransitionCallback = Callable[[State, State, str], None]
# (old_state, new_state, reason) -> None


@dataclass(frozen=True)
class _Cmd:
    name: str
    arg: Optional[str] = None


class GuidanceStateMachine(Node):
    """
    - 외부 호출은 thread-safe: command queue에 enqueue만 함
    - 내부 처리는 executor thread에서만 수행
    """

    def __init__(
        self,
        executor: Optional[SingleThreadedExecutor] = None,
        autospin: bool = True,
        arrival_check_hz: float = 10.0,
        cmd_process_hz: float = 50.0,
        stop_publish_hz: float = 10.0,
    ):
        super().__init__("guidance_state_machine")

        # --- executor / spin ownership ---
        self._owns_executor = executor is None
        self.executor = executor if executor is not None else SingleThreadedExecutor()
        self.executor.add_node(self)

        self._spin_thread: Optional[threading.Thread] = None
        if autospin and self._owns_executor:
            self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self._spin_thread.start()

        # --- ROS I/O ---
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.cancel_cli = self.create_client(CancelGoal, CANCEL_NAV_TO_POSE_SRV)

        # --- FSM state ---
        self.state: State = State.LOCKED
        self.last_target_key: Optional[str] = None
        self.nav_node: Optional[NavigationClient] = None

        # --- listeners ---
        self._listeners: list[TransitionCallback] = []

        # --- command queue (thread-safe) ---
        self._cmd_q: "queue.SimpleQueue[_Cmd]" = queue.SimpleQueue()

        # --- cooldown (executor thread에서만 체크) ---
        self._last_call_t = {"activate": 0.0, "pause": 0.0, "resume": 0.0, "abort": 0.0, "start": 0.0}
        self._cooldown = {"activate": 0.20, "pause": 0.40, "resume": 0.40, "abort": 0.20, "start": 0.20}

        # --- non-blocking stop publish control ---
        self._stop_until: float = 0.0
        self._stop_msg = Twist()

        # --- start worker state ---
        self._start_worker_running = False
        self._start_target_inflight: Optional[str] = None

        # --- timers ---
        self._cmd_timer = self.create_timer(1.0 / max(cmd_process_hz, 1.0), self._process_cmds)
        self._arrival_timer = self.create_timer(1.0 / max(arrival_check_hz, 1.0), self._tick_arrival)
        self._stop_timer = self.create_timer(1.0 / max(stop_publish_hz, 1.0), self._tick_stop_publish)

        self.get_logger().info("FSM ready. Initial state=LOCKED")

    # ========= 외부 연동용 API (thread-safe) =========
    def add_transition_listener(self, cb: TransitionCallback) -> None:
        self._listeners.append(cb)

    def activate(self) -> bool:
        self._cmd_q.put(_Cmd("activate"))
        return True

    def start_navigation(self, target_key: str) -> bool:
        if not target_key:
            return False
        self._cmd_q.put(_Cmd("start", target_key))
        return True

    def pause(self) -> bool:
        self._cmd_q.put(_Cmd("pause"))
        return True

    def resume(self) -> bool:
        self._cmd_q.put(_Cmd("resume"))
        return True

    def abort(self) -> bool:
        self._cmd_q.put(_Cmd("abort"))
        return True

    def shutdown(self) -> None:
        # 외부에서 종료 시 호출
        self._cmd_q.put(_Cmd("_shutdown"))

    # ========= executor thread에서만 실행되는 내부 로직 =========
    def _cooldown_ok(self, key: str) -> bool:
        now = time.monotonic()
        cd = self._cooldown.get(key, 0.0)
        last = self._last_call_t.get(key, 0.0)
        if now - last < cd:
            return False
        self._last_call_t[key] = now
        return True

    def _set_state(self, new_state: State, reason: str) -> None:
        old = self.state
        self.state = new_state

        for cb in list(self._listeners):
            try:
                cb(old, new_state, reason)
            except Exception as e:
                self.get_logger().warning(f"transition listener error: {e}")

        self.get_logger().info(f"STATE {old.name} -> {new_state.name} ({reason})")

    def _process_cmds(self) -> None:
        # 한 tick에 너무 많이 처리하지 않도록 제한(폭주 방지)
        max_per_tick = 20
        count = 0

        while count < max_per_tick:
            try:
                cmd = self._cmd_q.get_nowait()
            except Exception:
                break

            count += 1
            name, arg = cmd.name, cmd.arg

            if name == "_shutdown":
                self._handle_shutdown()
                return

            if name == "activate":
                self._handle_activate()
            elif name == "start":
                self._handle_start(arg)
            elif name == "pause":
                self._handle_pause()
            elif name == "resume":
                self._handle_resume()
            elif name == "abort":
                self._handle_abort()
            elif name == "_start_done":
                # arg = "OK:<target>" or "FAIL:<target>"
                self._handle_start_done(arg)
            else:
                self.get_logger().warning(f"Unknown cmd: {name}")

    # ---- handlers ----
    def _handle_activate(self) -> None:
        if not self._cooldown_ok("activate"):
            return
        if self.state != State.LOCKED:
            return
        self._set_state(State.READY, "activate")

    def _handle_start(self, target_key: Optional[str]) -> None:
        if not target_key:
            return
        if not self._cooldown_ok("start"):
            return
        if self.state != State.READY:
            return
        if self._start_worker_running:
            # start 연속 호출 방지
            return

        self.last_target_key = target_key
        self._destroy_nav_node()

        # nav node 생성 + executor 등록은 executor thread에서
        self.nav_node = NavigationClient(target_key)
        self.executor.add_node(self.nav_node)

        # start_navigation()은 내부에서 wait_for_server(최대 5초)로 블로킹 가능
        # -> executor thread를 막지 않도록 워커 스레드로 수행하고 결과만 큐로 반환
        self._start_worker_running = True
        self._start_target_inflight = target_key

        threading.Thread(
            target=self._start_worker,
            args=(target_key,),
            daemon=True,
        ).start()

    def _start_worker(self, target_key: str) -> None:
        ok = False
        try:
            # NavigationClient 내부 로직(서비스 wait / action wait) 실행
            if self.nav_node is not None:
                ok = bool(self.nav_node.start_navigation())
        except Exception:
            ok = False

        # 결과는 큐로 다시 전달 (ROS 상태 변경은 executor thread에서만)
        self._cmd_q.put(_Cmd("_start_done", f"{'OK' if ok else 'FAIL'}:{target_key}"))

    def _handle_start_done(self, payload: Optional[str]) -> None:
        self._start_worker_running = False
        target = None
        ok = False

        if payload and ":" in payload:
            head, target = payload.split(":", 1)
            ok = (head == "OK")

        # inflight가 달라졌으면 무시(중간 abort 등)
        if self._start_target_inflight and target != self._start_target_inflight:
            return
        self._start_target_inflight = None

        if self.state != State.READY:
            # start 도중 다른 상태로 바뀌었으면 정리
            if not ok:
                self._destroy_nav_node()
            return

        if ok:
            self._set_state(State.NAV, f"start:{target}")
        else:
            self.get_logger().warning("start_navigation() failed; stay READY")
            self._destroy_nav_node()
            self._set_state(State.READY, "start_failed")

    def _handle_pause(self) -> None:
        if not self._cooldown_ok("pause"):
            return
        if self.state != State.NAV:
            return

        self._set_state(State.PAUSED, "pause")
        self._request_stop(duration_sec=1.0)
        self._cancel_all_nav_goals_async()

    def _handle_resume(self) -> None:
        if not self._cooldown_ok("resume"):
            return
        if self.state != State.PAUSED:
            return
        if not self.last_target_key:
            self._set_state(State.LOCKED, "resume_no_target")
            return

        # READY를 거쳐 start 처리(명령 큐로 넣으면 동일 스레드에서 순서대로 실행됨)
        target = self.last_target_key
        self._set_state(State.READY, "resume_prepare")
        self._cmd_q.put(_Cmd("start", target))

    def _handle_abort(self) -> None:
        if not self._cooldown_ok("abort"):
            return
        if self.state == State.LOCKED:
            return

        self.last_target_key = None
        self._set_state(State.LOCKED, "abort")
        self._request_stop(duration_sec=1.0)
        self._cancel_all_nav_goals_async()
        self._destroy_nav_node()

    def _handle_shutdown(self) -> None:
        # 안전 종료
        try:
            self._request_stop(duration_sec=1.0)
            self._cancel_all_nav_goals_async()
            self._destroy_nav_node()
        except Exception:
            pass
        try:
            self.destroy_timer(self._cmd_timer)
            self.destroy_timer(self._arrival_timer)
            self.destroy_timer(self._stop_timer)
        except Exception:
            pass

    # ---- arrival & stop ----
    def _tick_arrival(self) -> None:
        if self.state != State.NAV or self.nav_node is None:
            return
        if getattr(self.nav_node, "is_arrived", False):
            self._destroy_nav_node()
            self.last_target_key = None
            self._set_state(State.LOCKED, "arrived")

    def _request_stop(self, duration_sec: float) -> None:
        until = time.time() + max(duration_sec, 0.0)
        self._stop_until = max(self._stop_until, until)

    def _tick_stop_publish(self) -> None:
        if time.time() < self._stop_until:
            self.cmd_pub.publish(self._stop_msg)

    # ---- cancel & cleanup (non-blocking) ----
    def _cancel_all_nav_goals_async(self) -> None:
        if not self.cancel_cli.service_is_ready():
            # 서비스 없으면 그냥 넘어감(로그만)
            self.get_logger().warning(f"Cancel service not ready: {CANCEL_NAV_TO_POSE_SRV}")
            return

        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        req.goal_info.goal_id = UUID(uuid=[0] * 16)         # match-all
        req.goal_info.stamp = BuiltinTime(sec=0, nanosec=0) # match-all

        fut = self.cancel_cli.call_async(req)

        def _done(_f):
            try:
                res = _f.result()
                if res and res.return_code != 0:
                    self.get_logger().warning(f"CancelGoal return_code={res.return_code}")
            except Exception as e:
                self.get_logger().warning(f"CancelGoal error: {e}")

        fut.add_done_callback(_done)

    def _destroy_nav_node(self) -> None:
        if not self.nav_node:
            return
        try:
            self.nav_node.cleanup()
        except Exception:
            pass
        try:
            self.executor.remove_node(self.nav_node)
        except Exception:
            pass
        try:
            self.nav_node.destroy_node()
        except Exception:
            pass
        self.nav_node = None


def main():
    rclpy.init()
    sm = GuidanceStateMachine(autospin=True)

    def _print(old: State, new: State, reason: str):
        print(f"[FSM] {old.name} -> {new.name} ({reason})")

    sm.add_transition_listener(_print)

    try:
        print("FSM running. (No input in this module)\n")
        while rclpy.ok():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        sm.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

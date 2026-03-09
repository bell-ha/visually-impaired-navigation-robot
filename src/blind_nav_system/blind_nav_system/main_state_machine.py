
#!/usr/bin/env python3
from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional

import rclpy
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID

from navigation_client import NavigationClient

CMD_VEL_TOPIC = "/stretch/cmd_vel"
CANCEL_NAV_TO_POSE_SRV = "/navigate_to_pose/_action/cancel_goal"


class State(Enum):
    LOCKED = auto()
    READY = auto()
    NAV = auto()
    PAUSED = auto()


TransitionCallback = Callable[[State, State, str], None]


@dataclass(frozen=True)
class _Cmd:
    name: str
    arg: Optional[str] = None


class GuidanceStateMachine(Node):
    def __init__(self, executor: Optional[SingleThreadedExecutor] = None, autospin: bool = True):
        super().__init__("guidance_state_machine")
        self._owns_executor = executor is None
        self._executor = executor if executor is not None else SingleThreadedExecutor()
        self._executor.add_node(self)

        self._spin_thread: Optional[threading.Thread] = None
        if autospin and self._owns_executor:
            self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._spin_thread.start()

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.cancel_cli = self.create_client(CancelGoal, CANCEL_NAV_TO_POSE_SRV)

        self.state: State = State.LOCKED
        self.last_target_key: Optional[str] = None
        self.nav_node: Optional[NavigationClient] = None
        self._listeners: list[TransitionCallback] = []
        self._cmd_q: "queue.SimpleQueue[_Cmd]" = queue.SimpleQueue()
        self._stop_until = 0.0
        self._stop_msg = Twist()
        self._start_worker_running = False
        self._start_target_inflight: Optional[str] = None

        self._cmd_timer = self.create_timer(0.02, self._process_cmds)
        self._arrival_timer = self.create_timer(0.1, self._tick_arrival)
        self._stop_timer = self.create_timer(0.1, self._tick_stop_publish)
        self.get_logger().info("FSM ready. Initial state=LOCKED")

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

    def force_ready(self, reason: str = "force_ready") -> bool:
        self._cmd_q.put(_Cmd("force_ready", reason))
        return True

    def shutdown(self) -> None:
        self._cmd_q.put(_Cmd("_shutdown"))

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
        for _ in range(20):
            try:
                cmd = self._cmd_q.get_nowait()
            except Exception:
                return
            name, arg = cmd.name, cmd.arg
            if name == "activate":
                if self.state == State.LOCKED:
                    self._set_state(State.READY, "activate")
            elif name == "start":
                self._handle_start(arg)
            elif name == "pause":
                if self.state == State.NAV:
                    self._set_state(State.PAUSED, "pause")
                    self._request_stop(1.0)
                    self._cancel_all_nav_goals_async()
            elif name == "resume":
                if self.state == State.PAUSED and self.last_target_key:
                    self._set_state(State.READY, "resume_prepare")
                    self._cmd_q.put(_Cmd("start", self.last_target_key))
            elif name == "abort":
                if self.state != State.LOCKED:
                    self.last_target_key = None
                    self._set_state(State.LOCKED, "abort")
                    self._request_stop(1.0)
                    self._cancel_all_nav_goals_async()
                    self._destroy_nav_node()
            elif name == "force_ready":
                if self.state == State.PAUSED:
                    self._cancel_all_nav_goals_async()
                    self._destroy_nav_node()
                    self._set_state(State.READY, arg or "force_ready")
            elif name == "_start_done":
                self._handle_start_done(arg)
            elif name == "_shutdown":
                self._handle_shutdown()
                return

    def _handle_start(self, target_key: Optional[str]) -> None:
        if not target_key:
            return
        if self.state != State.READY or self._start_worker_running:
            return
        self.last_target_key = target_key
        self._destroy_nav_node()
        self.nav_node = NavigationClient(target_key)
        self._executor.add_node(self.nav_node)
        self._start_worker_running = True
        self._start_target_inflight = target_key
        threading.Thread(target=self._start_worker, args=(target_key,), daemon=True).start()

    def _start_worker(self, target_key: str) -> None:
        ok = False
        try:
            if self.nav_node is not None:
                ok = bool(self.nav_node.start_navigation())
        except Exception as e:
            self.get_logger().warning(f"start worker error: {e}")
        self._cmd_q.put(_Cmd("_start_done", f"{'OK' if ok else 'FAIL'}:{target_key}"))

    def _handle_start_done(self, payload: Optional[str]) -> None:
        self._start_worker_running = False
        if not payload or ":" not in payload:
            return
        head, target = payload.split(":", 1)
        ok = head == "OK"
        if self._start_target_inflight and target != self._start_target_inflight:
            return
        self._start_target_inflight = None
        if self.state != State.READY:
            if not ok:
                self._destroy_nav_node()
            return
        if ok:
            self._set_state(State.NAV, f"start:{target}")
        else:
            self._destroy_nav_node()
            self._set_state(State.READY, "start_failed")

    def _tick_arrival(self) -> None:
        if self.state == State.NAV and self.nav_node is not None and getattr(self.nav_node, "is_arrived", False):
            self._destroy_nav_node()
            self.last_target_key = None
            self._set_state(State.LOCKED, "arrived")

    def _request_stop(self, duration_sec: float) -> None:
        self._stop_until = max(self._stop_until, time.time() + max(duration_sec, 0.0))

    def _tick_stop_publish(self) -> None:
        if time.time() < self._stop_until:
            self.cmd_pub.publish(self._stop_msg)

    def _cancel_all_nav_goals_async(self) -> None:
        if not self.cancel_cli.service_is_ready():
            return
        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        req.goal_info.goal_id = UUID(uuid=[0] * 16)
        req.goal_info.stamp = BuiltinTime(sec=0, nanosec=0)
        fut = self.cancel_cli.call_async(req)
        fut.add_done_callback(lambda _f: None)

    def _destroy_nav_node(self) -> None:
        if not self.nav_node:
            return
        try:
            self.nav_node.cleanup()
        except Exception:
            pass
        try:
            self._executor.remove_node(self.nav_node)
        except Exception:
            pass
        try:
            self.nav_node.destroy_node()
        except Exception:
            pass
        self.nav_node = None

    def _handle_shutdown(self) -> None:
        try:
            self._request_stop(1.0)
            self._cancel_all_nav_goals_async()
            self._destroy_nav_node()
        except Exception:
            pass
        try:
            self._executor.remove_node(self)
        except Exception:
            pass
        if self._owns_executor:
            try:
                self._executor.shutdown()
            except Exception:
                pass


def main():
    rclpy.init()
    sm = GuidanceStateMachine(autospin=True)
    try:
        while rclpy.ok():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        sm.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

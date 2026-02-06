#!/usr/bin/env python3
"""
main_state_machine.py

- 시작: LOCKED (아무것도 안 함)
- 'o' (TRIG 버튼 or 키보드 o) -> READY
- READY에서 'a'/'b' -> point_a / point_b로 이동(NAV)
- NAV에서 'p' (PULL or 키보드 p) -> PAUSED (cancel_goal + 0속도)
- PAUSED에서 'p' -> RESUME (마지막 목표를 다시 goal 전송)
- NAV/PAUSED에서 'o' -> ABORT (cancel + 0속도 + 목표삭제) 후 LOCKED로 복귀
- 도착하면 자동 LOCKED 복귀

추가(안정화):
- 상태전이/네비 제어는 단일 락(self._sm_lock)으로 보호 (세마포어/뮤텍스처럼)
- 빠른 연타/연속 PULL로 인한 토글 꼬임 방지: 키별 쿨다운 적용 (특히 'p'는 넉넉히)
- 하드웨어 큐는 한 loop tick에 1개만 처리 (폭주 방지)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

import threading
import time
import sys
import select
import termios
import tty
import queue
from enum import Enum, auto

from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from unique_identifier_msgs.msg import UUID
from builtin_interfaces.msg import Time as BuiltinTime

from navigation_client import NavigationClient

# ====== input_bridge import (설치/실행 위치에 따라 둘 중 하나가 맞을 수 있음) ======
try:
    from blind_nav_system.hardware.input_bridge import HardwareKeyBridge
except Exception:
    from hardware.input_bridge import HardwareKeyBridge  # fallback


# ====== 환경에 맞게 고정 ======
CMD_VEL_TOPIC = "/stretch/cmd_vel"
CANCEL_NAV_TO_POSE_SRV = "/navigate_to_pose/_action/cancel_goal"


class State(Enum):
    LOCKED = auto()
    READY = auto()
    NAV = auto()
    PAUSED = auto()


class KeyReader:
    """Enter 없이 한 글자 입력 받기(리눅스 터미널)."""
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = None

    def __enter__(self):
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self, timeout=0.05) -> str:
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return ""


class MainManager(Node):
    def __init__(self, executor: SingleThreadedExecutor, enable_keyboard=True):
        super().__init__("main_manager")

        self.executor = executor
        self.executor.add_node(self)

        self.state: State = State.LOCKED
        self.last_target: str | None = None  # 'a' or 'b'
        self.nav_node: NavigationClient | None = None

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.cancel_cli = self.create_client(CancelGoal, CANCEL_NAV_TO_POSE_SRV)

        # 입력 큐(하드웨어/키보드 모두 여기로)
        self.key_q: queue.Queue[str] = queue.Queue()

        # ====== 안정화(세마포어/뮤텍스 + 쿨다운) ======
        self._sm_lock = threading.Lock()
        self._last_key_t = {'o': 0.0, 'p': 0.0, 'a': 0.0, 'b': 0.0}
        # p는 "당김" 연타에 민감하니 넉넉히(원하면 0.4~1.0 사이에서 조절)
        self._cooldown = {'o': 0.25, 'p': 0.70, 'a': 0.15, 'b': 0.15}

        # 하드웨어 브릿지: TRIG -> 'o', PULL -> 'p'
        self.hw = None
        try:
            self.hw = HardwareKeyBridge(
                on_o=lambda: self.key_q.put("o"),
                on_p=lambda: self.key_q.put("p"),
            )
            self.hw.start()
            self.get_logger().info("HardwareKeyBridge started. (TRIG->o, PULL->p)")
        except Exception as e:
            self.get_logger().warning(f"HardwareKeyBridge start failed: {e}")

        # 상태별 키 핸들러 매핑
        self.handlers = {
            (State.LOCKED, 'o'): self.to_ready,

            (State.READY, 'a'): lambda: self.start_to('a'),
            (State.READY, 'b'): lambda: self.start_to('b'),

            (State.NAV, 'p'): self.pause,
            (State.NAV, 'o'): self.abort_to_locked,

            (State.PAUSED, 'p'): self.resume,
            (State.PAUSED, 'o'): self.abort_to_locked,
        }

        # ROS spin 백그라운드
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        # 키보드 입력 사용 여부(디버그용)
        self.enable_keyboard = enable_keyboard and sys.stdin.isatty()

    # ---------------- 상태/키 처리 ----------------
    def on_key(self, key: str):
        """키 입력(하드웨어/키보드)을 단일 락으로 직렬화 + 쿨다운으로 연타 방지."""
        key = key.lower()
        if key not in ('o', 'p', 'a', 'b'):
            return

        now = time.monotonic()
        cd = self._cooldown.get(key, 0.0)

        # 1차 쿨다운 컷(락 밖)
        if now - self._last_key_t.get(key, 0.0) < cd:
            return

        with self._sm_lock:
            now = time.monotonic()
            # 2차 쿨다운 컷(락 안)
            if now - self._last_key_t.get(key, 0.0) < cd:
                return
            self._last_key_t[key] = now

            fn = self.handlers.get((self.state, key))
            if fn:
                fn()

    def to_ready(self):
        self.state = State.READY
        print("\n[READY] a/b 입력 대기 (p는 이동중/일시정지에서만 사용).")

    # ---------------- 도착 처리 ----------------
    def on_arrived(self):
        self._destroy_nav_node()
        self.last_target = None
        self.state = State.LOCKED
        print("\n[LOCKED] 도착했습니다. 'o'(TRIG)로 다시 명령을 받습니다.")

    # ---------------- 네비게이션 제어 ----------------
    def start_to(self, target: str):
        """target: 'a' or 'b' -> point_a / point_b"""
        self.last_target = target

        self._destroy_nav_node()

        key = f"point_{target}"  # point_a / point_b
        self.nav_node = NavigationClient(key)
        self.executor.add_node(self.nav_node)

        ok = bool(self.nav_node.start_navigation())
        if ok:
            self.state = State.NAV
            print(f"\n[NAV] {key} 이동 시작! (p=PULL 일시정지, o=TRIG 중단)")
        else:
            print("\n[ERROR] start_navigation() 실패. READY로 돌아갑니다.")
            self._destroy_nav_node()
            self.state = State.READY

    def pause(self):
        """일시정지: goal 취소 + 0속도. last_target 유지"""
        self._cancel_all_nav_goals()
        self._publish_zero(duration_sec=1.0)
        self.state = State.PAUSED
        print("\n[PAUSED] p=PULL 재개, o=TRIG 중단(LOCKED 복귀)")

    def resume(self):
        """재개: last_target을 다시 goal 전송"""
        if not self.last_target:
            print("\n[WARN] 재개할 목표가 없습니다. 'o'(TRIG)로 READY 진입하세요.")
            self.state = State.LOCKED
            return
        self.start_to(self.last_target)

    def abort_to_locked(self):
        """중단: goal 취소+정지+목표삭제 후 LOCKED로"""
        self._cancel_all_nav_goals()
        self._publish_zero(duration_sec=1.0)

        self.last_target = None
        self._destroy_nav_node()

        self.state = State.LOCKED
        print("\n[LOCKED] 중단 완료. 'o'(TRIG)로 다시 명령을 받습니다.")

    # ---------------- 저수준 유틸 ----------------
    def _publish_zero(self, duration_sec: float = 1.0, rate_hz: float = 10.0):
        msg = Twist()
        period = 1.0 / rate_hz
        end_t = time.time() + duration_sec
        while rclpy.ok() and time.time() < end_t:
            self.cmd_pub.publish(msg)
            time.sleep(period)

    def _cancel_all_nav_goals(self, timeout_sec: float = 1.0) -> bool:
        """RViz에서 보낸 goal까지 포함해서 NavigateToPose goal 전부 취소"""
        if not self.cancel_cli.service_is_ready():
            if not self.cancel_cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().warning(f"Cancel service not ready: {CANCEL_NAV_TO_POSE_SRV}")
                return False

        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        req.goal_info.goal_id = UUID(uuid=[0] * 16)            # match-all
        req.goal_info.stamp = BuiltinTime(sec=0, nanosec=0)    # match-all

        fut = self.cancel_cli.call_async(req)
        deadline = time.time() + timeout_sec
        while rclpy.ok() and (not fut.done()) and time.time() < deadline:
            time.sleep(0.01)

        if not fut.done():
            self.get_logger().warning("CancelGoal request timed out.")
            return False

        res = fut.result()
        return (res.return_code == 0)

    def _destroy_nav_node(self):
        if self.nav_node:
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

    def shutdown(self):
        self._destroy_nav_node()
        try:
            if self.hw:
                self.hw.stop()
        except Exception:
            pass


def main():
    rclpy.init()
    exec_obj = SingleThreadedExecutor()
    manager = MainManager(exec_obj, enable_keyboard=True)

    print("\n=== INPUT MAP ===")
    print("TRIG(버튼) or 키보드 o : LOCKED->READY, (NAV/PAUSED)->중단 후 LOCKED")
    print("키보드 a/b           : READY에서 point_a / point_b 이동")
    print("PULL(당김) or 키보드 p: NAV<->PAUSED 토글(일시정지/재개)")
    print("q                    : 종료(키보드 사용 시)")
    print("=================")
    print("\n[LOCKED] 시작 상태. TRIG(또는 o)를 누르면 READY.")

    use_keyboard = manager.enable_keyboard
    kr = KeyReader() if use_keyboard else None

    try:
        if use_keyboard:
            kr.__enter__()

        while rclpy.ok():
            # 1) 하드웨어 키 처리(큐) - 한 tick에 1개만 처리(폭주 방지)
            try:
                hk = manager.key_q.get_nowait()
            except queue.Empty:
                hk = None
            if hk:
                manager.on_key(hk)

            # 2) 키보드 키 처리(있으면)
            if use_keyboard:
                k = kr.read_key(timeout=0.01)
                if k:
                    k = k.lower()
                    if k == 'q':
                        break
                    manager.on_key(k)

            # 3) 도착하면 LOCKED로 자동 복귀
            if manager.state == State.NAV and manager.nav_node:
                if getattr(manager.nav_node, "is_arrived", False):
                    # 도착 처리도 상태락으로 보호(간헐적 레이스 방지)
                    with manager._sm_lock:
                        if manager.state == State.NAV and manager.nav_node and getattr(manager.nav_node, "is_arrived", False):
                            manager.on_arrived()

            time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            if use_keyboard and kr:
                kr.__exit__(None, None, None)
        except Exception:
            pass

        manager.shutdown()
        rclpy.shutdown()
        print("\n[EXIT]")


if __name__ == "__main__":
    main()

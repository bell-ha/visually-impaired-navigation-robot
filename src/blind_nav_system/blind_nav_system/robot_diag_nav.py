#!/usr/bin/env python3
"""robot_diag_nav.py — nav2 주행 "블랙박스" 독립 프로세스 로거.

왜 독립 프로세스인가 (2026-08-10 규명):
  robot_diag.attach()는 대시보드/엘베앱 노드에 얹혀서 그 프로세스의 단 하나뿐인
  rclpy.spin 스레드에 의존한다. 런치 시 FastDDS 섬 현상(탐색은 되나 데이터 채널만
  잠김)이 호스트 participant를 마비시키면 얹혀있는 blackbox도 같이 침묵한다
  (실측: system 로거가 16:27:55 이후 49분간 완전 침묵 → 리턴 주행 멈춤을 놓침).
  독립 프로세스는 "자체 participant + 자체 spin"을 가져 그 섬을 피한다
  (같은 시각 별도 recorder 프로세스는 nav2 드롭 838개를 정상 수집).
  → nav2 멈춤/드롭/복구를 확실히 파일로 남긴다.

기록 위치: ~/.ros/robot_diag/nav/nav_<YYYYMMDD_HHMMSS>_pid<PID>.log  (7일 자동 보관)
실행:     대시보드(main.py)가 start_subprocesses에서 자동 spawn → 수동 실행 불필요.
          (interface.py / vision_assistant.py 와 똑같이 대시보드가 켜줌)
"""
import os
import sys
import time

# 기존 blackbox의 포맷·7일 보관정책·nav 필터를 그대로 재사용 (import 자체는 부작용 없음)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from robot_diag import DiagLogger, _NAV_NODES, _NAV_KW, _LVL_NAME
except Exception:
    DiagLogger = None


def main():
    if DiagLogger is None:
        print("robot_diag_nav: robot_diag 임포트 실패 — 종료", flush=True)
        return

    logger = DiagLogger("nav")            # ~/.ros/robot_diag/nav/ (생성 시 7일 지난 로그 자동 정리)
    try:
        logger.boot_snapshot({"role": "nav-blackbox (독립 프로세스 — 호스트 spin과 무관)"})
    except Exception:
        pass

    try:
        import rclpy
        from rclpy.node import Node
        from rcl_interfaces.msg import Log
        try:   # 종료 신호(SIGTERM/SIGINT) 시 rclpy가 던지는 정상 종료 예외
            from rclpy.executors import ExternalShutdownException
        except Exception:
            ExternalShutdownException = KeyboardInterrupt
    except Exception as e:
        logger.log("WARN", f"rclpy/rcl_interfaces 임포트 실패 — ROS 캡처 생략: {e!r}")
        return

    class NavBlackbox(Node):
        def __init__(self):
            super().__init__("robot_diag_nav")
            self._last = None              # (name, msg) 직전 — 연속 중복 접기
            self._dups = 0
            # /rosout 은 신뢰성 있게 받도록 큐 깊이 크게
            self.create_subscription(Log, "/rosout", self._on_rosout, 200)
            logger.log("BOOT", "nav2 /rosout 캡처 시작 (독립 프로세스, WARN+ 및 nav 노드/키워드)")

        def _on_rosout(self, m):
            try:
                name = m.name or ""
                msg = m.msg or ""
                keep = (m.level >= 30) \
                    or any(k in name for k in _NAV_NODES) \
                    or any(k in msg.lower() for k in _NAV_KW)
                if not keep:
                    return
                key = (name, msg)
                if key == self._last:                 # 연속 동일 메시지 접기
                    self._dups += 1
                    return
                if self._dups:
                    logger.log("NAV2", f"(…직전 메시지 {self._dups}회 반복)")
                    self._dups = 0
                self._last = key
                lvl = _LVL_NAME.get(m.level, str(m.level))
                logger.log("NAV2", f"[{lvl}][{name}] {msg}")
            except Exception:
                pass   # 메시지 하나 실패해도 프로세스는 계속 (blackbox는 안 죽는 게 목적)

    rclpy.init()
    node = NavBlackbox()

    # spin 이 어떤 예외로 죽어도 잠깐 쉬고 되살려 계속 기록 (침묵하지 않는 게 핵심 목적)
    while rclpy.ok():
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            break
        except Exception as e:
            try:
                logger.log("WARN", f"spin 예외 → 0.5s 후 재개: {e!r}")
            except Exception:
                pass
            time.sleep(0.5)

    for fn in (node.destroy_node, rclpy.shutdown):
        try:
            fn()
        except Exception:
            pass
    try:
        logger.log("EXIT", "nav-blackbox 종료")
    except Exception:
        pass


if __name__ == "__main__":
    main()

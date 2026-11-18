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

기록 위치: ~/.ros/robot_diag/nav/nav_<YYYYMMDD_HHMMSS>_pid<PID>.log  (자동 삭제 없음 — retain_days 기본 0)
실행:     대시보드(main.py)가 start_subprocesses에서 자동 spawn → 수동 실행 불필요.
          (interface.py / vision_assistant.py 와 똑같이 대시보드가 켜줌)
"""
import os
import sys
import time

# 기존 blackbox의 포맷·보관정책(기본 삭제 안 함)·nav 필터를 그대로 재사용 (import 자체는 부작용 없음)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from robot_diag import DiagLogger, _NAV_NODES, _NAV_KW, _LVL_NAME
except Exception:
    DiagLogger = None


def main():
    if DiagLogger is None:
        print("robot_diag_nav: robot_diag 임포트 실패 — 종료", flush=True)
        return

    logger = DiagLogger("nav")            # ~/.ros/robot_diag/nav/ (자동 삭제 없음 — retain_days 기본 0)
    try:
        logger.boot_snapshot({"role": "nav-blackbox (독립 프로세스 — 호스트 spin과 무관)"})
    except Exception:
        pass

    try:
        import rclpy
        from rclpy.node import Node
        from rcl_interfaces.msg import Log
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
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
            # ── 바퀴 명령 vs 실제 (2026-11-18 추가) ──────────────────────────────
            # 🔴 왜 필요한가: 11/18 ② 문앞정렬이 60초간 안 움직였는데, **이 블랙박스가
            #   /rosout 만 받아서** "nav2 가 0 을 명령했나 / 명령은 나갔는데 바퀴가 안
            #   받았나"를 **영원히 못 갈랐다.** 그 둘은 처방이 정반대다. DWB 는 그때
            #   `No valid trajectories` 를 한 번도 안 냈으므로(= 궤적은 있었다) 로그만
            #   보면 멀쩡해 보인다. 명령값과 실측값을 나란히 적어야 그 구분이 생긴다.
            # 기록 정책 — 20Hz 를 그대로 적으면 로그가 못 쓰게 된다:
            #   · 둘 중 하나라도 움직이는 동안 1초에 한 줄
            #   · 멈춤/출발 전이는 즉시 한 줄
            #   · **명령은 있는데 실측이 0** 이 1.5초 넘게 이어지면 🚨 한 줄 (회당 1회)
            self._cmd = (0.0, 0.0, 0.0)    # (vx, wz, 수신시각)
            self._od  = (0.0, 0.0, 0.0)
            self._emit_ts  = 0.0
            self._was_live = False
            self._stuck_since = None
            self._stuck_said  = False
            self.create_subscription(Twist, "/stretch/cmd_vel", self._on_cmd, 20)
            self.create_subscription(Odometry, "/odom", self._on_odom, 20)
            self.create_timer(0.5, self._tick)
            logger.log("BOOT", "nav2 /rosout + /stretch/cmd_vel + /odom 캡처 시작 "
                               "(독립 프로세스, WARN+ 및 nav 노드/키워드)")

        _EPS_V, _EPS_W = 0.005, 0.010      # m/s, rad/s — 이 아래는 '0' 으로 본다

        def _on_cmd(self, m):
            try:
                self._cmd = (float(m.linear.x), float(m.angular.z), time.time())
            except Exception:
                pass

        def _on_odom(self, m):
            try:
                self._od = (float(m.twist.twist.linear.x),
                            float(m.twist.twist.angular.z), time.time())
            except Exception:
                pass

        def _tick(self):
            """명령·실측을 한 줄로 묶어 남긴다. 어떤 실패도 삼킨다(블랙박스는 안 죽는다)."""
            try:
                now = time.time()
                cv, cw, cts = self._cmd
                ov, ow, ots = self._od
                fresh = (now - cts) < 0.5          # 0.5초 넘게 끊기면 명령 없음으로 본다
                cv, cw = (cv, cw) if fresh else (0.0, 0.0)
                cmd_on = abs(cv) > self._EPS_V or abs(cw) > self._EPS_W
                od_on  = abs(ov) > self._EPS_V or abs(ow) > self._EPS_W
                live   = cmd_on or od_on

                # 명령은 있는데 바퀴가 안 도는 상태 — 오늘 못 갈랐던 바로 그 경우
                if cmd_on and not od_on:
                    if self._stuck_since is None:
                        self._stuck_since = now
                    elif not self._stuck_said and now - self._stuck_since > 1.5:
                        self._stuck_said = True
                        logger.log("WHEEL", f"🚨 명령은 나가는데 바퀴가 안 돈다 — "
                                            f"명령 vx {cv:+.3f} wz {cw:+.3f} / "
                                            f"실측 vx {ov:+.3f} wz {ow:+.3f} "
                                            f"({now - self._stuck_since:.1f}초째)")
                else:
                    self._stuck_since = None
                    self._stuck_said  = False

                if live != self._was_live:         # 출발·정지 전이는 즉시
                    self._was_live = live
                    logger.log("WHEEL", f"{'▶ 출발' if live else '■ 정지'} — "
                                        f"명령 vx {cv:+.3f} wz {cw:+.3f} / "
                                        f"실측 vx {ov:+.3f} wz {ow:+.3f}")
                    self._emit_ts = now
                    return
                if live and now - self._emit_ts >= 1.0:
                    self._emit_ts = now
                    logger.log("WHEEL", f"명령 vx {cv:+.3f} wz {cw:+.3f} / "
                                        f"실측 vx {ov:+.3f} wz {ow:+.3f}"
                                        + ("" if fresh else "  (명령 끊김)"))
            except Exception:
                pass

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

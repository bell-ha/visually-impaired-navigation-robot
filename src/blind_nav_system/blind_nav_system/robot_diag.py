#!/usr/bin/env python3
"""robot_diag.py — 간헐 버그 "블랙박스(비행기록장치)" 로거.

system main / elevator main 공용. 목적: 실시간으로 못 보는 증상을
나중에 열어볼 수 있도록 타임스탬프 파일로 남긴다.

기록 대상 (3가지 가설 대응):
  ① 카메라 프레임 수신 여부   → 카메라 토픽별 프레시니스(살아있음/끊김) 이벤트
  ② cmd_vel 경합 / 제어권 오판 → /stretch/cmd_vel 퍼블리셔 수, authority 상태
  ③ FastDDS 잔재 / 유령 노드   → /dev/shm 잔재 개수, 노드 목록 중복

설계 원칙:
  - rclpy/메시지 타입이 없어도 import 자체는 안전 (파일 로깅만이라도 동작).
  - 기존 코드 로직은 건드리지 않고 node에 "진단 전용" 구독/타이머만 덧붙인다.
  - 카메라 워치는 sensor_data(best-effort) QoS로 구독 → reliable/best-effort
    퍼블리셔 어느 쪽이든 매칭. (즉 '버스에 프레임이 흐르는지'를 본다)

로그 위치: ~/.ros/robot_diag/<tag>/<tag>_<YYYYMMDD_HHMMSS>_pid<PID>.log
          (프로세스별 폴더 분리 + 7일 지난 로그 자동 정리)
"""

import os
import glob
import time
import signal
import threading
import datetime

# 진단용 메시지 타입 (없으면 카메라 워치만 생략, 나머지는 정상 동작)
try:
    from sensor_msgs.msg import Image
    from rclpy.qos import qos_profile_sensor_data
    _RCLPY_OK = True
except Exception:
    Image = None
    qos_profile_sensor_data = None
    _RCLPY_OK = False

# /rosout 필터 상수 — 독립 프로세스 robot_diag_nav.py 가 import 해서 사용한다.
# (대시보드 내장 rosout 캡처는 섬 현상으로 죽어 robot_diag_nav.py 로 이관됨 → 여기 상수는 삭제 금지)
_NAV_NODES = ("controller_server", "behavior_server", "bt_navigator",
              "planner_server", "smoother_server", "velocity_smoother",
              "collision_monitor", "local_costmap", "global_costmap",
              "controller", "planner", "amcl")
_NAV_KW = ("progress", "recovery", "backup", "spin", "no valid", "collision",
           "abort", "fail", "invalid", "oscillat", "stuck", "clearing",
           "costmap", "unable", "blocked", "no path", "goal")
_LVL_NAME = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

LOG_DIR = os.path.expanduser("~/.ros/robot_diag")
_SHM_GLOBS = ["/dev/shm/fastrtps_*",
              "/dev/shm/fast_datasharing*",
              "/dev/shm/sem.fastrtps*"]


def shm_count():
    """FastDDS 공유메모리 잔재 개수."""
    n = 0
    for g in _SHM_GLOBS:
        try:
            n += len(glob.glob(g))
        except Exception:
            pass
    return n


def _prune_old(directory, days):
    """directory 안에서 days일 지난 .log 파일 자동 삭제 (보관 정책)."""
    if not days or days <= 0:
        return
    cutoff = time.time() - days * 86400
    try:
        for name in os.listdir(directory):
            if not name.endswith(".log"):
                continue
            p = os.path.join(directory, name)
            try:
                if os.path.getmtime(p) < cutoff:
                    os.remove(p)
            except OSError:
                pass
    except Exception:
        pass


class DiagLogger:
    """파일 + 콘솔에 타임스탬프 로그를 남기는 스레드-세이프 로거.

    로그는 프로세스별 하위 폴더로 분리 저장:
        ~/.ros/robot_diag/system/system_<stamp>_pid<PID>.log
        ~/.ros/robot_diag/elevator/elevator_<stamp>_pid<PID>.log
    실행 시마다 retain_days(기본 7일)보다 오래된 파일은 자동 정리된다.
    """

    def __init__(self, tag, retain_days=7):
        self.tag = tag
        self.dir = os.path.join(LOG_DIR, tag)   # ← 프로세스별 폴더 분리
        try:
            os.makedirs(self.dir, exist_ok=True)
        except Exception:
            pass
        _prune_old(self.dir, retain_days)        # ← 7일 지난 로그 자동 삭제
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(self.dir, f"{tag}_{stamp}_pid{os.getpid()}.log")
        self._lock = threading.Lock()
        try:
            self._fp = open(self.path, "a", buffering=1)  # 줄 단위 버퍼링
        except Exception as e:
            self._fp = None
            # 디스크풀·권한거부로 파일이 안 열려도 본체는 살아있다 → 조용히 넘어가면
            # 로그가 안 쌓이는 걸 아무도 모른다. 반드시 알린다.
            try:
                print(f"[경고] 진단 로그 파일 열기 실패 {self.path}: {e}", flush=True)
            except Exception:
                pass
        self.ok = bool(self._fp)     # 호출측이 계측 활성 여부를 확인할 수 있게 노출
        self.log("BOOT", f"진단 로거 시작 pid={os.getpid()} → {self.path}")

    def log(self, level, msg, echo=True):
        # echo 기본 True → 기존 호출부·다른 프로세스 동작 변화 없음.
        # 고빈도 호출부(대시보드 _log 파이어호스)만 echo=False로 터미널 플러드 차단.
        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        line = f"{ts} [{self.tag}/{level}] {msg}"
        warn = None
        with self._lock:
            if self._fp:
                try:
                    self._fp.write(line + "\n")
                except Exception as e:
                    # 디스크풀 등으로 기록이 멈춘 것 → 최초 1회만 알린다(플러드 방지).
                    # print를 락 안에서 하면 락 안 I/O가 되어 호출자를 물고 늘어진다.
                    # 락 안에선 메시지만 만들고, 실제 출력은 락을 놓은 뒤.
                    if self.ok:
                        self.ok = False
                        warn = f"[경고] 진단 로그 쓰기 실패 — 계측 중단 {self.path}: {e}"
        if warn:
            try:
                print(warn, flush=True)
            except Exception:
                pass
        if echo:
            try:
                print(line, flush=True)
            except Exception:
                pass

    def boot_snapshot(self, extra=None):
        """부팅 순간의 DDS 환경/잔재 스냅샷."""
        self.log("SNAP", f"부팅 스냅샷 ─ /dev/shm fastrtps 잔재={shm_count()}개")
        prof = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
        self.log("SNAP",
                 f"RMW={os.environ.get('RMW_IMPLEMENTATION', '(default)')} "
                 f"DOMAIN={os.environ.get('ROS_DOMAIN_ID', '0')} "
                 f"FASTRTPS_PROFILE={prof or '(none)'}")
        if prof:
            self.log("SNAP",
                     f"SHM비활성 프로파일 존재={os.path.exists(os.path.expanduser(prof))}")
        if extra:
            for k, v in extra.items():
                self.log("SNAP", f"{k}={v}")


def attach(node, logger, cameras=None, cmd_vel_topic=None,
           authority_getter=None, phase_getter=None,
           own_node_name=None, expected_nodes=None,
           hb_period=2.0, snap_period=10.0, dead_after=2.5):
    """node에 진단 전용 구독/타이머를 붙인다. 기존 로직은 건드리지 않는다.

    cameras          : {라벨: 토픽} — 프레임 수신 여부를 감시할 카메라들
    cmd_vel_topic    : 퍼블리셔 수를 감시할 토픽 (예: /stretch/cmd_vel)
    authority_getter : 현재 제어권을 반환하는 callable (있으면 로그에 표시)
    phase_getter     : 현재 단계를 반환하는 callable (있으면 로그에 표시)
    own_node_name    : 자기 노드 이름 (참가자 중복=유령 감지)
    expected_nodes   : 존재해야 하는 노드 이름 목록 (사라지면 이벤트)
    """
    cameras = cameras or {}
    st = {"last": {}, "alive": {}, "pubcount": None, "ghost": None,
          "last_snap": 0.0}
    now = time.time

    # 카메라 프레시니스 구독 (진단 전용)
    if Image is not None and qos_profile_sensor_data is not None:
        for label, topic in cameras.items():
            def _mk(lbl):
                def _cb(_msg):
                    st["last"][lbl] = now()
                return _cb
            try:
                node.create_subscription(Image, topic, _mk(label),
                                         qos_profile_sensor_data)
            except Exception as e:
                logger.log("WARN", f"카메라 워치 구독 실패 {label}({topic}): {e}")
    elif cameras:
        logger.log("WARN", "sensor_msgs/Image 불가 — 카메라 프레시니스 워치 생략")

    def _hb():
      try:
        t = now()
        # ① 카메라 alive/dead 전이 이벤트
        for label in cameras:
            last = st["last"].get(label)
            alive = (last is not None) and (t - last < dead_after)
            prev = st["alive"].get(label)
            if prev is None or prev != alive:
                if last is None:
                    logger.log("CAM", f"{label} 아직 프레임 없음 (구독은 됨)")
                elif alive:
                    logger.log("CAM", f"{label} ▶ 수신 시작 (age={t - last:.2f}s)")
                else:
                    logger.log("CAM", f"{label} ✖ 프레임 끊김 (age={t - last:.2f}s)")
                st["alive"][label] = alive
        # ② cmd_vel 퍼블리셔 수 전이
        if cmd_vel_topic:
            try:
                c = node.count_publishers(cmd_vel_topic)
            except Exception:
                c = None
            if c is not None and c != st["pubcount"]:
                auth = _safe_call(authority_getter)
                warn = " ⚠️경합가능(2개 이상)" if c > 1 else ""
                logger.log("CMDVEL",
                           f"{cmd_vel_topic} 퍼블리셔 수={c} (authority={auth}){warn}")
                st["pubcount"] = c
        # ③ 유령/중복 노드 전이
        if own_node_name:
            try:
                dup = node.get_node_names().count(own_node_name)
            except Exception:
                dup = None
            if dup is not None and dup != st["ghost"]:
                if dup > 1:
                    logger.log("NODE",
                               f"⚠️ '{own_node_name}' 참가자 {dup}개 (유령/중복 의심)")
                else:
                    logger.log("NODE", f"'{own_node_name}' 참가자 {dup}개 (정상)")
                st["ghost"] = dup
        # ④ 주기 스냅샷 (전체 상태 한 줄)
        if t - st["last_snap"] >= snap_period:
            st["last_snap"] = t
            cam_ages = []
            for label in cameras:
                last = st["last"].get(label)
                cam_ages.append(
                    f"{label}={'never' if last is None else f'{t - last:.1f}s'}")
            extra = ""
            if cmd_vel_topic:
                try:
                    extra = f" cmd_vel_pubs={node.count_publishers(cmd_vel_topic)}"
                except Exception:
                    pass
            try:
                nodes = node.get_node_names()
            except Exception:
                nodes = []
            missing = ""
            if expected_nodes:
                miss = [n for n in expected_nodes if n not in nodes]
                if miss:
                    missing = f" 없는노드={miss}"
            logger.log("HB",
                       f"cams[{' '.join(cam_ages)}] shm={shm_count()} "
                       f"authority={_safe_call(authority_getter)} "
                       f"phase={_safe_call(phase_getter)}{extra} "
                       f"nodes={len(nodes)}{missing}")
      except Exception as _e:
        # HB 콜백이 어떤 이유로든 예외를 내도 executor(spin)를 죽이면 안 됨
        try:
            logger.log("WARN", f"HB 콜백 예외(무시, 타이머 유지): {_e}")
        except Exception:
            pass

    try:
        node.create_timer(hb_period, _hb)
        logger.log("BOOT",
                   f"헬스 하트비트 {hb_period}s 주기 시작 (카메라 {len(cameras)}개 워치)")
    except Exception as e:
        logger.log("WARN", f"하트비트 타이머 생성 실패: {e}")
    return st


def _safe_call(fn):
    if fn is None:
        return "?"
    try:
        return fn()
    except Exception:
        return "?"


def install_signal_logging(logger, reraise=True):
    """SIGINT/SIGTERM 수신을 로그로 남긴다 → "어떻게 죽었나"를 기록.

    reraise=True  (elevator용): 신호를 KeyboardInterrupt로 흘려보내
        기존 `except KeyboardInterrupt`/`finally` 정리 경로를 타게 한다.
        → SIGTERM(VSCode 정지 등)으로 죽여도 destroy_node가 돌아 잔재가 준다.
    reraise=False (system용): 로그만 남기고 원래 종료 동작을 그대로 수행.
    """
    prev = {}

    def _mk(signum):
        prev[signum] = signal.getsignal(signum)

        def _handler(sn, frame):
            try:
                name = signal.Signals(sn).name
            except Exception:
                name = str(sn)
            logger.log("EXIT", f"신호 수신 {name} → 종료 (shm잔재={shm_count()})")
            if reraise:
                raise KeyboardInterrupt()
            # passthrough: 원래 동작 재현
            old = prev.get(sn)
            if callable(old):
                old(sn, frame)
            else:
                signal.signal(sn, old if old is not None else signal.SIG_DFL)
                os.kill(os.getpid(), sn)

        try:
            signal.signal(signum, _handler)
        except Exception as e:
            logger.log("WARN", f"신호 핸들러 설치 실패 {signum}: {e}")

    for s in (signal.SIGINT, signal.SIGTERM):
        _mk(s)
    logger.log("BOOT", f"종료 신호 로깅 설치 (reraise={reraise})")

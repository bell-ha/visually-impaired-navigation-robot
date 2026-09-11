#!/usr/bin/env python3
"""journey_log.py — 대시보드 자동 여정의 블랙박스(비행기록장치). 실행 1회 = JSONL 1파일.

■ 왜 (2026-09-11)
  사용자: "내가 자동여정을 수없이 진행할거고, 안되면 계속 수정하고 그럴거니까 그걸 위한
  작업을 진행해줘" → 목표는 **실패한 한 번의 실행에서 재실행 없이 원인까지 가는 것**이다.
  9/10 에 헤맨 네 건이 기준이다: 링 흡수 33초(단계 전환이 없어 화면만 "조준 중"), 클램프가
  서보를 막음, ros2 데몬 staleness 로 게이트가 '모른다'가 됨, 작업 트리를 실행판으로 착각.
  넷 중 셋은 "상태"가 아니라 "판정이 어떤 입력으로 어떤 결론을 냈나"가 없어서 헤맸다.
  그래서 상태(step·sample·elev)와 **판정(gate)** 을 같이 적는다.

■ 절대 제약 — 이 파일을 고칠 때 먼저 읽을 것
  1. 여정 스레드에 블로킹을 더하지 않는다. 공개 메서드(훅)는 dict 를 만들어 put_nowait 만 한다.
     직렬화·파일쓰기·/proc·git·md5 는 writer 스레드(또는 기동 시 백그라운드)에서 한다.
  2. 새 호출을 만들지 않는다. /status 도 따로 부르지 않는다 — 대시보드가 이미 부르는
     `_elev_status()` 의 응답을 버리지 않고 받아 적을 뿐이다.
     (/status 는 읽기 전용이 아니다: 엘베앱이 그 핸들러 안에서 문 열림 기준선·연속관측을 센다.
      부르는 횟수가 늘면 그 자체가 판정에 섞인다.)
  3. 기록 실패는 **여정에 대해서만** 무음이다. 공개 메서드는 전부 예외를 삼킨다.
     기록에 대해서는 보인다: written / dropped / write_errors 를 run_end 에 싣고, 첫 실패는
     대시보드 로그에 한 줄 남긴다. **run_end 가 없는 파일 = 대시보드가 여정 도중 죽었다.**

■ 저장 위치 — ~/.ros/journey/<run_id>_<dest>.jsonl
  (시작 자체가 거부된 요청은 ~/.ros/journey/_rejected.jsonl 에 한 줄씩 덧붙인다)
  robot_diag(~/.ros/robot_diag) **밖**에 둔다. robot_diag 는 7일 지난 로그를 스스로 지운다
  (robot_diag.py `_prune_old`, 2026-09-11 verifier 실측: 8/27~9/1 기록 이미 소실). 반복 실행
  기록은 "어느 단계가 제일 자주 깨지나"를 보려고 쌓는 것이라 지워지면 목적이 없다.

■ 레코드 — 한 줄 = JSON 하나
  공통 봉투 {v, run, seq, t(ISO ms), ts(epoch), mono, ev}
  seq 는 큐에 넣는 순간 매긴다 → 파일에서 seq 가 비면 그만큼 버려진(dropped) 것이다.
  ev 종류: run_start · step · gate · call · human · elev · sample · notify · proc · exception · run_end
  필드는 각 메서드 독스트링에 있다. 필드를 늘릴 때는 호출부 kwargs 만 늘리면 된다
  (고정 스키마 목록을 두지 않았다 — advisor 비평이 추가 필드를 계속 낼 수 있어서).
"""

import datetime
import functools
import hashlib
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
import traceback

SCHEMA_V = 1
JOURNEY_DIR = os.path.expanduser("~/.ros/journey")
REJECT_FILE = "_rejected.jsonl"
Q_MAX = 5000                 # 넘치면 버리고 센다 — 블로킹 put 금지
ELEV_HEARTBEAT_SEC = 5.0     # 변화가 없어도 이 간격으로는 한 번 적는다
STALE_SLACK_SEC = 1.0        # proc_identity 의 stale 판정 여유 — 그 함수 주석 참고

# elev 변화 감지 키 — 이 값들이 바뀔 때만 적고, 나머지는 하트비트로 본다.
# lift·arm_ext·ex·ey 는 서보 중 매 폴링 바뀌어서 넣지 않았다. "33초 동안 안 변했다"는
# 5초 하트비트 여러 장으로 보인다(9/10 링 흡수가 정확히 그 모양이었다).
ELEV_CHANGE_KEYS = ("phase", "target", "centered", "ready", "pressing", "press_status",
                    "scene", "scene_next_ok", "door_open", "door_base", "authority",
                    "guard_off", "lease_expired", "place", "floor", "floor_confirmed",
                    "lock_shape", "base_align", "align_note", "ocr_gated", "camera_missing",
                    "no_ocr")
SCENE_RESULT_KEYS = ("seq", "n", "running", "ok", "reason", "dist_m", "yaw_deg", "lat_cm",
                     "fallback", "fallback_ok")
# 부피만 크고 판단에 안 쓰는 필드 — detections 는 개수·글자만 요약해 남긴다.
ELEV_DROP_KEYS = ("detections", "clearance_stat")


# ── 코드 정체 ────────────────────────────────────────────────────────────────
def file_md5(path):
    """파일 md5 16진 문자열. 못 읽으면 None."""
    try:
        h = hashlib.md5()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()
    except Exception:
        return None


def src_fingerprint(path):
    """**import 시점에** 부른다 — 이 프로세스가 로드한 소스의 {path, md5, mtime, size, at}.

    파이썬은 소스를 import 때 한 번 읽는다. 그 직후에 잰 md5·mtime 이 곧 메모리에 올라간
    코드다. 여정 시작 시점에 재면 '그때의 디스크'를 재게 되고, 9/10 의 "작업 트리를
    실행판으로 착각"이 바로 그 차이였다."""
    p = os.path.abspath(str(path))
    try:
        st = os.stat(p)
        mtime, size = st.st_mtime, st.st_size
    except Exception:
        mtime = size = None
    return {"path": p, "md5": file_md5(p), "mtime": mtime, "size": size, "at": time.time()}


def proc_start_epoch(pid):
    """/proc 로 잰 프로세스 기동 시각(epoch 초). 없거나 못 읽으면 None. subprocess 없음."""
    try:
        with open(f"/proc/{int(pid)}/stat", "rb") as f:
            data = f.read().decode("utf-8", "replace")
        # comm 에 공백·괄호가 들어갈 수 있어 마지막 ')' 뒤에서 자른다.
        # 그 뒤 첫 토큰이 3번 필드(state)이므로 22번 필드(starttime)는 인덱스 19 다.
        start_ticks = int(data[data.rindex(")") + 2:].split()[19])
        btime = None
        with open("/proc/stat") as f:
            for line in f:
                if line.startswith("btime "):
                    btime = int(line.split()[1])
                    break
        if btime is None:
            return None
        return btime + start_ticks / os.sysconf("SC_CLK_TCK")
    except Exception:
        return None


def proc_identity(pid, src_path, at_import=None, **extra):
    """프로세스 하나의 코드 정체.

    {pid, alive, start_t, src, src_mtime, md5_now, stale, (md5_import, mtime_import), **extra}
    stale = 소스 mtime > 프로세스 기동 시각 → 이 프로세스는 **디스크의 지금 파일이 아닌**
    코드로 돈다. 엘베앱은 이미 떠 있으면 재사용되고(main.py `_auto_run` 의
    `if not _elev_app_running()`), interface 는 location.yaml 을 init 에 한 번만 읽는다 —
    둘 다 '어제 떠 있던 옛 코드'가 조용히 쓰일 수 있는 자리다."""
    out = dict(extra)
    out["pid"] = pid
    src = os.path.abspath(str(src_path)) if src_path else None
    out["src"] = src
    try:
        out["src_mtime"] = os.stat(src).st_mtime if src else None
    except Exception:
        out["src_mtime"] = None
    out["md5_now"] = file_md5(src) if src else None
    if at_import:
        out["md5_import"] = at_import.get("md5")
        out["mtime_import"] = at_import.get("mtime")
    st = proc_start_epoch(pid) if pid is not None else None
    out["alive"] = st is not None
    out["start_t"] = st
    # 기동 시각은 최대 1초 **이르게** 나온다 — /proc/stat 의 btime 이 정수 초라 부팅 시각의
    # 소수부가 잘린다(2026-09-11 실측 −0.27s). 그래서 1초 여유를 둔다: 기동 1초 안에 바뀐
    # 파일은 못 잡지만, 반대(멀쩡한 프로세스를 stale 로 오판)는 없다.
    out["stale"] = (out["src_mtime"] > st + STALE_SLACK_SEC) \
        if (st is not None and out["src_mtime"] is not None) else None
    return out


def git_identity(path_in_repo, timeout=5.0):
    """{head, src_dirty, dirty_files, rc, err, latency_ms}. **기동 시 1회, 백그라운드에서만** 부를 것.

    dirty 는 `git diff HEAD -- src/`(tracked 만)로 잰다. git status 기준이면 snapshots/ 아래
    untracked 폴더 때문에 **항상 참**이라 정보가 0이다(2026-09-11 advisor).
    GIT_OPTIONAL_LOCKS=0: 인덱스를 여러 세션이 공유한다. 기록 때문에 index.lock 을 잡지 않는다.
    이 값은 '디스크'의 정체다 — '메모리'의 정체는 src_fingerprint / proc_identity 가 말한다."""
    t0 = time.monotonic()
    out = {"head": None, "src_dirty": None, "dirty_files": None, "rc": None, "err": None}
    env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")

    def _git(*a):
        return subprocess.run(["git", "-c", "core.quotepath=false"] + list(a),
                              capture_output=True, text=True, timeout=timeout, env=env)
    try:
        d = path_in_repo if os.path.isdir(str(path_in_repo)) else os.path.dirname(str(path_in_repo))
        top = _git("-C", d, "rev-parse", "--show-toplevel")
        if top.returncode != 0:
            out.update(rc=top.returncode, err=(top.stderr or "").strip()[:200])
        else:
            root = top.stdout.strip()
            h = _git("-C", root, "rev-parse", "HEAD")
            out["head"] = h.stdout.strip() if h.returncode == 0 else None
            df = _git("-C", root, "diff", "--name-only", "HEAD", "--", "src/")
            out["rc"] = df.returncode
            if df.returncode == 0:
                files = [x for x in (df.stdout or "").splitlines() if x.strip()]
                out["src_dirty"] = bool(files)
                out["dirty_files"] = files[:30]
            else:
                out["err"] = (df.stderr or "").strip()[:200]
    except Exception as e:
        out["err"] = repr(e)
    out["latency_ms"] = round((time.monotonic() - t0) * 1000)
    return out


def code_identity_line(label, src_id, git, pid=None):
    """사람이 읽는 코드 정체 한 줄. 대시보드·엘베앱 기동 로그에 같은 모양으로 찍힌다."""
    s = src_id or {}
    g = git or {}
    pid = os.getpid() if pid is None else pid
    now_md5 = file_md5(s["path"]) if s.get("path") else None
    changed = (now_md5 != s.get("md5")) if (now_md5 and s.get("md5")) else None
    files = g.get("dirty_files") or []
    try:
        mt = datetime.datetime.fromtimestamp(s["mtime"]).strftime("%m-%d %H:%M:%S") \
            if s.get("mtime") else "?"
    except Exception:
        mt = "?"
    return (f"[CODE] {label} pid={pid} HEAD={(g.get('head') or '?')[:10]} "
            f"src_dirty={g.get('src_dirty')}"
            + (f"({len(files)}: {', '.join(files[:4])}{' …' if len(files) > 4 else ''})"
               if files else "")
            + f" md5(import)={(s.get('md5') or '?')[:12]} mtime={mt}"
            + (" ⚠ import 뒤 파일이 바뀌었다" if changed else "")
            + (f" git_err={g.get('err')}" if g.get("err") else ""))


def log_code_identity_async(label, src_id, log_fn, path=None):
    """git 은 느릴 수 있어 스레드로 돌려 한 줄을 남긴다(기동을 막지 않는다). 예외는 삼킨다."""
    def _run():
        try:
            g = git_identity(path or (src_id or {}).get("path") or os.getcwd())
            log_fn(code_identity_line(label, src_id, g))
        except Exception:
            pass
    try:
        threading.Thread(target=_run, name="journey-code-id", daemon=True).start()
    except Exception:
        pass


# ── 직렬화 보조 ──────────────────────────────────────────────────────────────
def _iso(ts):
    try:
        return datetime.datetime.fromtimestamp(ts).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
    except Exception:
        return None


def _json_default(o):
    if isinstance(o, (set, frozenset)):
        return sorted(o, key=repr)
    if isinstance(o, bytes):
        return o.decode("utf-8", "replace")
    try:
        return os.fspath(o)
    except TypeError:
        pass
    return repr(o)


def safe_name(s, limit=40):
    """파일명 안전화 — 글자(한글 포함)·숫자·-_ 만 남긴다."""
    out = "".join(c if (c.isalnum() or c in "-_") else "_" for c in str(s or "none"))
    return out[:limit] or "none"


# ── 레코더 ───────────────────────────────────────────────────────────────────
class _Run:
    """실행 하나의 상태. writer 가 쓰는 필드(fp·written·write_errors·closed)는 writer 만 만진다."""

    def __init__(self, rid, dest, path):
        self.id = rid
        self.dest = dest
        self.path = path
        self.mono0 = time.monotonic()
        self.seq = 0
        # writer 전용
        self.fp = None
        self.written = 0
        self.write_errors = 0
        self.open_failed = False
        self.warned = False
        self.closed = False
        # 호출자 쪽(레코더 락 아래)
        self.dropped = 0
        self.last_step_mono = None
        self.last_step = None
        self.phase = None
        self.wait_since = None
        self.shown_msg = None
        self.goal = None
        self.confirm_id = None        # 지금 열려 있는 '다음' 자리(C1~C9)
        self.first_error = None       # 첫 "오류" 단계
        self.first_abort = None       # 첫 "취소" 단계(사람 취소 없이 난 것 포함)
        self.first_done = None        # 첫 "완료" 단계
        self.human_cancel = None      # 사람이 누른 취소
        self.forced = []              # 사람이 누른 강제 넘기기
        self.exception = None
        self.sampler_stop = threading.Event()


class JourneyRecorder:
    """자동 여정 한 번의 기록기. 동시에 하나의 실행만 연다(대시보드 `_AUTO.active` 와 같다).

    공개 메서드는 전부 예외를 삼키고, 실행이 열려 있지 않으면 아무것도 안 한다."""

    def __init__(self, log_fn=None, src_id=None, repo_path=None, directory=None,
                 q_max=Q_MAX, pose_fn=None):
        self._log_fn = log_fn
        self.src_id = src_id
        self.repo_path = repo_path
        self.dir = directory or JOURNEY_DIR
        self.pose_fn = pose_fn          # 사람 이벤트 그림자용 — 메모리 값만 읽는 함수여야 한다
        self.git = None                 # boot() 가 백그라운드로 채운다
        self._q = queue.Queue(maxsize=q_max)
        self._lock = threading.Lock()
        self._run = None
        self._writer = None
        # 마지막으로 **이미 받은** /status — 새 호출 없이 그림자·run_end 가 읽는다
        self._st_last = None
        self._st_mono = None
        self._st_key = None
        self._st_emit_mono = 0.0
        self._st_miss = 0
        self.confirm_id_fn = None       # (step, msg) → '다음' 자리 식별자. 대시보드가 붙인다
        # 문 열림이 **대시보드가 본 기준으로** 처음 참이 된 시각. 엘베앱의 래치 시각이 아니라
        # 폴링 관측 시각이다(폴링 주기만큼 늦다). 씬이 바뀌거나 거짓으로 돌아가면 지운다.
        self._door_true_mono = None
        self._door_scene = None

    # ── 내부 ──
    @property
    def active(self):
        return self._run is not None

    def _say(self, msg):
        try:
            if self._log_fn is not None:
                self._log_fn(msg)
        except Exception:
            pass

    def _ensure_writer(self):
        w = self._writer
        if w is None or not w.is_alive():
            w = threading.Thread(target=self._writer_loop, name="journey-writer", daemon=True)
            self._writer = w
            w.start()

    def _put(self, run, ev, fields, deferred=None):
        """큐에 넣는다(호출자 스레드). 블로킹 없음 — 넘치면 버리고 센다."""
        rec = {"v": SCHEMA_V, "run": run.id, "seq": 0, "t": None,
               "ts": time.time(), "mono": round(time.monotonic(), 3), "ev": ev}
        rec.update(fields)
        with self._lock:
            run.seq += 1
            rec["seq"] = run.seq
            try:
                self._q.put_nowait((run, rec, deferred))
            except queue.Full:
                run.dropped += 1

    def _confirm_id(self, step, msg):
        try:
            return self.confirm_id_fn(step, msg) if self.confirm_id_fn is not None else None
        except Exception:
            return None

    def _pose(self):
        try:
            return self.pose_fn() if self.pose_fn is not None else None
        except Exception:
            return None

    # ── 기동 ──
    def boot(self, label="대시보드"):
        """기동 시 1회. git 은 느릴 수 있어 스레드로 — 결과는 self.git 에 캐시되고
        run_start 가 싣는다. 코드 정체 한 줄을 로그에 남긴다."""
        def _run():
            try:
                g = git_identity(self.repo_path or (self.src_id or {}).get("path")
                                 or os.getcwd())
                self.git = g
                self._say(code_identity_line(label, self.src_id, g))
            except Exception:
                pass
        try:
            threading.Thread(target=_run, name="journey-boot", daemon=True).start()
        except Exception:
            pass

    # ── 실행 경계 ──
    def begin(self, dest, fields=None, deferred=None, sample_fn=None, sample_period=1.0):
        """run_start. 반환: run_id(실패 시 None).

        fields   : 호출자가 **메모리에서** 읽은 값(dict)
        deferred : {필드명: 함수} — writer 스레드가 run_start 를 쓰기 직전에 부른다
                   (파일·/proc·pgrep·yaml 처럼 여정 스레드에 두면 안 되는 것)
        sample_fn: 1Hz 샘플러가 부를 함수(메모리 값만)"""
        try:
            if self._run is not None:
                self.end({"note": "새 여정이 시작돼 이전 기록을 닫았다"},
                         cause_override="superseded")
            ts = time.time()
            rid = time.strftime("%Y%m%dT%H%M%S", time.localtime(ts)) + "-" + os.urandom(2).hex()
            run = _Run(rid, dest, os.path.join(self.dir, f"{rid}_{safe_name(dest)}.jsonl"))
            self._ensure_writer()
            with self._lock:
                self._run = run
            d = dict(deferred or {})
            d.setdefault("git", lambda: self.git)
            self._put(run, "run_start",
                      dict(fields or {}, dest=dest, dash_src=self.src_id), d)
            if sample_fn is not None:
                threading.Thread(target=self._sampler, args=(run, sample_fn, sample_period),
                                 name="journey-sampler", daemon=True).start()
            return rid
        except Exception:
            return None

    def end(self, state=None, deferred=None, cause_override=None):
        """run_end — exit_cause(첫 종결 원인)·outcome·dur_s·state·elev_last·writer 카운터.

        종료 사유를 마지막 step 으로 읽지 않는다: 중단 경로 대부분이 `_auto_set("오류", 진짜사유)`
        뒤에 `_auto_abort_elev()` 가 `"취소"` 로 **덮어쓴다**(main.py `_auto_abort_elev`)."""
        try:
            with self._lock:
                run = self._run
                if run is None:
                    return
                self._run = None
            run.sampler_stop.set()
            cause, outcome = self._exit_cause(run, cause_override)
            self._put(run, "run_end",
                      {"exit_cause": cause, "outcome": outcome, "last_step": run.last_step,
                       "dur_s": round(time.monotonic() - run.mono0, 1),
                       "forced": run.forced or None, "state": dict(state or {}),
                       "elev_last": self._elev_summary()},
                      deferred)
        except Exception:
            pass

    @staticmethod
    def _exit_cause(run, override=None):
        """첫 종결 원인. 우선순위: 코드 예외 > (첫 '오류' 단계 ↔ 사람 취소 중 먼저 난 것)
        > 사람 취소 없이 난 '취소' 단계(대기 시간초과 등) > '완료' > 모름."""
        if override:
            return {"kind": override}, override
        if run.exception:
            return dict(run.exception, kind="exception", first_error=run.first_error), "exception"
        cands = [c for c in ((run.first_error, "error"), (run.human_cancel, "cancel")) if c[0]]
        if cands:
            data, kind = min(cands, key=lambda c: c[0].get("mono", 0.0))
            return dict(data, kind=kind), kind
        if run.first_abort:
            return dict(run.first_abort, kind="abort"), "abort"
        if run.first_done:
            msg = run.first_done.get("msg") or ""
            kind = "done" if ("✅" in msg or "🎉" in msg) else "fail"
            return dict(run.first_done, kind=kind), kind
        return {"kind": "unknown"}, "unknown"

    def _elev_summary(self):
        st, sm = self._st_last, self._st_mono
        if not isinstance(st, dict):
            return None
        out = {k: st.get(k) for k in ("guard_off", "authority", "place", "scene", "phase",
                                       "ready", "door_open", "pressing", "lift", "arm_ext",
                                       "floor", "floor_confirmed")}
        out["age_s"] = round(time.monotonic() - sm, 1) if sm else None
        return out

    def _sampler(self, run, fn, period):
        while not run.sampler_stop.wait(period):
            if self._run is not run:
                return
            try:
                self._put(run, "sample", dict(fn() or {}))
            except Exception:
                pass

    # ── 이벤트 ──
    def ev(self, ev, **fields):
        """아무 이벤트 한 줄(notify 등). 실행이 없으면 버린다."""
        try:
            run = self._run
            if run is not None:
                self._put(run, ev, fields)
        except Exception:
            pass

    def gate(self, name, **fields):
        """게이트 판정 — {name, inputs, result, why, latency_ms, rc, stderr_head, ...}.
        '무엇을 보고 그렇게 판정했나'가 남아야 데몬 staleness·P=None 같은 사고가 한 줄로 보인다."""
        self.ev("gate", name=name, **fields)

    def step(self, step, msg, wait=False, phase=None):
        """`_auto_set` 마다. {step, msg, wait, phase(유효값), phase_arg, dt_prev}."""
        try:
            run = self._run
            if run is None:
                return
            now = time.monotonic()
            cid = self._confirm_id(step, msg) if wait else None
            with self._lock:
                dt = None if run.last_step_mono is None else round(now - run.last_step_mono, 3)
                run.last_step_mono = now
                if phase is not None:
                    run.phase = phase
                if wait:
                    if run.wait_since is None:
                        run.wait_since = now
                    run.shown_msg = msg
                    run.confirm_id = cid
                else:
                    run.wait_since = None
                    run.shown_msg = None
                    run.confirm_id = None
                mark = {"step": step, "msg": msg, "phase": run.phase, "mono": round(now, 3),
                        "ts": time.time()}
                run.last_step = mark
                if step == "오류" and run.first_error is None:
                    run.first_error = mark
                elif step == "취소" and run.first_abort is None:
                    run.first_abort = mark
                elif step == "완료" and run.first_done is None:
                    run.first_done = mark
                cur_phase = run.phase
            self._put(run, "step", {"step": step, "msg": msg, "wait": bool(wait),
                                    "phase": cur_phase, "phase_arg": phase, "dt_prev": dt,
                                    "confirm_id": cid})
        except Exception:
            pass

    def human(self, kind, **fields):
        """사람 이벤트 — kind: confirm | confirm_locked | force | cancel.

        {kind, waited_s, shown_msg, shadow, ...호출자 필드}. 반환: waited_s(없으면 None).
        shadow = 그 순간 **이미 받아 둔** 자동 판정 값들(새 /status 호출 없음 — 클릭 응답이
        느려지면 사람이 두 번 누른다). 반복 실행이 곧 '사람 판단 vs 자동 판정' 섀도우 데이터가 된다."""
        try:
            run = self._run
            if run is None:
                return None
            now = time.monotonic()
            with self._lock:
                ws = run.wait_since
                shown = run.shown_msg
                cid = run.confirm_id if ws is not None else None
                mark = {"step": fields.get("step"), "mono": round(now, 3), "ts": time.time()}
                if kind == "cancel" and run.human_cancel is None:
                    run.human_cancel = mark
                elif kind == "force":
                    run.forced.append(mark)
            waited = round(now - ws, 1) if ws is not None else None
            self._put(run, "human", dict(fields, kind=kind, confirm_id=cid, waited_s=waited,
                                         shown_msg=shown, shadow=self.shadow()))
            return waited
        except Exception:
            return None

    def shadow(self):
        """자동 판정 그림자 — 캐시만 읽는다."""
        out = {}
        try:
            st, sm = self._st_last, self._st_mono
            if isinstance(st, dict):
                sr = st.get("scene_result")
                out.update(
                    door_open=st.get("door_open"), door_base=st.get("door_base"),
                    clear_f=st.get("clear_f"), ready=st.get("ready"),
                    centered=st.get("centered"), scene_next_ok=st.get("scene_next_ok"),
                    scene=st.get("scene"), phase=st.get("phase"), target=st.get("target"),
                    pressing=st.get("pressing"), lock_shape=st.get("lock_shape"),
                    scene_result=({k: sr.get(k) for k in SCENE_RESULT_KEYS}
                                  if isinstance(sr, dict) else None),
                    elev_age_s=round(time.monotonic() - sm, 2) if sm else None)
                # ③ 자동화 근거: 문 열림을 처음 본 뒤 경과(엘베앱은 한 번 참이면 씬이 끝날 때까지
                # 래치한다 — 지금 전방 여유가 기준선에서 얼마나 떴는지를 같이 봐야 '열렸다 닫힌 문'이 보인다)
                dts = self._door_true_mono
                out["door_open_seen_s"] = round(time.monotonic() - dts, 1) if dts else None
                try:
                    out["clear_jump_m"] = round(float(st["clear_f"]) - float(st["door_base"]), 2)
                except Exception:
                    out["clear_jump_m"] = None
            else:
                out["elev_age_s"] = None
            pose = self._pose()
            out["pose"] = pose
            run = self._run
            g = run.goal if run is not None else None
            if g and isinstance(pose, dict):
                out["goal"] = g.get("name")
                try:
                    out["d_m"] = round(math.hypot(float(pose["x"]) - float(g["x"]),
                                                  float(pose["y"]) - float(g["y"])), 3)
                except Exception:
                    out["d_m"] = None
                try:
                    out["dyaw_deg"] = round((float(pose["yaw_deg"]) - float(g["yaw_deg"])
                                             + 180.0) % 360.0 - 180.0, 1)
                except Exception:
                    out["dyaw_deg"] = None
        except Exception:
            pass
        return out

    def set_goal(self, name, loc):
        """현재 주행 목표(그림자의 d_m·dyaw_deg 기준). yaw 는 main.py ② 도착 로그와 같은 식."""
        try:
            run = self._run
            if run is None or not isinstance(loc, dict):
                return
            yaw = None
            try:
                yaw = math.degrees(2.0 * math.atan2(float(loc.get("z") or 0.0),
                                                    float(loc.get("w") or 1.0)))
            except Exception:
                pass
            run.goal = {"name": name, "x": loc.get("x"), "y": loc.get("y"), "yaw_deg": yaw}
        except Exception:
            pass

    def elev_status(self, st, err=None, caller=None):
        """`_elev_status()` 가 **이미 받은** 응답. 캐시는 항상 갱신하고, 실행 중에만 적는다.

        적는 때: ELEV_CHANGE_KEYS·scene_result 요약이 바뀔 때 + ELEV_HEARTBEAT_SEC 하트비트
        + 무응답 첫 회/하트비트. {ok, why(change|heartbeat|recover), caller, miss_before, st}
        st 에서 detections 는 det_n·det(글자·belief 12개)로 요약한다."""
        try:
            now = time.monotonic()
            if isinstance(st, dict):
                self._st_last, self._st_mono = st, now
                sc = st.get("scene")
                if sc != self._door_scene:
                    self._door_scene, self._door_true_mono = sc, None
                if st.get("door_open") is True:
                    if self._door_true_mono is None:
                        self._door_true_mono = now
                else:
                    self._door_true_mono = None
            run = self._run
            if run is None:
                return
            if not isinstance(st, dict):
                self._st_miss += 1
                if self._st_miss == 1 or now - self._st_emit_mono >= ELEV_HEARTBEAT_SEC:
                    self._st_emit_mono = now
                    self._put(run, "elev", {"ok": False, "caller": caller, "miss": self._st_miss,
                                            "err": repr(err) if err is not None
                                            else f"비정상 응답 {type(st).__name__}"})
                return
            miss, self._st_miss = self._st_miss, 0
            sr = st.get("scene_result")
            key = tuple(st.get(k) for k in ELEV_CHANGE_KEYS) + (
                tuple(sr.get(k) for k in ("seq", "n", "running", "ok", "reason"))
                if isinstance(sr, dict) else (None,))
            changed = key != self._st_key
            if not changed and not miss and now - self._st_emit_mono < ELEV_HEARTBEAT_SEC:
                return
            self._st_key, self._st_emit_mono = key, now
            rec = {k: v for k, v in st.items() if k not in ELEV_DROP_KEYS}
            dets = st.get("detections")
            if isinstance(dets, list):
                rec["det_n"] = len(dets)
                rec["det"] = [{"text": d.get("text"), "belief": d.get("belief"),
                               "shape": d.get("shape"), "suspect": d.get("suspect")}
                              for d in dets[:12] if isinstance(d, dict)]
            self._put(run, "elev", {"ok": True, "caller": caller, "miss_before": miss,
                                    "why": "change" if changed else
                                           ("recover" if miss else "heartbeat"),
                                    "st": rec})
        except Exception:
            pass

    def exception(self):
        """except 블록 **안에서** 부를 것 — traceback 전문을 싣는다(repr 만으로는 줄번호가 없다)."""
        try:
            run = self._run
            if run is None:
                return
            e = sys.exc_info()[1]
            tb = traceback.format_exc()
            with self._lock:
                if run.exception is None:
                    run.exception = {"repr": repr(e), "mono": round(time.monotonic(), 3),
                                     "ts": time.time()}
            self._put(run, "exception", {"repr": repr(e), "traceback": tb})
        except Exception:
            pass

    def proc(self, role, pid, src_path=None, spawned_by_run=True):
        """이 실행이 띄운 자식 프로세스 — 정체(/proc·mtime·md5)는 writer 가 잰다."""
        try:
            run = self._run
            if run is None:
                return
            self._put(run, "proc", {"role": role, "spawned_by_run": spawned_by_run},
                      {"identity": lambda: proc_identity(pid, src_path)})
        except Exception:
            pass

    def reject(self, dest, error, **fields):
        """시작 자체가 거부된 요청 — _rejected.jsonl 에 한 줄. 실행 파일은 만들지 않는다."""
        try:
            rec = {"v": SCHEMA_V, "t": None, "ts": time.time(),
                   "mono": round(time.monotonic(), 3), "ev": "reject", "dest": dest,
                   "error": error, "git_head": (self.git or {}).get("head")}
            rec.update(fields)
            self._ensure_writer()
            self._q.put_nowait((None, rec, None))
        except Exception:
            pass

    def traced(self, name, post=None, ev="call"):
        """호출 기록 데코레이터. {name, args, kwargs, ret | raised, latency_ms, **post(ret)}

        원 함수의 인자·반환·예외를 **그대로** 통과시킨다(행동 불변). 실행이 없으면 원 함수를
        바로 부른다. post(ret) 는 반환 직후 호출자 스레드에서 부르므로 메모리 값만 읽을 것
        (예: 대기 함수의 False 가 '취소'인지 '시간초과'인지 가르는 cancel 플래그)."""
        def deco(fn):
            @functools.wraps(fn)
            def wrapper(*args, **kwargs):
                if self._run is None:
                    return fn(*args, **kwargs)
                t0 = time.monotonic()
                try:
                    ret = fn(*args, **kwargs)
                except BaseException as e:
                    self._traced_rec(ev, name, args, kwargs, None, e, t0, post)
                    raise
                self._traced_rec(ev, name, args, kwargs, ret, None, t0, post)
                return ret
            return wrapper
        return deco

    def _traced_rec(self, ev, name, args, kwargs, ret, exc, t0, post):
        try:
            run = self._run
            if run is None:
                return
            rec = {"name": name, "args": list(args), "kwargs": kwargs or None,
                   "latency_ms": round((time.monotonic() - t0) * 1000, 1)}
            if exc is None:
                rec["ret"] = ret
            else:
                rec["raised"] = repr(exc)
            if post is not None:
                try:
                    rec.update(post(ret) or {})
                except Exception as pe:
                    rec["post_err"] = repr(pe)
            self._put(run, ev, rec)
        except Exception:
            pass

    # ── writer ──
    def _writer_loop(self):
        """큐를 비우는 유일한 스레드. deferred 해석 → 직렬화 → 한 줄 쓰고 flush."""
        while True:
            try:
                run, rec, deferred = self._q.get()
            except Exception:
                time.sleep(0.1)
                continue
            try:
                if deferred:
                    for k, fn in deferred.items():
                        try:
                            rec[k] = fn()
                        except Exception as e:
                            rec[k] = {"deferred_err": repr(e)}
                rec["t"] = _iso(rec.get("ts"))
                if run is None:
                    self._append_reject(rec)
                    continue
                ev = rec.get("ev")
                if ev == "run_end":
                    rec["writer"] = {"written": run.written, "dropped": run.dropped,
                                     "write_errors": run.write_errors}
                self._write_line(run, json.dumps(rec, ensure_ascii=False, default=_json_default))
                if ev == "run_end" and not run.closed:
                    run.closed = True
                    try:
                        if run.fp is not None:
                            run.fp.close()
                    except Exception:
                        pass
                    run.fp = None
                    oc = rec.get("outcome")
                    cz = rec.get("exit_cause") or {}
                    self._say(f"기록 종료 → {oc}"
                              + (f" [{cz.get('step')}] {str(cz.get('msg') or '')[:80]}"
                                 if cz.get("step") else "")
                              + f" (written {run.written}, dropped {run.dropped}, "
                                f"write_errors {run.write_errors}) {run.path}")
            except Exception:
                pass

    def _write_line(self, run, line):
        try:
            if run.closed:
                # run_end 뒤에 도착한 늦은 줄(다른 스레드가 종료 직전에 집어 둔 것) — 버리지 않는다.
                with open(run.path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
                run.written += 1
                return
            if run.fp is None:
                if run.open_failed:
                    run.write_errors += 1
                    return
                try:
                    os.makedirs(os.path.dirname(run.path), exist_ok=True)
                    run.fp = open(run.path, "a", encoding="utf-8")
                except Exception as e:
                    run.open_failed = True
                    run.write_errors += 1
                    self._say(f"⚠ 기록 파일을 못 열었다 — 이 여정은 블랙박스 없음: "
                              f"{run.path} ({e!r})")
                    return
                self._say(f"기록 시작 → {run.path}")
            run.fp.write(line + "\n")
            run.fp.flush()
            run.written += 1
        except Exception as e:
            run.write_errors += 1
            if not run.warned:
                run.warned = True
                self._say(f"⚠ 기록 쓰기 실패 — 이후 실패는 조용히 센다: {e!r}")

    def _append_reject(self, rec):
        try:
            os.makedirs(self.dir, exist_ok=True)
            with open(os.path.join(self.dir, REJECT_FILE), "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False, default=_json_default) + "\n")
        except Exception:
            pass

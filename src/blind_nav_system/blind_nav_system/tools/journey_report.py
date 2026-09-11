#!/usr/bin/env python3
"""journey_report.py — 여정 블랙박스(~/.ros/journey/*.jsonl) 판독기. ROS 없이 돈다(표준 라이브러리만).

쓰는 법
  python3 journey_report.py                   최신 실행 1건
  python3 journey_report.py <run_id 앞부분 | 파일 경로>
  python3 journey_report.py --all             실행별 한 줄 표 + 어느 단계가 제일 자주 깨지나
                                              + '다음' 자리별 사람 대기·자동판정 일치율
  옵션: --dir DIR     (기본 ~/.ros/journey)
        --diag DIR    (기본 ~/.ros/robot_diag — 로그 발췌용)
        --no-logs     로그 발췌 생략
        --before SEC  실패 단계 시작보다 이만큼 앞부터 발췌 (기본 10)
        --after SEC   종결 뒤 이만큼까지 발췌 (기본 5)
        --max-lines N 발췌 상한 (기본 160)

한 건 보기가 보여주는 것 (위에서 아래로 = 원인에 가까운 순서)
  1. 결말 — outcome · 첫 종결 원인 · 소요. run_end 가 없으면 그 사실을 맨 위에.
  2. 타임라인 — 단계 | 시작 | 소요 | 사람 대기 | 마지막 문구. 실패 단계 🔴.
  3. 판정 이상 — gate(입력·결론)와 call(반환)에서 실패·거부·시간초과만.
  4. 멈춘 구간 — 엘베앱이 ready 아닌 채 lift·arm_ext 가 오래 안 변한 구간(9/10 링 흡수 33초 모양),
     명령은 나가는데 pose 가 안 오는 구간(측위 사망 시그니처).
  5. 사람 확인 — 누른 순간의 자동판정 그림자(캐시 기준). 여정 중 조작 거부(C5).
  6. 측위(AMCL) 스냅샷 — ⑥ 하차 직후·목적지 출발 직전의 pose·공분산(σ).
  7. 코드 정체 — HEAD·src_dirty·프로세스별 stale.
  8. 마지막 엘베 상태 — 종결 직전 /status 핵심 필드.
  9. 실패 구간 로그 발췌 — elevator/*.log 핵심 태그 + 대시보드 자신의 system 줄.

왜 이렇게 읽나 (기록 쪽 설계와 짝이다 — journey_log.py 머리말)
  · 종결 원인은 run_end.exit_cause(첫 종결 원인)를 쓴다. 마지막 step 은 "취소"로 덮여 있을 수 있다.
  · seq 구멍 = 기록이 버려진 것. 조용히 넘기지 않는다.
  · system 로그는 자식 stdout 을 [system/ELEV]·[system/ROS2]·[system/NAV2] 로 **다시** 적는다.
    그대로 합치면 같은 사건을 두 번 센다(9/9·9/10 실제로 틀렸다). 그래서 엘베 줄은 elevator/*.log
    만 쓰고, system 에서는 대시보드 자신의 줄과 NAV2 경고만 쓰며, 같은 초·같은 내용은 한 번만 쓴다.
"""

import argparse
import collections
import datetime
import glob
import json
import math
import os
import re
import sys
import unicodedata

JOURNEY_DIR = os.path.expanduser("~/.ros/journey")
DIAG_DIR = os.path.expanduser("~/.ros/robot_diag")
REJECT_FILE = "_rejected.jsonl"
TERMINAL = ("오류", "완료", "취소")

# 엘베앱 핵심 태그 — 2026-09-11 robot_diag/elevator 전 파일 빈도 실측으로 골랐다.
# [OCR] 6821 · [INFER] 847 줄은 전체의 약 80% 라 뺐다(경고 기호가 있는 줄은 ALARM 으로 들어온다).
ELEV_TAGS = ("[ALIGN]", "[APPROACH]", "[LIFT]", "[PRESS]", "[SCAN]", "[TARGET]", "[SCENE]",
             "[AUTO]", "[MOVE]", "[GUARD]", "[AUTH]", "[LOCK]", "[READY]", "[HOLD]", "[PRIOR]",
             "[SEEK]", "[LEG]", "[TRACK]", "[ROTATE]", "[GOAL]", "[CODE]", "[GOAL]")
ALARM = ("⛔", "🚨", "⚠", "❌", "Traceback", "ERROR")
SYS_SRCS = {"AUTO", "JOURNEY", "ELEVLEASE", "MAP", "IFACE", "SYS", "EXIT", "MAIN"}
NAV2_KEEP = re.compile(r"warn|error|fail|abort|reject|거부|실패|⚠|🚨", re.I)
LINE_RE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}) \[(\w+)/(\w+)\] ?(.*)$")
INNER_PREFIX = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3} \[\w+/\w+\] |"
                          r"\[(INFO|WARN|ERROR|DEBUG)\] \[[\d.]+\] \[[^\]]+\]: )")
NUM = re.compile(r"[-+]?\d+(?:\.\d+)?")

# 자리별 "그 순간 자동판정이 이미 참이었나" — 사람 판단과의 일치율을 쌓는 기준(캐시 기준값).
SHADOW_CHECK = {
    "C1_call_press": ("ready", lambda s: s.get("ready") is True),
    "C8_floor_press": ("ready", lambda s: s.get("ready") is True),
    "C4_front_arrived": ("d≤10cm", lambda s: isinstance(s.get("d_m"), (int, float))
                         and s["d_m"] <= 0.10),
    "C6_door_open": ("door_open", lambda s: s.get("door_open") is True),
    "C7_ride_in": ("scene_result.ok", lambda s: (s.get("scene_result") or {}).get("ok") is True),
}


# ── 표시 보조 ────────────────────────────────────────────────────────────────
def wlen(s):
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in str(s))


def wpad(s, w):
    s = str(s)
    if wlen(s) > w:
        out, n = "", 0
        for c in s:
            cw = 2 if unicodedata.east_asian_width(c) in "WF" else 1
            if n + cw > w - 1:
                break
            out += c
            n += cw
        s = out + "…"
    return s + " " * max(0, w - wlen(s))


def hms(ts):
    try:
        return datetime.datetime.fromtimestamp(float(ts)).strftime("%H:%M:%S")
    except Exception:
        return "--:--:--"


def dur(sec):
    if not isinstance(sec, (int, float)):
        return "?"
    sec = float(sec)
    return f"{sec:.1f}s" if sec < 60 else f"{int(sec // 60)}m{sec % 60:04.1f}s"


def short(v, n=80):
    s = v if isinstance(v, str) else json.dumps(v, ensure_ascii=False, default=str)
    return s if len(s) <= n else s[:n - 1] + "…"


# ── 읽기 ────────────────────────────────────────────────────────────────────
def run_files(d):
    return sorted(p for p in glob.glob(os.path.join(d, "*.jsonl"))
                  if os.path.basename(p) != REJECT_FILE)


def load_run(path):
    recs, bad = [], 0
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            try:
                recs.append(json.loads(line))
            except Exception:
                bad += 1          # 대시보드가 쓰는 중 죽으면 마지막 줄이 잘릴 수 있다
    recs.sort(key=lambda r: r.get("seq") if isinstance(r.get("seq"), int) else 0)
    return recs, bad


def analyze(recs):
    by = collections.defaultdict(list)
    for r in recs:
        by[r.get("ev")].append(r)
    a = {"by": by, "start": (by["run_start"] or [None])[0], "end": (by["run_end"] or [None])[-1]}
    seqs = sorted(r["seq"] for r in recs if isinstance(r.get("seq"), int))
    a["gaps"] = (seqs[-1] - len(set(seqs))) if seqs else 0
    stages = []
    for s in by["step"]:
        if stages and stages[-1]["step"] == s.get("step"):
            stages[-1]["msgs"].append(s)
        else:
            stages.append({"step": s.get("step"), "ts0": s.get("ts"), "msgs": [s], "human": []})
    last_ts = a["end"]["ts"] if a["end"] else (recs[-1].get("ts") if recs else None)
    for i, st in enumerate(stages):
        st["ts1"] = stages[i + 1]["ts0"] if i + 1 < len(stages) else last_ts
        st["last"] = st["msgs"][-1]
        st["cids"] = sorted({m.get("confirm_id") for m in st["msgs"] if m.get("confirm_id")})
    for h in by["human"]:
        for st in stages:
            if st["ts0"] is not None and st["ts0"] <= h.get("ts", 0) and \
                    (st["ts1"] is None or h.get("ts", 0) <= st["ts1"]):
                st["human"].append(h)
    a["stages"] = stages
    # 실패 단계: 첫 종결 원인이 가리키는 단계의 **직전** 실제 단계("오류"·"취소" 표지 앞)
    cause = (a["end"] or {}).get("exit_cause") or {}
    a["cause"] = cause
    fail_idx = None
    cts = cause.get("ts") or ((cause.get("first_error") or {}).get("ts"))
    if cause.get("kind") not in (None, "done") and cts is not None:
        for i, st in enumerate(stages):
            if st["ts0"] is not None and st["ts0"] <= cts:
                fail_idx = i
        while fail_idx is not None and fail_idx > 0 and stages[fail_idx]["step"] in TERMINAL:
            fail_idx -= 1
    elif a["end"] is None and stages:
        fail_idx = len(stages) - 1
    a["fail_idx"] = fail_idx
    a["fail_ts"] = cts if cts is not None else last_ts
    return a


# ── 판정 이상 ────────────────────────────────────────────────────────────────
def gate_issue(g):
    n = g.get("name")
    if g.get("raised"):
        return f"{n} 예외 {g['raised']}"
    if n in ("param_get", "map_server_yaml"):
        if g.get("exc"):
            return f"{n} {short(g.get('inputs') or '', 50)} 예외 {g['exc']} ({g.get('latency_ms')}ms)"
        if g.get("rc") not in (0, None):
            return (f"{n} {short(g.get('inputs') or '', 50)} rc={g['rc']} "
                    f"stderr={short(g.get('stderr_head') or '', 60)} ({g.get('latency_ms')}ms)")
    elif n == "arrival" and g.get("result") is False:
        return (f"도착 판정 실패 [{(g.get('inputs') or {}).get('name')}] {g.get('why')} "
                f"d={g.get('d_last')} 경과={g.get('elapsed_s')}s "
                f"pose_age={((g.get('pose') or {}).get('pose_age_s'))}")
    elif n == "lease_attempt" and g.get("ok") is False:
        return f"리스={g.get('granted')} 시도{g.get('attempt')}/{g.get('attempts')} 실패 실측={g.get('st')}"
    elif n == "nav_padding_read" and g.get("cur") != g.get("want"):
        return f"footprint_padding 읽기 cur={g.get('cur')} want={g.get('want')} [{g.get('why')}]"
    elif n == "nav_padding_write":
        return f"footprint_padding 쓰기 ok={g.get('ok')} {short(g.get('err') or '', 60)} [{g.get('why')}]"
    elif n == "switch_map_resp":
        body = g.get("body") or {}
        if g.get("status", 200) >= 400 or body.get("ok") is False:
            return f"/switch_map {g.get('req')} → HTTP {g.get('status')} {short(body, 70)}"
    elif n in ("grant_elev_lease", "nav_params_restore_normal") and g.get("ret") is False:
        return f"{n}{tuple(g.get('args') or [])} → False"
    elif n == "map_loaded_floor":
        ret = g.get("ret") or [None, None]
        if isinstance(ret, list) and ret and ret[0] is None:
            return f"지도 층 '모름' — {ret[1] if len(ret) > 1 else ''} (R={g.get('R_path')})"
    elif n == "nav_release_wheels" and g.get("ret") is False:
        return f"Nav2 바퀴 반납 확인 실패 {tuple(g.get('args') or [])} — 엘베앱에 제어권을 넘기지 않았다"
    return None


def call_issue(c):
    n, ret = c.get("name"), c.get("ret")
    if c.get("raised"):
        return f"{n} 예외 {c['raised']}"
    args = short(c.get("args") or [], 40)
    if n == "auto_wait_confirm" and ret is False:
        return f"'다음' 대기 실패 — {'취소' if c.get('cancel_flag') else '시간초과'} ({dur(c.get('latency_ms', 0) / 1000)})"
    if n in ("elev_wait_ready", "elev_wait_press_done") and ret is False:
        return f"{n} → False {'(취소)' if c.get('cancel_flag') else '(시간초과)'} {dur(c.get('latency_ms', 0) / 1000)}"
    if n in ("elev_select", "http_self", "wait_elev_app_up", "press_or_pass",
             "auto_front_nav2") and ret is False:
        return f"{n}{args} → False ({dur(c.get('latency_ms', 0) / 1000)})"
    if n == "elev_press" and isinstance(ret, list) and ret and ret[0] is False:
        return f"/press 거부: {ret[1] if len(ret) > 1 else ''}"
    if n == "elev_scene" and isinstance(ret, list) and ret and ret[0] is False:
        return f"/scene{args} 전송 실패"
    if n == "elev_wait_scene_done" and isinstance(ret, list) and ret and ret[0] not in ("done", "noanmu"):
        return f"씬{args} 안무 → {ret[0]} {short(ret[1] if len(ret) > 1 else '', 60)}"
    if n == "elev_wait_exit_done" and isinstance(ret, list) and ret and ret[0] != "done":
        return f"⑥ 하차 → {ret[0]} 진행={ret[2] if len(ret) > 2 else '?'}cm"
    if n == "auto_scene_step" and isinstance(ret, list) and len(ret) > 1 and ret[1] not in ("done", "noanmu"):
        return f"{args} → {ret[1]}"
    if n == "http_self_json" and c.get("args") and c["args"][0] == "/switch_map":
        body, err = ret if (isinstance(ret, list) and len(ret) == 2) else (None, None)
        if not (isinstance(body, dict) and body.get("ok") is True):
            return (f"/switch_map 확인 실패 — {(body or {}).get('error') or err or '본문 없음'} "
                    f"({dur(c.get('latency_ms', 0) / 1000)})")
    if n == "elev_post" and isinstance(ret, dict) and ret.get("ok") is False:
        return f"엘베 {args} 거부 — {short(ret.get('error') or '', 90)}"
    if n == "elev_post" and ret is None:
        return f"엘베 {args} 응답 없음"
    return None


def stuck_spans(elevs, t0, t1, min_s=15.0):
    """엘베앱이 ready 아닌 채 lift·arm_ext·phase 가 min_s 이상 안 변한 구간."""
    out, run = [], None
    for e in elevs:
        if not e.get("ok") or not (t0 <= e.get("ts", 0) <= t1):
            continue
        st = e.get("st") or {}
        key = (round(st.get("lift") or 0, 3), round(st.get("arm_ext") or 0, 3), st.get("phase"),
               bool(st.get("ready")))
        if run and run["key"] == key:
            run["ts1"] = e["ts"]
            run["st"] = st
        else:
            if run:
                out.append(run)
            run = {"key": key, "ts0": e["ts"], "ts1": e["ts"], "st": st}
    if run:
        out.append(run)
    return [r for r in out if not r["key"][3] and r["ts1"] - r["ts0"] >= min_s]


def pose_dead(samples, t0, t1):
    """명령은 나가는데(cmd_age<1s) pose 가 5초 넘게 안 온 샘플 수."""
    n = 0
    for s in samples:
        p = s.get("pose") or {}
        if t0 <= s.get("ts", 0) <= t1 and isinstance(p.get("pose_age_s"), (int, float)) \
                and p["pose_age_s"] > 5 and isinstance(p.get("cmd_age_s"), (int, float)) \
                and p["cmd_age_s"] < 1:
            n += 1
    return n


# ── 로그 발췌 ────────────────────────────────────────────────────────────────
def _parse_ts(s):
    return datetime.datetime.strptime(s, "%Y-%m-%d %H:%M:%S.%f").timestamp()


def _file_start(p):
    m = re.search(r"_(\d{8}_\d{6})_pid", os.path.basename(p))
    try:
        return datetime.datetime.strptime(m.group(1), "%Y%m%d_%H%M%S").timestamp()
    except Exception:
        return None


def log_lines(diag, t0, t1):
    """(ts, 출처, 태그, 본문) 목록. 엘베는 elevator/*.log, system 은 대시보드 자신의 줄만."""
    out = []
    for tag in ("elevator", "system"):
        for p in sorted(glob.glob(os.path.join(diag, tag, f"{tag}_*.log"))):
            fs = _file_start(p)
            try:
                if (fs is not None and fs > t1) or os.path.getmtime(p) < t0:
                    continue
                f = open(p, encoding="utf-8", errors="replace")
            except Exception:
                continue
            with f:
                for line in f:
                    m = LINE_RE.match(line.rstrip("\n"))
                    if not m:
                        continue
                    try:
                        ts = _parse_ts(m.group(1))
                    except Exception:
                        continue
                    if not (t0 <= ts <= t1):
                        continue
                    src, msg = m.group(3), m.group(4)
                    if tag == "elevator":
                        if any(k in msg for k in ELEV_TAGS) or any(k in msg for k in ALARM):
                            out.append((ts, "elev", src, msg))
                    else:
                        if src in SYS_SRCS or (src == "NAV2" and NAV2_KEEP.search(msg)):
                            out.append((ts, "dash", src, msg))
    out.sort(key=lambda x: x[0])
    # system 이중계수 제거 — 같은 초 · 앞머리(내부 타임스탬프·rclpy 머리) 뗀 같은 내용은 한 번
    seen, uniq = set(), []
    for ts, who, src, msg in out:
        key = (int(ts), who, INNER_PREFIX.sub("", msg))
        if key in seen:
            continue
        seen.add(key)
        uniq.append((ts, who, src, msg))
    return uniq


def collapse(lines, max_lines):
    """숫자만 다른 연속 줄([SCAN] 반복 등)을 접는다."""
    out, prev, rep = [], None, 0
    for ts, who, src, msg in lines:
        norm = (who, NUM.sub("#", msg))
        if norm == prev:
            rep += 1
            continue
        if rep:
            out.append(f"{'':12}… 숫자만 다른 같은 모양 {rep}줄 생략")
            rep = 0
        prev = norm
        out.append(f"{hms(ts)} {who:4} {short(msg, 150)}")
    if rep:
        out.append(f"{'':12}… 숫자만 다른 같은 모양 {rep}줄 생략")
    if len(out) > max_lines:
        cut = len(out) - max_lines
        out = out[:max_lines // 2] + [f"{'':12}… 중간 {cut}줄 생략 (--max-lines)"] + out[-max_lines // 2:]
    return out


# ── 한 건 보기 ───────────────────────────────────────────────────────────────
def report_one(path, args):
    recs, bad = load_run(path)
    if not recs:
        print(f"빈 기록: {path}")
        return 2
    a = analyze(recs)
    st0, end, by = a["start"] or {}, a["end"], a["by"]
    P = print
    P(f"═══ 여정 {st0.get('run') or recs[0].get('run')} → {st0.get('dest')}  ({os.path.basename(path)})")
    pr = st0.get("prev_run")
    if isinstance(pr, dict):
        fcp = pr.get("floor_check") or {}
        P(f"   직전 실행({pr.get('source')}): {pr.get('run') or pr.get('file')} "
          + (pr.get("outcome") or ("run_end 없음" if pr.get("no_run_end") else "?"))
          + f" — {short((pr.get('exit_cause') or {}).get('msg') or '', 60)}"
          + (f" · 끝 층 {fcp.get('floor')}/목적 {fcp.get('dest_floor')} match={fcp.get('match')}" if fcp else ""))
    # 1. 결말
    if end is None:
        P("🚨 run_end 없음 — 대시보드가 여정 도중 죽었거나 아직 진행 중이다 "
          f"(마지막 기록 {hms(recs[-1].get('ts'))} {recs[-1].get('ev')})")
    else:
        cz = a["cause"]
        P(f"■ 결말: {end.get('outcome')}  소요 {dur(end.get('dur_s'))}  "
          f"첫 종결 원인 [{cz.get('kind')}] {cz.get('step') or ''} {short(cz.get('msg') or cz.get('repr') or '', 100)}")
        wr = end.get("writer") or {}
        if wr.get("dropped") or wr.get("write_errors") or a["gaps"] or bad:
            P(f"   ⚠ 기록 손실: dropped {wr.get('dropped')} · write_errors {wr.get('write_errors')} · "
              f"seq 구멍 {a['gaps']} · 깨진 줄 {bad}")
        if end.get("forced"):
            P(f"   ⏭ 강제 넘어가기 {len(end['forced'])}회: " + ", ".join(
                f"{hms(x.get('ts'))} [{x.get('step')}]" for x in end["forced"]))
        stt = end.get("state") or {}
        P(f"   종료 상태: arm_safe={stt.get('arm_safe')} lease={stt.get('lease_held')} "
          f"rescue_hold={stt.get('rescue_hold')} floor={stt.get('floor')}"
          f"({'확정' if stt.get('floor_confirmed') else '미확정'}) elev_running={end.get('elev_running')} "
          f"guard_off={(end.get('elev_last') or {}).get('guard_off')}")
        fc = end.get("floor_check") or {}
        if fc:
            mark = {True: "✅ 일치", False: "🔴 불일치"}.get(fc.get("match"), "⚪ 판정 안 함(미확정·정보 없음)")
            P(f"   층 대조: 끝 {fc.get('floor')} / 목적 {fc.get('dest_floor')} ({fc.get('mode')}) — {mark}")
        sm, smc = end.get("switch_map") or {}, end.get("switch_map_call") or {}
        if sm or smc:
            P(f"   지도 전환: 본문 ok={sm.get('ok')} floor={sm.get('floor')} {sm.get('error') or ''}"
              + (f" · 여정 쪽 반환 {short(smc.get('ret'), 60)}" if smc else ""))
    for ex in by["exception"]:
        P("■ 예외 traceback:")
        for ln in (ex.get("traceback") or "").rstrip().splitlines()[-12:]:
            P("   " + ln)
    # 2. 타임라인
    P("■ 타임라인")
    P("   " + wpad("단계", 18) + wpad("시작", 9) + wpad("소요", 9) + wpad("사람대기", 10) + "마지막 문구")
    for i, st in enumerate(a["stages"]):
        mark = "🔴" if i == a["fail_idx"] else "  "
        hw = [h.get("waited_s") for h in st["human"] if isinstance(h.get("waited_s"), (int, float))]
        hwt = dur(sum(hw)) if hw else ("잠김클릭" if st["human"] else "")
        span = (st["ts1"] - st["ts0"]) if (st["ts1"] and st["ts0"]) else None
        P(f"{mark} " + wpad(st["step"], 18) + wpad(hms(st["ts0"]), 9) + wpad(dur(span), 9)
          + wpad(hwt, 10) + short(st["last"].get("msg") or "", 90))
    # 3. 판정 이상
    issues = []
    for g in by["gate"]:
        s = gate_issue(g)
        if s:
            issues.append((g.get("ts"), "gate", s))
    for c in by["call"]:
        s = call_issue(c)
        if s:
            issues.append((c.get("ts"), "call", s))
    for e in by["elev"]:
        if e.get("ok") is False and (e.get("miss") or 0) == 1:
            issues.append((e.get("ts"), "elev", f"/status 무응답 ({e.get('caller')}) {short(e.get('err') or '', 60)}"))
    P("■ 판정 이상" + ("" if issues else " — 없음"))
    for ts, kind, s in sorted(issues, key=lambda x: x[0] or 0):
        P(f"   {hms(ts)} {kind:4} {s}")
    # 4. 멈춘 구간
    t_fail = a["fail_ts"] or recs[-1].get("ts")
    fi = a["fail_idx"]
    t_win0 = (a["stages"][fi]["ts0"] if fi is not None else st0.get("ts", t_fail)) - args.before
    t_win1 = (t_fail or 0) + args.after
    spans = stuck_spans(by["elev"], st0.get("ts", 0), t_win1)
    dead = pose_dead(by["sample"], st0.get("ts", 0), t_win1)
    holds = [g for g in by["gate"] if g.get("name") == "auto_hold" and (g.get("latency_ms") or 0) >= 500]
    if spans or dead or holds:
        P("■ 멈춘 구간")
        for g in holds:
            P(f"   ⏸ {hms(g.get('ts'))} 멈춤 [{(g.get('args') or ['?'])[0]}] {dur((g.get('latency_ms') or 0) / 1000)}"
              f" → {'재개' if g.get('ret') else '멈춘 채 취소'}")
        for r in spans:
            s = r["st"]
            P(f"   ⚠ {hms(r['ts0'])}~{hms(r['ts1'])} {dur(r['ts1'] - r['ts0'])} 동안 lift {s.get('lift')} · "
              f"arm_ext {s.get('arm_ext')} 불변, ready=False phase={s.get('phase')} "
              f"centered={s.get('centered')} ex/ey={s.get('ex')}/{s.get('ey')} dz={s.get('dz')}")
        if dead:
            P(f"   🚨 명령은 나가는데 pose 가 5초 넘게 안 온 샘플 {dead}개 — 측위 사망 시그니처")
    # 5. 사람 확인
    if by["human"]:
        P("■ 사람 확인 (누른 순간 캐시된 자동판정)")
        for h in by["human"]:
            sh = h.get("shadow") or {}
            sr = sh.get("scene_result") or {}
            P(f"   {hms(h.get('ts'))} {wpad(h.get('confirm_id') or '-', 22)} {h.get('kind'):14} "
              f"대기 {dur(h.get('waited_s'))}  ready={sh.get('ready')} lock_shape={sh.get('lock_shape')} "
              f"door_open={sh.get('door_open')}(본지 {sh.get('door_open_seen_s')}s, 점프 {sh.get('clear_jump_m')}m) "
              f"씬결과={sr.get('ok')}/{sr.get('reason')} d={sh.get('d_m')} dyaw={sh.get('dyaw_deg')} "
              f"elev_age={sh.get('elev_age_s')}s")
    if by["blocked"]:
        P("■ 여정 중 조작 거부")
        for b in by["blocked"]:
            P(f"   {hms(b.get('ts'))} {b.get('what')} [{b.get('step')}] ({b.get('remote_addr')})")
    # 6. 측위 스냅샷
    if by["amcl"]:
        P("■ 측위(AMCL) 스냅샷")

        def _sd(v, deg=False):
            """분산 → 표준편차(cm 또는 °). 못 읽으면 '?'."""
            try:
                s = math.sqrt(float(v))
                return f"{math.degrees(s):.1f}°" if deg else f"{s * 100:.1f}cm"
            except Exception:
                return "?"
        # 루프 변수를 `a` 로 두지 않는다 — 이 함수의 분석 결과 `a` 를 덮어써 아래 발췌가 깨졌다.
        for am in by["amcl"]:
            ap = am.get("pose") or {}
            cv = ap.get("cov") or {}
            acc = (am.get("elev") or {}).get("scene_acc")
            P(f"   {hms(am.get('ts'))} {wpad(am.get('where') or '?', 16)} x={ap.get('x')} y={ap.get('y')} "
              f"yaw={ap.get('yaw_deg')} σx={_sd(cv.get('xx'))} σy={_sd(cv.get('yy'))} "
              f"σyaw={_sd(cv.get('yawyaw'), True)} pose_age={ap.get('pose_age_s')}s"
              + (f" scene_acc={acc}" if acc else ""))
    # 7. 코드 정체
    P("■ 코드 정체")
    g = st0.get("git") or {}
    P(f"   HEAD {(g.get('head') or '?')[:10]}  src_dirty={g.get('src_dirty')}"
      + (f" {g.get('dirty_files')}" if g.get("dirty_files") else "")
      + (f"  git_err={g.get('err')}" if g.get("err") else ""))
    procs = st0.get("procs") or {}
    for role in ("dash", "iface", "elev"):
        p = procs.get(role) or {}
        warn = []
        if p.get("stale"):
            warn.append("⚠ stale — 기동 뒤 소스가 바뀌었다(디스크의 지금 파일이 아닌 코드로 돈다)")
        if p.get("md5_import") and p.get("md5_now") and p["md5_import"] != p["md5_now"]:
            warn.append("⚠ import 뒤 파일이 바뀌었다")
        if role == "elev" and (p.get("found") or 0) > 1:
            warn.append(f"⚠ 엘베앱 {p['found']}개")
        P(f"   {role:5} pid={p.get('pid')} alive={p.get('alive')} "
          f"기동={hms(p.get('start_t')) if p.get('start_t') else '-'} "
          f"src_mtime={hms(p.get('src_mtime')) if p.get('src_mtime') else '-'} " + " ".join(warn))
    for pr in by["proc"]:
        idn = pr.get("identity") or {}
        P(f"   이번 실행이 띄움: {pr.get('role')} pid={idn.get('pid')} stale={idn.get('stale')}")
    # 8. 마지막 엘베 상태
    last_ok = [e for e in by["elev"] if e.get("ok") and e.get("ts", 0) <= (t_fail or 1e18)]
    if last_ok:
        s = last_ok[-1].get("st") or {}
        sr = s.get("scene_result") or {}
        P(f"■ 마지막 엘베 상태 ({hms(last_ok[-1].get('ts'))}, {last_ok[-1].get('why')})")
        P(f"   phase={s.get('phase')} target={s.get('target')} ready={s.get('ready')} "
          f"centered={s.get('centered')} lock_shape={s.get('lock_shape')} ex/ey={s.get('ex')}/{s.get('ey')} "
          f"dz={s.get('dz')} dist={s.get('dist')} lift={s.get('lift')} arm_ext={s.get('arm_ext')}")
        P(f"   scene={s.get('scene')} place={s.get('place')} floor={s.get('floor')}"
          f"({'확정' if s.get('floor_confirmed') else '미확정'}) authority={s.get('authority')} "
          f"guard_off={s.get('guard_off')} door_open={s.get('door_open')} clear_f={s.get('clear_f')} "
          f"씬결과 n={sr.get('n')} running={sr.get('running')} ok={sr.get('ok')} {sr.get('reason') or ''} "
          f"press={s.get('press_status')} ocr_gated={s.get('ocr_gated')}")
    # 9. 로그 발췌
    if not args.no_logs:
        lines = log_lines(args.diag, t_win0, t_win1)
        P(f"■ 로그 발췌 {hms(t_win0)} ~ {hms(t_win1)}"
          + (f" (실패 단계 [{a['stages'][fi]['step']}] 앞 {args.before:.0f}s ~ 종결 뒤 {args.after:.0f}s)"
             if fi is not None else "") + f" — {len(lines)}줄")
        for ln in collapse(lines, args.max_lines):
            P("   " + ln)
    return 0


# ── 전체 요약 ────────────────────────────────────────────────────────────────
def report_all(args):
    files = run_files(args.dir)
    if not files:
        print(f"기록 없음: {args.dir}")
        return 2
    rows, fail_stage, cid_stats = [], collections.Counter(), collections.defaultdict(list)
    P = print
    P(wpad("시작", 15) + wpad("코드", 10) + wpad("목적지", 12) + wpad("결말", 10) + wpad("층", 4)
      + wpad("도달 단계", 16) + wpad("소요", 9) + wpad("사람", 10) + "첫 종결 원인")
    for p in files:
        recs, bad = load_run(p)
        if not recs:
            continue
        a = analyze(recs)
        st0, end = a["start"] or {}, a["end"]
        g = st0.get("git") or {}
        procs = st0.get("procs") or {}
        code = (g.get("head") or "?")[:7] + ("*" if g.get("src_dirty") else "") \
            + ("!" if any((procs.get(k) or {}).get("stale") for k in procs) else "")
        real = [s for s in a["stages"] if s["step"] not in TERMINAL]
        reached = (a["stages"][a["fail_idx"]]["step"] if a["fail_idx"] is not None
                   else (real[-1]["step"] if real else "-"))
        outcome = end.get("outcome") if end else "no_end"   # 열 폭 안에 들게 ASCII — run_end 가 없다
        fc = (end or {}).get("floor_check") or {}
        fcm = {True: "✅", False: "🔴"}.get(fc.get("match"), "⚪") if fc else "-"
        cz = a["cause"]
        hs = [h for h in a["by"]["human"] if isinstance(h.get("waited_s"), (int, float))]
        loss = (end or {}).get("writer", {}).get("dropped") or a["gaps"] or bad
        P(wpad(datetime.datetime.fromtimestamp(st0.get("ts", recs[0].get("ts", 0))).strftime("%m-%d %H:%M:%S"), 15)
          + wpad(code, 10) + wpad(st0.get("dest") or "?", 12) + wpad(outcome, 10) + wpad(fcm, 4)
          + wpad(reached, 16)
          + wpad(dur((end or {}).get("dur_s")), 9)
          + wpad(f"{len(hs)}회 {dur(sum(h['waited_s'] for h in hs))}" if hs else "-", 10)
          + short(f"{cz.get('step') or ''} {cz.get('msg') or cz.get('repr') or ''}".strip(), 60)
          + ("  ⚠기록손실" if loss else ""))
        if outcome != "done":
            fail_stage[reached] += 1
        for h in a["by"]["human"]:
            if h.get("kind") == "confirm" and h.get("confirm_id"):
                cid_stats[h["confirm_id"]].append(h)
    P("")
    P("■ 끝나지 못한 실행의 도달 단계 (많이 깨지는 순)" + ("" if fail_stage else " — 없음"))
    for k, n in fail_stage.most_common():
        P(f"   {n:3}회  {k}")
    if cid_stats:
        P("■ '다음' 자리별 — 사람 대기와 그 순간 자동판정(캐시 기준)")
        for cid in sorted(cid_stats):
            hs = cid_stats[cid]
            ws = sorted(h["waited_s"] for h in hs if isinstance(h.get("waited_s"), (int, float)))
            med = ws[len(ws) // 2] if ws else None
            chk = SHADOW_CHECK.get(cid)
            agree = ""
            if chk:
                yes = sum(1 for h in hs if chk[1](h.get("shadow") or {}))
                agree = f"  자동판정 {chk[0]} 이미 참 {yes}/{len(hs)}"
            P(f"   {wpad(cid, 24)} {len(hs):3}회  대기 중앙값 {dur(med)}{agree}")
    rj = os.path.join(args.dir, REJECT_FILE)
    if os.path.exists(rj):
        c = collections.Counter()
        with open(rj, encoding="utf-8") as f:
            for line in f:
                try:
                    c[json.loads(line).get("error")] += 1
                except Exception:
                    pass
        P("■ 시작 거부 (/auto_goto)")
        for k, n in c.most_common():
            P(f"   {n:3}회  {k}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="여정 블랙박스 판독기 (ROS 불필요)")
    ap.add_argument("run", nargs="?", help="run_id 앞부분 또는 .jsonl 경로 (없으면 최신)")
    ap.add_argument("--all", action="store_true", help="실행별 한 줄 요약")
    ap.add_argument("--dir", default=JOURNEY_DIR)
    ap.add_argument("--diag", default=DIAG_DIR)
    ap.add_argument("--no-logs", action="store_true")
    ap.add_argument("--before", type=float, default=10.0)
    ap.add_argument("--after", type=float, default=5.0)
    ap.add_argument("--max-lines", type=int, default=160)
    args = ap.parse_args(argv)
    if args.all:
        return report_all(args)
    if args.run and os.path.isfile(args.run):
        return report_one(args.run, args)
    files = run_files(args.dir)
    if args.run:
        files = [p for p in files if os.path.basename(p).startswith(args.run)]
    if not files:
        print(f"기록 없음: {args.dir}" + (f" (앞부분 '{args.run}')" if args.run else ""))
        return 2
    return report_one(files[-1], args)


if __name__ == "__main__":
    sys.exit(main())

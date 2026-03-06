#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import sys
import time
import json
import queue
import argparse
import threading
import urllib.request
import urllib.error
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple
import importlib.util
import re


THIS_DIR = Path(__file__).resolve().parent
DEFAULT_LOCATIONS = (THIS_DIR / "../config/location.yaml").resolve()
DEFAULT_ENV_FILE = (THIS_DIR / "../../.env").resolve()


# =========================
# YAML (간단 파서: locations 키만)
# =========================
def load_locations_yaml_simple(path: Path) -> Dict[str, Dict[str, float]]:
    locs: Dict[str, Dict[str, float]] = {}
    if not path.exists():
        return locs

    cur = None
    in_locations = False
    key_re = re.compile(r"^\s{2}([^:#]+)\s*:\s*$")
    val_re = re.compile(r"^\s{4}(x|y|w)\s*:\s*([-\d.eE]+)\s*$")
    loc_re = re.compile(r"^\s*locations\s*:\s*$")

    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if not in_locations:
                if loc_re.match(line):
                    in_locations = True
                continue

            mk = key_re.match(line)
            if mk:
                cur = mk.group(1).strip()
                locs[cur] = {}
                continue

            mv = val_re.match(line)
            if mv and cur:
                k = mv.group(1)
                v = float(mv.group(2))
                locs[cur][k] = v

    out: Dict[str, Dict[str, float]] = {}
    for k, v in locs.items():
        if all(t in v for t in ("x", "y", "w")):
            out[k] = {"x": v["x"], "y": v["y"], "w": v["w"]}
    return out


# =========================
# .env에서 OPENAI_API_KEY 읽기
# =========================
def read_env_openai_key(env_path: Path) -> str:
    if not env_path.exists():
        return ""
    for line in env_path.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        if s.startswith("OPENAI_API_KEY="):
            return s.split("=", 1)[1].strip().strip('"').strip("'")
    return ""


def load_openai_key(env_path: Path) -> str:
    k = os.environ.get("OPENAI_API_KEY", "").strip()
    if k:
        return k
    return read_env_openai_key(env_path)


# =========================
# TTS (gTTS + pygame) - 있으면 사용, 없으면 무시
# =========================
class GttsTTS:
    def __init__(self, lang: str = "ko", debug: bool = False):
        self.lang = lang
        self.debug = debug
        self._q: "queue.Queue[str]" = queue.Queue()
        self._stop = threading.Event()
        self._ok = False

        try:
            from gtts import gTTS  # noqa
            import pygame  # noqa
            self._ok = True
        except Exception as e:
            self._ok = False
            if self.debug:
                print(f"[DEBUG][TTS] disabled: {e}", file=sys.stderr)

        self._th = threading.Thread(target=self._worker, daemon=True)
        if self._ok:
            self._th.start()

    def say(self, text: str) -> None:
        if not self._ok:
            return
        t = (text or "").strip()
        if not t:
            return
        if len(t) > 260:
            t = t[:260] + "…"
        self._q.put(t)

    def stop(self) -> None:
        self._stop.set()
        try:
            self._q.put_nowait("")
        except Exception:
            pass

    def _worker(self) -> None:
        try:
            from gtts import gTTS
            import pygame
        except Exception:
            return

        try:
            pygame.mixer.init()
        except Exception as e:
            if self.debug:
                print(f"[DEBUG][TTS] pygame init failed: {e}", file=sys.stderr)
            return

        import tempfile

        while not self._stop.is_set():
            try:
                text = self._q.get(timeout=0.2)
            except Exception:
                continue
            if self._stop.is_set():
                break
            if not text:
                continue

            path = None
            try:
                fd, path = tempfile.mkstemp(prefix="gtts_", suffix=".mp3")
                os.close(fd)
                gTTS(text=text, lang=self.lang, slow=False).save(path)

                pygame.mixer.music.load(path)
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy() and (not self._stop.is_set()):
                    time.sleep(0.03)
                pygame.mixer.music.unload()
            except Exception as e:
                if self.debug:
                    print(f"[DEBUG][TTS] speak error: {e}", file=sys.stderr)
            finally:
                if path:
                    try:
                        os.remove(path)
                    except Exception:
                        pass


# =========================
# Hardware bridge 로더 (없으면 /button, /pull로 테스트 가능)
# =========================
def _load_hardware_key_bridge(debug: bool = False):
    candidates = [
        (THIS_DIR / "hardware/input_bridge.py").resolve(),
        (THIS_DIR / "../hardware/input_bridge.py").resolve(),
        (THIS_DIR / "../../hardware/input_bridge.py").resolve(),
    ]
    found = None
    for p in candidates:
        if p.exists():
            found = p
            break
    if found is None:
        return None

    spec = importlib.util.spec_from_file_location("input_bridge_local", str(found))
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore
    if not hasattr(module, "HardwareKeyBridge"):
        return None
    if debug:
        print(f"[INFO] Loaded HardwareKeyBridge from: {found}", file=sys.stderr)
    return module.HardwareKeyBridge


# =========================
# OpenAI Responses API (Structured Outputs)
# =========================
def _extract_output_text(resp_json: dict) -> str:
    out = resp_json.get("output", [])
    for item in out:
        if item.get("type") == "message" and item.get("role") == "assistant":
            for c in item.get("content", []):
                if c.get("type") == "output_text":
                    return c.get("text", "")
    return ""


PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "say": {"type": "string"},
        "action": {
            "type": "string",
            "enum": [
                "ask_again",
                "list",
                "propose",
                "cancel_confirm",
                "resume_propose",
                "change_dest",
                "abort_propose",
                "noop",
            ],
        },
        "destination": {"type": ["string", "null"]},
        "confidence": {"type": "number"},
        "need_button": {"type": "boolean"},
        "button_action": {"type": "string", "enum": ["none", "start", "resume", "abort"]},
    },
    "required": ["say", "action", "destination", "confidence", "need_button", "button_action"],
    "additionalProperties": False,
}


class OpenAIPlanner:
    def __init__(self, api_key: str, model: str, debug: bool = False):
        self.api_key = api_key
        self.model = model
        self.debug = debug
        self.base_url = "https://api.openai.com/v1"
        self._prev_id: Optional[str] = None

        self.instructions = (
            "너는 시각장애인 안내 로봇의 '장소 선택/재질문' 비서다.\n"
            "정답 후보는 locations 리스트뿐이다.\n"
            "사용자의 발화(user_text)가 들어오면 가장 그럴듯한 destination을 고르거나, 확신이 없으면 다시 질문해라.\n"
            "확실하다고 판단되면 propose로 하고, '버튼으로 확인' 절차를 반드시 요구해라(need_button=true).\n"
            "예: '이호'는 '2호'일 가능성이 크다.\n"
            "예: '7빼기2호' 같은 표현이 오면 산술적으로 5호가 있으면 5호를 제안해도 된다.\n"
            "사용자가 '목록/뭐가 있어/몇 호 있어' 같은 질문이면 list로.\n"
            "출력은 반드시 스키마 JSON만.\n"
        )

    def plan(self, context: dict) -> Tuple[Optional[dict], Optional[str]]:
        url = f"{self.base_url}/responses"
        payload: Dict[str, Any] = {
            "model": self.model,
            "instructions": self.instructions,
            "input": json.dumps(context, ensure_ascii=False),
            "max_output_tokens": 220,
            "text": {
                "format": {
                    "type": "json_schema",
                    "name": "location_plan",
                    "schema": PLAN_SCHEMA,
                    "strict": True
                }
            }
        }
        if self._prev_id:
            payload["previous_response_id"] = self._prev_id

        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            url=url,
            data=data,
            method="POST",
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
        )

        try:
            with urllib.request.urlopen(req, timeout=25) as resp:
                resp_json = json.loads(resp.read().decode("utf-8"))

            rid = resp_json.get("id")
            if isinstance(rid, str) and rid:
                self._prev_id = rid

            raw = _extract_output_text(resp_json).strip()
            if not raw:
                return None, "empty_output_text"
            return json.loads(raw), None

        except urllib.error.HTTPError as e:
            body = ""
            try:
                body = e.read().decode("utf-8", errors="ignore")
            except Exception:
                pass
            return None, f"HTTP {e.code}: {body[:300]}"
        except Exception as e:
            return None, f"{type(e).__name__}: {e}"


# =========================
# State + Events
# =========================
class State(Enum):
    LOCKED = auto()
    READY = auto()
    NAV = auto()
    PAUSED = auto()


@dataclass
class PendingConfirm:
    action: str                 # start/resume/abort
    destination: Optional[str]
    deadline: float


@dataclass
class Event:
    type: str                   # text/button/pull/arrive
    text: Optional[str] = None
    ts: float = 0.0


# =========================
# Main App
# =========================
class InterfaceApp:
    def __init__(
        self,
        *,
        locations_path: Path,
        env_path: Path,
        model: str,
        confirm_sec: float,
        ready_idle_sec: float,
        arrival_sec: float,   # 0이면 자동 도착 비활성
        no_tts: bool,
        no_hw: bool,
        port: str,
        baud: int,
        debug: bool,
    ):
        self.debug = debug
        self.state = State.LOCKED

        self.locations_path = locations_path
        self.locations = load_locations_yaml_simple(self.locations_path)
        self._loc_mtime = self.locations_path.stat().st_mtime if self.locations_path.exists() else 0.0

        self.confirm_sec = float(confirm_sec)
        self.ready_idle_sec = float(ready_idle_sec)

        # 도착 시뮬레이션
        self.arrival_sec = float(arrival_sec)
        self.nav_start_t: float = 0.0
        self.nav_elapsed: float = 0.0
        self.nav_target: Optional[str] = None

        # 버튼 더블-트리거(하드웨어/시리얼 특성)로 인한 "시작하자마자 종료 질문" 방지
        self._btn_guard_until: float = 0.0
        self._btn_guard_sec: float = 0.40

        self.pending_destination: Optional[str] = None
        self.pending: Optional[PendingConfirm] = None
        self.ready_last_activity = 0.0

        self.ev_q: "queue.Queue[Event]" = queue.Queue()
        self._stop = threading.Event()

        self.tts: Optional[GttsTTS] = None
        if not no_tts:
            self.tts = GttsTTS(lang="ko", debug=debug)

        api_key = load_openai_key(env_path)
        if not api_key:
            raise RuntimeError(f"OPENAI_API_KEY not found (env var or {env_path})")

        self.gpt = OpenAIPlanner(api_key=api_key, model=model, debug=debug)

        # GPT worker
        self._req_q: "queue.Queue[Tuple[Event, dict]]" = queue.Queue()
        self._res_q: "queue.Queue[Tuple[Event, Optional[dict], Optional[str]]]" = queue.Queue()
        self._inflight = False
        threading.Thread(target=self._gpt_worker, daemon=True).start()

        # stdin worker
        threading.Thread(target=self._stdin_worker, daemon=True).start()

        # hardware
        self.hw = None
        if not no_hw:
            HardwareKeyBridge = _load_hardware_key_bridge(debug=debug)
            if HardwareKeyBridge is not None:
                self.hw = HardwareKeyBridge(
                    on_o=lambda: self.ev_q.put(Event(type="button", ts=time.time())),
                    on_p=lambda: self.ev_q.put(Event(type="pull", ts=time.time())),
                    port=port,
                    baud=baud,
                )
                try:
                    self.hw.start()
                except Exception as e:
                    if self.debug:
                        print(f"[DEBUG] HW start failed: {e}", file=sys.stderr)
                    self.hw = None

    def destinations(self) -> List[str]:
        return list(self.locations.keys())

    def say(self, text: str) -> None:
        t = (text or "").strip()
        if not t:
            return
        print(f"[ROBOT] {t}")
        if self.tts:
            self.tts.say(t)

    def _state_log(self, reason: str) -> None:
        print(f"[STATE] {self.state.name} ({reason})")

    def _stdin_worker(self) -> None:
        # 테스트용: /button, /pull, /arrive 지원
        while not self._stop.is_set():
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.05)
                continue
            s = line.strip()
            if not s:
                continue
            if s == "/button":
                self.ev_q.put(Event(type="button", ts=time.time()))
            elif s == "/pull":
                self.ev_q.put(Event(type="pull", ts=time.time()))
            elif s == "/arrive":
                self.ev_q.put(Event(type="arrive", ts=time.time()))
            else:
                self.ev_q.put(Event(type="text", text=s, ts=time.time()))

    def _gpt_worker(self) -> None:
        while not self._stop.is_set():
            try:
                ev, ctx = self._req_q.get(timeout=0.2)
            except Exception:
                continue
            plan, err = self.gpt.plan(ctx)
            self._res_q.put((ev, plan, err))

    def _reload_locations_if_changed(self) -> None:
        if not self.locations_path.exists():
            return
        mt = self.locations_path.stat().st_mtime
        if mt <= self._loc_mtime:
            return
        self._loc_mtime = mt
        self.locations = load_locations_yaml_simple(self.locations_path)
        if self.pending_destination and self.pending_destination not in self.destinations():
            self.pending_destination = None
        if self.pending and self.pending.destination and self.pending.destination not in self.destinations():
            self.pending = None

    def _set_confirm(self, action: str, destination: Optional[str]) -> None:
        self.pending = PendingConfirm(action=action, destination=destination, deadline=time.time() + self.confirm_sec)

    def _clear_confirm(self) -> None:
        self.pending = None

    # NAV timer utils
    def _nav_timer_start(self) -> None:
        if self.arrival_sec <= 0:
            return
        if self.nav_start_t <= 0:
            self.nav_start_t = time.time()

    def _nav_timer_pause(self) -> None:
        if self.arrival_sec <= 0:
            return
        if self.nav_start_t > 0:
            self.nav_elapsed += (time.time() - self.nav_start_t)
            self.nav_start_t = 0.0

    def _nav_timer_reset(self) -> None:
        self.nav_start_t = 0.0
        self.nav_elapsed = 0.0
        self.nav_target = None

    def _nav_total_elapsed(self) -> float:
        total = self.nav_elapsed
        if self.nav_start_t > 0:
            total += (time.time() - self.nav_start_t)
        return total

    def _arrive_now(self, reason: str = "arrived_sim") -> None:
        dest = self.nav_target or self.pending_destination
        self.state = State.LOCKED
        self.pending_destination = None
        self._clear_confirm()
        self._nav_timer_reset()
        self._state_log(reason)
        if dest:
            self.say(f"{dest}에 도착했습니다.")
        else:
            self.say("목적지에 도착했습니다.")

    def _nav_arrival_tick(self) -> None:
        if self.arrival_sec <= 0:
            return
        if self.state != State.NAV:
            return
        if self.nav_start_t <= 0:
            return
        if self._nav_total_elapsed() >= self.arrival_sec:
            self._arrive_now("arrived_auto")

    def _confirm_timeout_tick(self) -> None:
        if not self.pending:
            return
        if time.time() < self.pending.deadline:
            return
        self.pending = None
        if self.state == State.READY:
            self.say("확인이 없어 취소했어요. 목적지를 다시 말씀해 주세요.")
            self.ready_last_activity = time.time()
        elif self.state == State.PAUSED:
            self.say("확인이 없어 계속 멈춰 있을게요. 괜찮으시면 다시 요청해 주세요.")
        elif self.state == State.NAV:
            # NAV에서는 확인 없으면 그냥 계속 진행(안전)
            self.say("확인이 없어 계속 이동을 유지할게요. 필요하면 다시 말씀해 주세요.")

    def _ready_idle_tick(self) -> None:
        if self.state != State.READY:
            return
        if self.pending:
            return
        if self.ready_last_activity <= 0:
            return
        if (time.time() - self.ready_last_activity) >= self.ready_idle_sec:
            self.state = State.LOCKED
            self.pending_destination = None
            self._clear_confirm()
            self._nav_timer_reset()
            self._state_log("idle_timeout")
            self.say("오래 응답이 없어 안내를 종료할게요. 필요하면 버튼을 눌러 주세요.")

    # =========================
    # 핵심 수정: 버튼 동작을 상태별로 “안내 문구와 일치”하게
    # =========================
    def _handle_button(self) -> None:
        now = time.time()
        if now < self._btn_guard_until:
            return

        # 1) 확인 대기(PendingConfirm)가 있으면: 버튼은 무조건 "확인"이다.
        if self.pending:
            pc = self.pending
            self.pending = None
            self._btn_guard_until = now + self._btn_guard_sec

            if pc.action == "start" and self.state == State.READY and pc.destination:
                self.state = State.NAV
                self.pending_destination = pc.destination
                self.nav_target = pc.destination
                self.nav_elapsed = 0.0
                self.nav_start_t = time.time() if self.arrival_sec > 0 else 0.0
                self._state_log("confirm_start")
                self.say(f"확인했습니다. {pc.destination}로 안내를 시작하겠습니다.")
                return

            if pc.action == "resume" and self.state == State.PAUSED:
                self.state = State.NAV
                self._nav_timer_start()
                self._state_log("confirm_resume")
                self.say("확인했습니다. 다시 이동하겠습니다.")
                return

            if pc.action == "abort" and self.state != State.LOCKED:
                self.state = State.LOCKED
                self.pending_destination = None
                self._nav_timer_reset()
                self._state_log("confirm_abort")
                self.say("확인했습니다. 안내를 종료합니다.")
                return

            # 상태가 맞지 않는 확인이면, 조용히 무시하지 말고 안내
            self.say("지금은 그 확인을 적용할 수 없어요. 다시 말씀해 주세요.")
            return

        # 2) LOCKED -> READY : 버튼은 “시작”
        if self.state == State.LOCKED:
            self.state = State.READY
            self.pending_destination = None
            self._clear_confirm()
            self._nav_timer_reset()
            self.ready_last_activity = time.time()
            self._btn_guard_until = now + self._btn_guard_sec
            self._state_log("button_start")
            self.say("어디로 안내해 드릴까요?")
            return

        # 3) READY에서 버튼(확인 대기 없음):
        #    - 절대 “종료”로 가지 않는다 (여기가 기존 혼선의 핵심)
        if self.state == State.READY:
            self._btn_guard_until = now + self._btn_guard_sec
            if self._inflight:
                self.say("목적지를 확인 중이에요. 잠시만 기다려 주세요.")
                return
            self.say("목적지를 말씀해 주세요. 예: '2호', '편의점'.")
            return

        # 4) PAUSED에서 버튼(확인 대기 없음): “재개 제안 → 버튼 확인”
        if self.state == State.PAUSED:
            self._btn_guard_until = now + self._btn_guard_sec
            self._set_confirm("resume", None)
            self.say(f"다시 이동할까요? 맞으면 {int(self.confirm_sec)}초 안에 버튼을 눌러 주세요.")
            return

        # 5) NAV에서 버튼(확인 대기 없음): “종료 제안 → 버튼 확인(2단계)”
        if self.state == State.NAV:
            self._btn_guard_until = now + self._btn_guard_sec
            self._set_confirm("abort", None)
            self.say(f"안내를 종료할까요? 종료하려면 {int(self.confirm_sec)}초 안에 버튼을 한 번 더 눌러 주세요.")
            return

    def _handle_pull(self) -> None:
        if self.state == State.NAV:
            self.state = State.PAUSED
            self._nav_timer_pause()
            self._clear_confirm()
            self._state_log("pull_pause")
            self.say("멈췄습니다. 무슨 일 있으신가요? 원하시는 요청을 말씀해 주세요.")
            return

    def _dispatch_to_gpt(self, ev: Event) -> None:
        if self._inflight:
            return

        ctx = {
            "state": self.state.name,
            "locations": self.destinations(),
            "user_text": ev.text if ev.type == "text" else None,
            "pending_destination": self.pending_destination,
            "nav_target": self.nav_target,
            "pending_confirm": None if not self.pending else {
                "action": self.pending.action,
                "destination": self.pending.destination,
                "time_left": round(max(0.0, self.pending.deadline - time.time()), 2)
            }
        }
        self._req_q.put((ev, ctx))
        self._inflight = True

    def _apply_plan(self, plan: dict) -> None:
        say = (plan.get("say") or "").strip()
        action = plan.get("action", "noop")
        dest = plan.get("destination", None)
        need_button = bool(plan.get("need_button", False))
        button_action = plan.get("button_action", "none")

        if say:
            self.say(say)

        if action == "list":
            d = self.destinations()
            if (not say) or (not any(name in say for name in d)):
                self.say("여기에는 " + (", ".join(d) if d else "(없음)") + "가 있습니다. 어떤 곳을 원하시나요?")
            return

        if action == "cancel_confirm":
            self._clear_confirm()
            return

        # READY: 목적지 제안 → 버튼 확인
        if action == "propose" and self.state == State.READY:
            if dest and dest in self.destinations():
                self.pending_destination = dest
                self.ready_last_activity = time.time()
                if need_button and button_action == "start":
                    self._set_confirm("start", dest)
                    # 안내 문구와 전환이 항상 일치하도록 강제 보강
                    if "버튼" not in say:
                        self.say(f"{dest}로 안내하려면 {int(self.confirm_sec)}초 안에 버튼을 눌러 주세요.")
            return

        # PAUSED: 재개 제안 → 버튼 확인
        if action == "resume_propose" and self.state == State.PAUSED:
            if need_button and button_action == "resume":
                self._set_confirm("resume", None)
                if "버튼" not in say:
                    self.say(f"다시 이동하려면 {int(self.confirm_sec)}초 안에 버튼을 눌러 주세요.")
            return

        # 목적지 변경: READY로 유도 (이동 타이머 리셋)
        if action == "change_dest":
            if self.state in (State.NAV, State.PAUSED):
                self.state = State.READY
                self.pending_destination = None
                self._clear_confirm()
                self._nav_timer_reset()
                self.ready_last_activity = time.time()
                self._state_log("change_dest")
                if "어디로" not in say:
                    self.say("그럼 어디로 안내해 드릴까요?")
            return

        # 종료 제안 → 버튼 확인
        if action == "abort_propose" and self.state != State.LOCKED:
            if need_button and button_action == "abort":
                self._set_confirm("abort", None)
                if "버튼" not in say:
                    self.say(f"종료하려면 {int(self.confirm_sec)}초 안에 버튼을 눌러 주세요.")
            return

        return

    def run(self) -> None:
        last_reload = 0.0

        while not self._stop.is_set():
            now = time.time()

            if now - last_reload > 0.5:
                self._reload_locations_if_changed()
                last_reload = now

            self._confirm_timeout_tick()
            self._ready_idle_tick()
            self._nav_arrival_tick()

            # 이벤트 처리
            try:
                ev = self.ev_q.get_nowait()
            except Exception:
                ev = None

            if ev is not None:
                if ev.type == "button":
                    self._handle_button()

                elif ev.type == "pull":
                    self._handle_pull()

                elif ev.type == "arrive":
                    if self.state == State.NAV:
                        self._arrive_now("arrived_manual")

                elif ev.type == "text":
                    if ev.text in ("q", "quit", "exit"):
                        self._stop.set()
                        break

                    if self.state == State.READY:
                        self.ready_last_activity = time.time()

                    self._dispatch_to_gpt(ev)

            # GPT 결과 반영
            try:
                _done_ev, plan, err = self._res_q.get_nowait()
                self._inflight = False
                if plan is None:
                    if self.debug:
                        print(f"[GPT-ERR] {err}", file=sys.stderr)
                    if self.state == State.READY:
                        self.say("지금은 연결이 불안정해요. 다시 한 번 말씀해 주세요. '목록'이라고 하면 가능한 목적지를 알려드릴게요.")
                    elif self.state == State.PAUSED:
                        self.say("연결이 불안정해요. 원하시면 '다시 가줘' 또는 '목적지 바꿔'처럼 말해 주세요.")
                else:
                    self._apply_plan(plan)
            except Exception:
                pass

            time.sleep(0.02)

        try:
            if self.hw:
                self.hw.stop()
        except Exception:
            pass
        if self.tts:
            self.tts.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--locations", default=str(DEFAULT_LOCATIONS))
    ap.add_argument("--env", default=str(DEFAULT_ENV_FILE))
    ap.add_argument("--model", default=os.environ.get("OPENAI_MODEL", "gpt-4o-mini"))

    ap.add_argument("--confirm-sec", type=float, default=15.0)
    ap.add_argument("--ready-idle-sec", type=float, default=15.0)
    ap.add_argument("--arrival-sec", type=float, default=20.0)  # 0이면 자동도착 OFF

    ap.add_argument("--no-tts", action="store_true")
    ap.add_argument("--no-hw", action="store_true")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    app = InterfaceApp(
        locations_path=Path(args.locations).resolve(),
        env_path=Path(args.env).resolve(),
        model=args.model,
        confirm_sec=args.confirm_sec,
        ready_idle_sec=args.ready_idle_sec,
        arrival_sec=args.arrival_sec,
        no_tts=args.no_tts,
        no_hw=args.no_hw,
        port=args.port,
        baud=args.baud,
        debug=args.debug,
    )
    app.run()


if __name__ == "__main__":
    main()

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
from typing import Optional, Dict, Any, List, Tuple, Callable
import importlib.util
import re

import speech_recognition as sr


THIS_DIR = Path(__file__).resolve().parent
DEFAULT_LOCATIONS = (THIS_DIR / "../config/location.yaml").resolve()
DEFAULT_ENV_FILE = (THIS_DIR / "../../.env").resolve()


# =========================
# YAML
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
# env
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
# Audio gate
# =========================
class AudioGate:
    """
    TTS/삑음/안내음 재생 중 또는 끝난 직후 마이크를 잠깐 막는 게이트.
    """
    def __init__(self, post_audio_cooldown_sec: float = 0.7):
        self.post_audio_cooldown_sec = float(post_audio_cooldown_sec)
        self._lock = threading.Lock()
        self._blocked_until = 0.0

    def block_for(self, sec: float) -> None:
        with self._lock:
            self._blocked_until = max(self._blocked_until, time.time() + max(0.0, sec))

    def block_audio(self, playback_sec: float) -> None:
        self.block_for(playback_sec + self.post_audio_cooldown_sec)

    def is_open(self) -> bool:
        with self._lock:
            return time.time() >= self._blocked_until

    def seconds_left(self) -> float:
        with self._lock:
            return max(0.0, self._blocked_until - time.time())


# =========================
# Beep
# =========================
class Beeper:
    """
    콘솔벨('\a') + 가능하면 pygame tone 파일 재생.
    """
    def __init__(self, audio_gate: Optional[AudioGate] = None, debug: bool = False):
        self.audio_gate = audio_gate
        self.debug = debug
        self._ok = False
        self._pygame = None
        self._tmpdir = None
        self._cache: Dict[Tuple[int, int], str] = {}

        try:
            import pygame
            import wave
            import math
            import struct
            import tempfile
            self._pygame = pygame
            self._wave = wave
            self._math = math
            self._struct = struct
            self._tempfile = tempfile
            self._ok = True
            try:
                pygame.mixer.init()
            except Exception:
                self._ok = False
        except Exception as e:
            self._ok = False
            if self.debug:
                print(f"[DEBUG][BEEP] disabled: {e}", file=sys.stderr)

    def _wav_path(self, freq_hz: int, duration_ms: int) -> Optional[str]:
        if not self._ok:
            return None
        key = (freq_hz, duration_ms)
        if key in self._cache:
            return self._cache[key]

        try:
            if self._tmpdir is None:
                self._tmpdir = self._tempfile.mkdtemp(prefix="beep_cache_")
            path = os.path.join(self._tmpdir, f"beep_{freq_hz}_{duration_ms}.wav")
            framerate = 44100
            amplitude = 12000
            nframes = int(framerate * (duration_ms / 1000.0))

            with self._wave.open(path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(framerate)
                frames = bytearray()
                for i in range(nframes):
                    t = i / framerate
                    sample = int(amplitude * self._math.sin(2.0 * self._math.pi * freq_hz * t))
                    frames.extend(self._struct.pack("<h", sample))
                wf.writeframes(bytes(frames))

            self._cache[key] = path
            return path
        except Exception as e:
            if self.debug:
                print(f"[DEBUG][BEEP] wav gen error: {e}", file=sys.stderr)
            return None

    def beep(self, freq_hz: int = 1200, duration_ms: int = 120) -> None:
        sec = duration_ms / 1000.0
        if self.audio_gate:
            self.audio_gate.block_audio(sec)

        # 콘솔벨
        try:
            print("\a", end="", flush=True)
        except Exception:
            pass

        if not self._ok:
            return

        try:
            path = self._wav_path(freq_hz, duration_ms)
            if path is None:
                return
            self._pygame.mixer.music.load(path)
            self._pygame.mixer.music.play()
            while self._pygame.mixer.music.get_busy():
                time.sleep(0.01)
            self._pygame.mixer.music.unload()
        except Exception as e:
            if self.debug:
                print(f"[DEBUG][BEEP] play error: {e}", file=sys.stderr)

    def start_listen_beep(self) -> None:
        self.beep(freq_hz=1350, duration_ms=110)

    def end_listen_beep(self) -> None:
        self.beep(freq_hz=900, duration_ms=90)


# =========================
# TTS
# =========================
class GttsTTS:
    def __init__(self, lang: str = "ko", debug: bool = False, audio_gate: Optional[AudioGate] = None):
        self.lang = lang
        self.debug = debug
        self.audio_gate = audio_gate
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

    @staticmethod
    def estimate_duration_sec(text: str) -> float:
        chars = len((text or "").strip())
        return min(9.0, max(1.2, 0.085 * chars + 0.9))

    def say(self, text: str) -> float:
        if not self._ok:
            return 0.0
        t = (text or "").strip()
        if not t:
            return 0.0
        if len(t) > 260:
            t = t[:260] + "…"

        estimated = self.estimate_duration_sec(t)
        if self.audio_gate:
            self.audio_gate.block_audio(estimated)

        self._q.put(t)
        return estimated

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
# Hardware bridge
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
# OpenAI Responses API
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
                "change_dest_request",
                "change_dest_propose",
                "abort_propose",
                "noop",
            ],
        },
        "destination": {"type": ["string", "null"]},
        "confidence": {"type": "number"},
        "need_button": {"type": "boolean"},
        "button_action": {
            "type": "string",
            "enum": ["none", "start", "resume", "abort", "change_ready", "change_start"],
        },
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
            "너는 시각장애인 안내 로봇의 장소 선택/재질문 비서다.\n"
            "정답 후보 목적지는 locations 리스트뿐이다.\n"
            "반드시 스키마 JSON만 출력해라.\n"
            "\n"
            "[공통 규칙]\n"
            "- 사용자의 발화(user_text)를 보고 action, destination, need_button, button_action을 정해라.\n"
            "- 확실한 목적지가 있으면 destination에 넣어라.\n"
            "- 버튼 확인이 필요한 행동이면 need_button=true로 하라.\n"
            "- say에는 사용자에게 실제로 말할 문장을 한국어로 자연스럽게 써라.\n"
            "- 잡음/혼잣말/짧은 중얼거림처럼 의도가 불분명하면 ask_again 또는 noop을 사용해라.\n"
            "\n"
            "[READY 상태]\n"
            "- 목적지를 확실히 이해하면 action='propose', need_button=true, button_action='start'.\n"
            "- 사용자가 목록/뭐가 있어/몇 호 있어 라고 물으면 action='list'.\n"
            "- 애매하면 action='ask_again'.\n"
            "\n"
            "[PAUSED 상태]\n"
            "- '다시 가', '계속 가', '재개' 계열이면 action='resume_propose', need_button=true, button_action='resume'.\n"
            "- '종료해줘', '그만해줘', '안내 끝' 계열이면 action='abort_propose', need_button=true, button_action='abort'.\n"
            "- '목적지 바꿔', '다른 곳 가고 싶어'처럼 바꾸고 싶지만 새 목적지가 없으면 "
            "action='change_dest_request', destination=null, need_button=true, button_action='change_ready'.\n"
            "- '3호 가자', '편의점으로 바꿔', '화장실 가고 싶어'처럼 새 목적지가 같이 있으면 "
            "action='change_dest_propose', destination에 그 장소, need_button=true, button_action='change_start'.\n"
            "- 목적지가 locations에 없거나 불명확하면 action='ask_again'.\n"
        )

    def plan(self, context: dict) -> Tuple[Optional[dict], Optional[str]]:
        url = f"{self.base_url}/responses"
        payload: Dict[str, Any] = {
            "model": self.model,
            "instructions": self.instructions,
            "input": json.dumps(context, ensure_ascii=False),
            "max_output_tokens": 260,
            "text": {
                "format": {
                    "type": "json_schema",
                    "name": "location_plan",
                    "schema": PLAN_SCHEMA,
                    "strict": True,
                }
            },
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
# Mic bridge
# =========================
class MicTextBridge:
    """
    마이크 입력:
    - TTS/삑음 중에는 닫힘
    - 시작 삑이 난 뒤 수집
    - 끝나면 종료 삑
    - 중복/노이즈/너무 짧은 텍스트 필터
    """
    def __init__(
        self,
        on_text: Callable[[str], None],
        is_listen_allowed: Callable[[], bool],
        audio_gate: AudioGate,
        beeper: Beeper,
        debug: bool = False,
        timeout: float = 0.8,
        phrase_time_limit: float = 5.0,
        pause_threshold: float = 1.0,
        energy_threshold: int = 350,
        ambient_duration: float = 1.0,
        min_text_chars: int = 2,
        dedup_window_sec: float = 2.2,
        listen_open_delay_sec: float = 0.35,
    ):
        self.on_text = on_text
        self.is_listen_allowed = is_listen_allowed
        self.audio_gate = audio_gate
        self.beeper = beeper
        self.debug = debug

        self.timeout = timeout
        self.phrase_time_limit = phrase_time_limit
        self.pause_threshold = pause_threshold
        self.energy_threshold = energy_threshold
        self.ambient_duration = ambient_duration
        self.min_text_chars = min_text_chars
        self.dedup_window_sec = dedup_window_sec
        self.listen_open_delay_sec = listen_open_delay_sec

        self._stop = threading.Event()
        self._th = threading.Thread(target=self._worker, daemon=True)

        self._last_text = ""
        self._last_text_ts = 0.0
        self._listen_session_armed = False

    def start(self):
        self._th.start()

    def stop(self):
        self._stop.set()

    @staticmethod
    def normalize_text(s: str) -> str:
        s = (s or "").strip()
        s = re.sub(r"\s+", "", s)
        return s

    def _is_noise_text(self, text: str) -> bool:
        t = self.normalize_text(text)
        if len(t) < self.min_text_chars:
            return True
        bads = {
            "음", "어", "아", "어어", "음음", "그", "저", "흠",
            "응", "네", "예", "아니", "뭐", "잠깐", "잠시",
        }
        return t in bads

    def _is_duplicate(self, text: str) -> bool:
        now = time.time()
        t = self.normalize_text(text)
        if t == self._last_text and (now - self._last_text_ts) <= self.dedup_window_sec:
            return True
        self._last_text = t
        self._last_text_ts = now
        return False

    def _worker(self):
        r = sr.Recognizer()
        r.energy_threshold = self.energy_threshold
        r.dynamic_energy_threshold = True
        r.pause_threshold = self.pause_threshold
        r.non_speaking_duration = 0.5

        try:
            mic = sr.Microphone()
        except Exception as e:
            if self.debug:
                print(f"[DEBUG][MIC] microphone open failed: {e}", file=sys.stderr)
            return

        with mic as source:
            try:
                r.adjust_for_ambient_noise(source, duration=self.ambient_duration)
                if self.debug:
                    print(f"[DEBUG][MIC] ambient calibrated threshold={r.energy_threshold:.1f}", file=sys.stderr)
            except Exception as e:
                if self.debug:
                    print(f"[DEBUG][MIC] ambient calibration failed: {e}", file=sys.stderr)

        while not self._stop.is_set():
            if not self.is_listen_allowed():
                self._listen_session_armed = False
                time.sleep(0.05)
                continue

            if not self.audio_gate.is_open():
                self._listen_session_armed = False
                time.sleep(0.03)
                continue

            # 처음 열리는 시점에만 시작 삑
            if not self._listen_session_armed:
                time.sleep(self.listen_open_delay_sec)
                if self._stop.is_set() or (not self.is_listen_allowed()) or (not self.audio_gate.is_open()):
                    continue
                self.beeper.start_listen_beep()
                self._listen_session_armed = True

            try:
                with mic as source:
                    audio = r.listen(
                        source,
                        timeout=self.timeout,
                        phrase_time_limit=self.phrase_time_limit,
                    )
            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                if self.debug:
                    print(f"[DEBUG][MIC] listen error: {e}", file=sys.stderr)
                time.sleep(0.15)
                continue

            # 수집 끝 삑
            self.beeper.end_listen_beep()

            try:
                text = r.recognize_google(audio, language="ko-KR").strip()
                if self.debug and text:
                    print(f"[MIC] {text}", file=sys.stderr)

                if not text:
                    self._listen_session_armed = False
                    continue
                if self._is_noise_text(text):
                    self._listen_session_armed = False
                    continue
                if self._is_duplicate(text):
                    self._listen_session_armed = False
                    continue

                self.on_text(text)
            except sr.UnknownValueError:
                pass
            except Exception as e:
                if self.debug:
                    print(f"[DEBUG][MIC] STT error: {e}", file=sys.stderr)
            finally:
                self._listen_session_armed = False


# =========================
# State / Event
# =========================
class State(Enum):
    LOCKED = auto()
    READY = auto()
    NAV = auto()
    PAUSED = auto()


@dataclass
class PendingConfirm:
    action: str  # start/resume/abort/change_ready/change_start
    destination: Optional[str]
    deadline: float


@dataclass
class Event:
    type: str   # text/button/pull/arrive
    text: Optional[str] = None
    ts: float = 0.0


# =========================
# Main app
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
        arrival_sec: float,
        no_tts: bool,
        no_hw: bool,
        no_mic: bool,
        port: str,
        baud: int,
        mic_timeout: float,
        mic_phrase_time_limit: float,
        mic_pause_threshold: float,
        mic_energy_threshold: int,
        post_audio_cooldown_sec: float,
        listen_open_delay_sec: float,
        debug: bool,
    ):
        self.debug = debug
        self.state = State.LOCKED

        self.locations_path = locations_path
        self.locations = load_locations_yaml_simple(self.locations_path)
        self._loc_mtime = self.locations_path.stat().st_mtime if self.locations_path.exists() else 0.0

        # 시간 기준
        self.confirm_sec = float(confirm_sec)
        self.ready_idle_sec = float(ready_idle_sec)
        self.arrival_sec = float(arrival_sec)
        self.listen_open_delay_sec = float(listen_open_delay_sec)

        # NAV 경과시간
        self.nav_start_t: float = 0.0
        self.nav_elapsed: float = 0.0
        self.nav_target: Optional[str] = None

        # 버튼 연속입력 보호
        self._btn_guard_until: float = 0.0
        self._btn_guard_sec: float = 0.40

        self.pending_destination: Optional[str] = None
        self.pending: Optional[PendingConfirm] = None
        self.ready_last_activity = 0.0

        # 말 끝난 뒤 특정 작업 예약
        self._scheduled_tasks: List[Tuple[float, Callable[[], None]]] = []

        self.ev_q: "queue.Queue[Event]" = queue.Queue()
        self._stop = threading.Event()

        self.audio_gate = AudioGate(post_audio_cooldown_sec=post_audio_cooldown_sec)
        self.beeper = Beeper(audio_gate=self.audio_gate, debug=debug)

        self.tts: Optional[GttsTTS] = None
        if not no_tts:
            self.tts = GttsTTS(lang="ko", debug=debug, audio_gate=self.audio_gate)

        api_key = load_openai_key(env_path)
        if not api_key:
            raise RuntimeError(f"OPENAI_API_KEY not found (env var or {env_path})")

        self.gpt = OpenAIPlanner(api_key=api_key, model=model, debug=debug)

        self._req_q: "queue.Queue[Tuple[Event, dict]]" = queue.Queue()
        self._res_q: "queue.Queue[Tuple[Event, Optional[dict], Optional[str]]]" = queue.Queue()
        self._inflight = False
        threading.Thread(target=self._gpt_worker, daemon=True).start()

        # stdin 테스트용
        threading.Thread(target=self._stdin_worker, daemon=True).start()

        # 마이크
        self.mic = None
        if not no_mic:
            self.mic = MicTextBridge(
                on_text=lambda text: self.ev_q.put(Event(type="text", text=text, ts=time.time())),
                is_listen_allowed=self._is_mic_listen_allowed,
                audio_gate=self.audio_gate,
                beeper=self.beeper,
                debug=debug,
                timeout=mic_timeout,
                phrase_time_limit=mic_phrase_time_limit,
                pause_threshold=mic_pause_threshold,
                energy_threshold=mic_energy_threshold,
                ambient_duration=1.0,
                min_text_chars=2,
                dedup_window_sec=2.2,
                listen_open_delay_sec=listen_open_delay_sec,
            )
            try:
                self.mic.start()
            except Exception as e:
                if self.debug:
                    print(f"[DEBUG] MIC start failed: {e}", file=sys.stderr)
                self.mic = None

        # 하드웨어 버튼/pull
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

    # -----------------
    # basic helpers
    # -----------------
    def destinations(self) -> List[str]:
        return list(self.locations.keys())

    def say(self, text: str) -> float:
        t = (text or "").strip()
        if not t:
            return 0.0
        print(f"[ROBOT] {t}")
        if self.tts:
            return self.tts.say(t)
        return 0.0

    def say_and_then(self, text: str, cb: Optional[Callable[[], None]] = None, extra_delay: float = 0.0) -> None:
        dur = self.say(text)
        total_delay = dur + self.audio_gate.post_audio_cooldown_sec + max(0.0, extra_delay)
        if cb is not None:
            self._scheduled_tasks.append((time.time() + total_delay, cb))

    def _run_scheduled_tasks(self) -> None:
        if not self._scheduled_tasks:
            return
        now = time.time()
        remaining = []
        for when, cb in self._scheduled_tasks:
            if now >= when:
                try:
                    cb()
                except Exception as e:
                    if self.debug:
                        print(f"[DEBUG] scheduled task error: {e}", file=sys.stderr)
            else:
                remaining.append((when, cb))
        self._scheduled_tasks = remaining

    def _state_log(self, reason: str) -> None:
        print(f"[STATE] {self.state.name} ({reason})")

    def _is_mic_listen_allowed(self) -> bool:
        # 실사용 안정성을 위해 READY/PAUSED 위주로 듣기
        return self.state in (State.READY, State.PAUSED) and (not self._inflight)

    # -----------------
    # stdin worker
    # -----------------
    def _stdin_worker(self) -> None:
        """
        테스트용:
        /button
        /pull
        /arrive
        그 외 텍스트
        """
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

    # -----------------
    # GPT worker
    # -----------------
    def _gpt_worker(self) -> None:
        while not self._stop.is_set():
            try:
                ev, ctx = self._req_q.get(timeout=0.2)
            except Exception:
                continue
            plan, err = self.gpt.plan(ctx)
            self._res_q.put((ev, plan, err))

    # -----------------
    # reload locations
    # -----------------
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
        if self.nav_target and self.nav_target not in self.destinations():
            self.nav_target = None

    # -----------------
    # confirm helpers
    # -----------------
    def _set_confirm(self, action: str, destination: Optional[str]) -> None:
        self.pending = PendingConfirm(
            action=action,
            destination=destination,
            deadline=time.time() + self.confirm_sec
        )

    def _clear_confirm(self) -> None:
        self.pending = None

    def _set_confirm_after_prompt(self, action: str, destination: Optional[str], prompt_text: str) -> None:
        def _cb():
            self._set_confirm(action, destination)
        self.say_and_then(prompt_text, cb=_cb, extra_delay=self.listen_open_delay_sec)

    # -----------------
    # NAV timer
    # -----------------
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

    # -----------------
    # timeout들
    # -----------------
    def _confirm_timeout_tick(self) -> None:
        if not self.pending:
            return
        if time.time() < self.pending.deadline:
            return

        expired = self.pending
        self.pending = None

        if self.state == State.READY:
            self.say_and_then("확인이 없어 취소했어요. 목적지를 다시 말씀해 주세요.")
            self.ready_last_activity = time.time()
            return

        if self.state == State.PAUSED:
            if expired.action == "change_ready":
                self.say_and_then("확인이 없어 목적지 변경을 취소했어요. 계속 멈춰 있을게요.")
            elif expired.action == "change_start":
                self.say_and_then("확인이 없어 새 목적지 변경을 취소했어요. 계속 멈춰 있을게요.")
            elif expired.action == "abort":
                self.say_and_then("확인이 없어 종료를 취소했어요. 계속 멈춰 있을게요.")
            elif expired.action == "resume":
                self.say_and_then("확인이 없어 재개를 취소했어요. 계속 멈춰 있을게요.")
            else:
                self.say_and_then("확인이 없어 계속 멈춰 있을게요.")
            return

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

    def _nav_arrival_tick(self) -> None:
        if self.arrival_sec <= 0:
            return
        if self.state != State.NAV:
            return
        if self.nav_start_t <= 0:
            return
        if self._nav_total_elapsed() >= self.arrival_sec:
            self._arrive_now("arrived_auto")

    # -----------------
    # 상태 전이 헬퍼
    # -----------------
    def _go_ready_for_new_destination(self) -> None:
        self.state = State.READY
        self.pending_destination = None
        self._clear_confirm()
        self._nav_timer_reset()
        self.ready_last_activity = time.time()
        self._state_log("confirm_change_ready")
        self.say_and_then("확인했습니다. 새로운 목적지를 말씀해 주세요.")

    def _go_nav_with_destination(self, dest: str, reason: str) -> None:
        self.state = State.NAV
        self.pending_destination = dest
        self.nav_target = dest
        self.nav_elapsed = 0.0
        self.nav_start_t = time.time() if self.arrival_sec > 0 else 0.0
        self._state_log(reason)
        self.say(f"확인했습니다. 목적지를 {dest}로 변경하고 안내를 시작하겠습니다.")

    def _go_paused(self, reason: str, say_text: Optional[str] = None) -> None:
        self.state = State.PAUSED
        self._nav_timer_pause()
        self._clear_confirm()
        self._state_log(reason)
        if say_text:
            self.say_and_then(say_text)

    # -----------------
    # 버튼
    # -----------------
    def _handle_button(self) -> None:
        now = time.time()
        if now < self._btn_guard_until:
            return

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

            if pc.action == "change_ready" and self.state == State.PAUSED:
                self._go_ready_for_new_destination()
                return

            if pc.action == "change_start" and self.state == State.PAUSED and pc.destination:
                self._go_nav_with_destination(pc.destination, "confirm_change_start")
                return

            if pc.action == "abort" and self.state == State.PAUSED:
                self.state = State.LOCKED
                self.pending_destination = None
                self._clear_confirm()
                self._nav_timer_reset()
                self._state_log("confirm_abort")
                self.say("확인했습니다. 안내를 종료합니다.")
                return

            self.say("지금은 그 확인을 적용할 수 없어요.")
            return

        if self.state == State.LOCKED:
            self.state = State.READY
            self.pending_destination = None
            self._clear_confirm()
            self._nav_timer_reset()
            self.ready_last_activity = time.time()
            self._btn_guard_until = now + self._btn_guard_sec
            self._state_log("button_start")
            self.say_and_then("어디로 안내해 드릴까요?")
            return

        if self.state == State.READY:
            self._btn_guard_until = now + self._btn_guard_sec
            if self._inflight:
                self.say("목적지를 확인 중이에요. 잠시만 기다려 주세요.")
                return
            self.say_and_then("목적지를 말씀해 주세요.")
            return

        if self.state == State.PAUSED:
            self._btn_guard_until = now + self._btn_guard_sec
            self._set_confirm_after_prompt(
                "resume",
                None,
                f"다시 이동할까요? 맞으면 {int(self.confirm_sec)}초 안에 버튼을 눌러 주세요."
            )
            return

        if self.state == State.NAV:
            self._btn_guard_until = now + self._btn_guard_sec
            self._go_paused("button_pause", "일시정지했습니다. 원하시는 요청을 말씀해 주세요.")
            return

    # -----------------
    # pull
    # -----------------
    def _handle_pull(self) -> None:
        if self.state == State.NAV:
            self._go_paused("pull_pause", "멈췄습니다. 원하시는 요청을 말씀해 주세요.")
            return

    # -----------------
    # GPT
    # -----------------
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
                "time_left": round(max(0.0, self.pending.deadline - time.time()), 2),
            },
        }
        self._req_q.put((ev, ctx))
        self._inflight = True

    def _apply_plan(self, plan: dict) -> None:
        say = (plan.get("say") or "").strip()
        action = plan.get("action", "noop")
        dest = plan.get("destination", None)
        need_button = bool(plan.get("need_button", False))
        button_action = plan.get("button_action", "none")

        if action == "noop":
            if say:
                self.say_and_then(say)
            return

        if action == "list":
            d = self.destinations()
            if say:
                self.say_and_then(say)
            else:
                self.say_and_then("여기에는 " + (", ".join(d) if d else "(없음)") + "가 있습니다. 어떤 곳을 원하시나요?")
            self.ready_last_activity = time.time()
            return

        if action == "cancel_confirm":
            self._clear_confirm()
            if say:
                self.say_and_then(say)
            return

        if action == "ask_again":
            if self.state == State.READY:
                self.ready_last_activity = time.time()
                self.say_and_then(say if say else "목적지를 다시 말씀해 주세요.")
            elif self.state == State.PAUSED:
                self.say_and_then(say if say else "다시 말씀해 주세요. 예: 다시 가줘, 목적지 바꿔, 종료해줘.")
            return

        if action == "propose" and self.state == State.READY:
            if dest and dest in self.destinations() and need_button and button_action == "start":
                self.pending_destination = dest
                self.ready_last_activity = time.time()
                prompt = say if say else f"{dest}로 안내할까요? 맞으면 {int(self.confirm_sec)}초 안에 버튼을 눌러 주세요."
                self._set_confirm_after_prompt("start", dest, prompt)
            return

        if action == "resume_propose" and self.state == State.PAUSED:
            if need_button and button_action == "resume":
                prompt = say if say else f"다시 이동할까요? 맞으면 {int(self.confirm_sec)}초 안에 버튼을 눌러 주세요."
                self._set_confirm_after_prompt("resume", None, prompt)
            return

        if action == "change_dest_request" and self.state == State.PAUSED:
            if need_button and button_action == "change_ready":
                prompt = say if say else f"목적지를 변경하시겠습니까? 맞으면 {int(self.confirm_sec)}초 안에 버튼을 눌러 주세요."
                self._set_confirm_after_prompt("change_ready", None, prompt)
            return

        if action == "change_dest_propose" and self.state == State.PAUSED:
            if dest and dest in self.destinations() and need_button and button_action == "change_start":
                prompt = say if say else f"{dest}로 목적지를 변경할까요? 맞으면 {int(self.confirm_sec)}초 안에 버튼을 눌러 주세요."
                self._set_confirm_after_prompt("change_start", dest, prompt)
            else:
                self.say_and_then("변경할 목적지를 다시 말씀해 주세요.")
            return

        if action == "abort_propose" and self.state == State.PAUSED:
            if need_button and button_action == "abort":
                prompt = say if say else f"안내를 종료할까요? 맞으면 {int(self.confirm_sec)}초 안에 버튼을 눌러 주세요."
                self._set_confirm_after_prompt("abort", None, prompt)
            return

    # -----------------
    # 메인 루프
    # -----------------
    def run(self) -> None:
        last_reload = 0.0

        while not self._stop.is_set():
            now = time.time()

            if now - last_reload > 0.5:
                self._reload_locations_if_changed()
                last_reload = now

            self._run_scheduled_tasks()
            self._confirm_timeout_tick()
            self._ready_idle_tick()
            self._nav_arrival_tick()

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

                    # READY / PAUSED에서만 GPT 처리
                    if self.state in (State.READY, State.PAUSED):
                        self._dispatch_to_gpt(ev)

            try:
                _done_ev, plan, err = self._res_q.get_nowait()
                self._inflight = False
                if plan is None:
                    if self.debug:
                        print(f"[GPT-ERR] {err}", file=sys.stderr)
                    if self.state in (State.READY, State.PAUSED):
                        self.say_and_then("연결이 불안정해요. 다시 말씀해 주세요.")
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

        try:
            if self.mic:
                self.mic.stop()
        except Exception:
            pass

        if self.tts:
            self.tts.stop()


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("--locations", default=str(DEFAULT_LOCATIONS))
    ap.add_argument("--env", default=str(DEFAULT_ENV_FILE))
    ap.add_argument("--model", default=os.environ.get("OPENAI_MODEL", "gpt-4o-mini"))

    # 상태 관련 시간
    ap.add_argument("--confirm-sec", type=float, default=14.0,
                    help="버튼 확인 대기 시간(초) - 질문을 다 말한 뒤부터 시작")
    ap.add_argument("--ready-idle-sec", type=float, default=30.0,
                    help="READY에서 무응답 시 LOCKED로 돌아가는 시간(초)")
    ap.add_argument("--arrival-sec", type=float, default=0.0,
                    help="테스트용 자동 도착 시간(초), 0이면 비활성")

    # 출력 / 입력 옵션
    ap.add_argument("--no-tts", action="store_true")
    ap.add_argument("--no-hw", action="store_true")
    ap.add_argument("--no-mic", action="store_true")

    # 하드웨어 포트
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)

    # 마이크/STT
    ap.add_argument("--mic-timeout", type=float, default=0.8,
                    help="말 시작을 기다리는 최대 시간(초)")
    ap.add_argument("--mic-phrase-time-limit", type=float, default=5.0,
                    help="한 번 발화를 최대 몇 초까지 들을지")
    ap.add_argument("--mic-pause-threshold", type=float, default=1.0,
                    help="이 시간만큼 조용하면 말 끝으로 판단")
    ap.add_argument("--mic-energy-threshold", type=int, default=350,
                    help="마이크 입력 민감도 기준")

    # 오디오 게이트
    ap.add_argument("--post-audio-cooldown-sec", type=float, default=0.7,
                    help="TTS/삑음 끝난 뒤 마이크를 더 닫아둘 시간")
    ap.add_argument("--listen-open-delay-sec", type=float, default=0.35,
                    help="마이크를 열기 전 추가 지연 후 시작 삑")

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
        no_mic=args.no_mic,
        port=args.port,
        baud=args.baud,
        mic_timeout=args.mic_timeout,
        mic_phrase_time_limit=args.mic_phrase_time_limit,
        mic_pause_threshold=args.mic_pause_threshold,
        mic_energy_threshold=args.mic_energy_threshold,
        post_audio_cooldown_sec=args.post_audio_cooldown_sec,
        listen_open_delay_sec=args.listen_open_delay_sec,
        debug=args.debug,
    )
    app.run()


if __name__ == "__main__":
    main()
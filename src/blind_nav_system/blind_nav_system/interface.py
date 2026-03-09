#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hello Robot – Interface v3
최종 기능 명세서 v3 기준 구현.
ROS 없이 테스트 가능: 이동은 20초 타이머로 시뮬레이션.
"""
from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import json
import math
import os
import queue
import re
import signal
import struct
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
import wave
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import speech_recognition as sr

try:
    import serial as _serial_mod
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False

# ──────────────────────────────────────────────
# 경로 기본값
# ──────────────────────────────────────────────
THIS_DIR = Path(__file__).resolve().parent
DEFAULT_LOCATIONS = (THIS_DIR / "../config/location.yaml").resolve()
DEFAULT_ENV_FILE  = (THIS_DIR / "../../.env").resolve()

# ──────────────────────────────────────────────
# 숫자 파라미터 (명세서 §6)
# ──────────────────────────────────────────────
BUTTON_GUARD_SEC            = 0.8
PULL_GUARD_SEC              = 1.0

LISTEN_OPEN_DELAY_SEC       = 0.35
LISTEN_TIMEOUT_SEC          = 10.0
LISTEN_PHRASE_TIME_LIMIT_SEC= 10.0
LISTEN_PAUSE_THRESHOLD_SEC  = 0.8
DEDUP_WINDOW_SEC            = 2.2

NO_RESPONSE_SESSION_SEC     = 10.0
NO_RESPONSE_MAX_SESSIONS    = 3
REMINDER_MAX_COUNT          = 2

READY_RETRY_MAX             = 3
PAUSED_RETRY_MAX            = 3
CONFIRM_TIMEOUT_SEC         = 30.0

START_LISTEN_BEEP_HZ        = 1350
START_LISTEN_BEEP_MS        = 110
END_LISTEN_BEEP_HZ          = 900
END_LISTEN_BEEP_MS          = 90

NAV_STOP_PUBLISH_SEC        = 1.0   # 시뮬레이션에서는 참고용
NAV_SIM_DURATION_SEC        = 20.0  # 로봇 없을 때 자동 도착 타이머

TTS_MAX_CHARS               = 120

# ──────────────────────────────────────────────
# 고정 TTS 문구 (명세서 §13)
# ──────────────────────────────────────────────
MSG = {
    1:  "버튼이 눌렸습니다. 어디로 가실 건가요?",
    2:  "목적지를 다시 짧게 말씀해 주세요.",
    3:  "현재 갈 수 있는 장소가 아닙니다. 다시 말씀해 주세요.",
    # 4: 복수 후보 – 동적 생성
    # 5: 단일 후보 확인 – 동적 생성
    6:  "확인했습니다. 출발하려면 버튼을 눌러주세요.",
    7:  "알겠습니다. 목적지를 다시 말씀해 주세요.",
    8:  "일시정지되었습니다. 문제가 있으신가요?",
    9:  "다시 원래 목적지로 이동할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요.",
    10: "안내를 종료할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요.",
    11: "목적지를 변경할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요.",
    # 12: 새 목적지 변경 확인 – 동적 생성
    13: "어디로 변경할까요? 목적지를 말씀해 주세요.",
    14: "안내를 시작하지 못했습니다. 다시 시도해 주세요.",
    15: "안내를 계속할 수 없어 중지했습니다. 필요하면 버튼을 눌러 다시 시작해 주세요.",
    16: "목적지에 도착했습니다.",
    17: "응답이 없어 안내를 종료합니다.",
    18: "안내를 종료합니다. 필요하면 버튼을 눌러 다시 시작해 주세요.",
    19: "말씀해 주세요.",          # 리마인드 1
    20: "필요하시면 말씀해 주세요.", # 리마인드 2
    21: "종료, 변경, 다시 이동 중 하나로 말씀해 주세요.",
    22: "알겠습니다. 원하시는 내용을 말씀해 주세요.",
    23: "확인했습니다. 진행하려면 버튼을 눌러주세요.",
}

def msg4(candidates: List[str]) -> str:
    return "후보가 여러 개 있습니다. " + ", ".join(candidates[:3]) + " 중 어디로 가실까요?"

def msg5(dest: str) -> str:
    return f"{dest}로 이동할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요."

def msg12(dest: str) -> str:
    return f"{dest}로 목적지를 변경할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요."


# ══════════════════════════════════════════════
# ALSA 오류 억제
# ══════════════════════════════════════════════
def _suppress_alsa() -> None:
    try:
        asound = ctypes.cdll.LoadLibrary(ctypes.util.find_library("asound") or "")
        HANDLER = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int,
                                   ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)
        _h = HANDLER(lambda *_: None)
        asound.snd_lib_error_set_handler(_h)
        _suppress_alsa._ref = _h
    except Exception:
        pass


# ══════════════════════════════════════════════
# YAML 파서
# ══════════════════════════════════════════════
def load_locations(path: Path) -> Dict[str, Dict[str, float]]:
    locs: Dict[str, Dict[str, float]] = {}
    if not path.exists():
        return locs
    in_loc = False
    cur: Optional[str] = None
    key_re = re.compile(r"^\s{2}([^:#\s][^:#]*)\s*:\s*$")
    val_re = re.compile(r"^\s{4}(x|y|w)\s*:\s*([-\d.eE]+)")
    for line in path.read_text("utf-8").splitlines():
        if not in_loc:
            if re.match(r"^\s*locations\s*:\s*$", line):
                in_loc = True
            continue
        m = key_re.match(line)
        if m:
            cur = m.group(1).strip(); locs.setdefault(cur, {}); continue
        m = val_re.match(line)
        if m and cur:
            locs[cur][m.group(1)] = float(m.group(2))
    return {k: v for k, v in locs.items() if all(t in v for t in ("x","y","w"))}


# ══════════════════════════════════════════════
# .env
# ══════════════════════════════════════════════
def load_openai_key(env_path: Path) -> str:
    k = os.environ.get("OPENAI_API_KEY", "").strip()
    if k:
        return k
    if env_path.exists():
        for line in env_path.read_text("utf-8").splitlines():
            s = line.strip()
            if s.startswith("OPENAI_API_KEY="):
                return s.split("=", 1)[1].strip().strip('"\'')
    return ""


# ══════════════════════════════════════════════
# TTS  (gTTS + pygame, 동기)
# ══════════════════════════════════════════════
class TTS:
    def __init__(self, debug: bool = False, enabled: bool = True):
        self.debug = debug
        self.enabled = enabled
        self._lock = threading.Lock()
        self._ok = False
        if enabled:
            try:
                import pygame
                from gtts import gTTS as _gTTS
                self._pygame = pygame
                self._gTTS = _gTTS
                pygame.mixer.init()
                self._ok = True
            except Exception as e:
                if debug:
                    print(f"[TTS] init failed: {e}", file=sys.stderr)

    def say(self, text: str) -> None:
        t = (text or "").strip()[:TTS_MAX_CHARS]
        if not t:
            return
        print(f"[ROBOT] {t}")
        if not self._ok:
            return
        with self._lock:
            try:
                import tempfile, os
                fd, path = tempfile.mkstemp(suffix=".mp3")
                os.close(fd)
                self._gTTS(text=t, lang="ko").save(path)
                self._pygame.mixer.music.load(path)
                self._pygame.mixer.music.play()
                while self._pygame.mixer.music.get_busy():
                    time.sleep(0.02)
                self._pygame.mixer.music.unload()
                os.remove(path)
            except Exception as e:
                if self.debug:
                    print(f"[TTS] say error: {e}", file=sys.stderr)


# ══════════════════════════════════════════════
# Beeper (wav 생성 후 pygame 재생)
# ══════════════════════════════════════════════
class Beeper:
    def __init__(self, debug: bool = False, enabled: bool = True):
        self.debug = debug
        self.enabled = enabled
        self._ok = False
        self._cache: Dict[Tuple[int,int], str] = {}
        self._tmpdir = tempfile.mkdtemp(prefix="beep_")
        if enabled:
            try:
                import pygame
                self._pygame = pygame
                pygame.mixer.init()
                self._ok = True
            except Exception as e:
                if debug:
                    print(f"[BEEP] init failed: {e}", file=sys.stderr)

    def _make_wav(self, hz: int, ms: int) -> str:
        key = (hz, ms)
        if key in self._cache:
            return self._cache[key]
        path = os.path.join(self._tmpdir, f"beep_{hz}_{ms}.wav")
        n = int(44100 * ms / 1000)
        with wave.open(path, "wb") as wf:
            wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(44100)
            data = bytearray()
            for i in range(n):
                s = int(12000 * math.sin(2 * math.pi * hz * i / 44100))
                data.extend(struct.pack("<h", s))
            wf.writeframes(bytes(data))
        self._cache[key] = path
        return path

    def beep(self, hz: int, ms: int) -> None:
        try:
            print("\a", end="", flush=True)
        except Exception:
            pass
        if not self._ok:
            return
        try:
            path = self._make_wav(hz, ms)
            self._pygame.mixer.music.load(path)
            self._pygame.mixer.music.play()
            while self._pygame.mixer.music.get_busy():
                time.sleep(0.005)
            self._pygame.mixer.music.unload()
        except Exception as e:
            if self.debug:
                print(f"[BEEP] error: {e}", file=sys.stderr)

    def start(self) -> None:
        self.beep(START_LISTEN_BEEP_HZ, START_LISTEN_BEEP_MS)

    def end(self) -> None:
        self.beep(END_LISTEN_BEEP_HZ, END_LISTEN_BEEP_MS)


# ══════════════════════════════════════════════
# AudioGate – TTS/beep 중 마이크 차단
# ══════════════════════════════════════════════
class AudioGate:
    def __init__(self) -> None:
        self._until = 0.0
        self._lock  = threading.Lock()

    def block(self, sec: float) -> None:
        with self._lock:
            self._until = max(self._until, time.time() + sec)

    def is_open(self) -> bool:
        with self._lock:
            return time.time() >= self._until


# ══════════════════════════════════════════════
# 상태 / 국면 / 이벤트 정의
# ══════════════════════════════════════════════
class State(Enum):
    LOCKED  = auto()
    READY   = auto()
    NAV     = auto()
    PAUSED  = auto()

class Phase(Enum):
    # READY
    READY_DEST_INPUT   = auto()
    READY_DISAMBIGUATE = auto()
    READY_CONFIRM      = auto()
    # PAUSED
    PAUSED_INTENT_INPUT = auto()
    PAUSED_CONFIRM      = auto()
    # NAV / LOCKED (phase 불필요하나 타입 완성용)
    NONE = auto()

@dataclass
class PendingConfirm:
    action:      str             # start|resume|abort|change_ready|change_start
    destination: Optional[str]
    deadline:    float

@dataclass
class Event:
    kind: str                    # button|pull|text|stt_empty|stt_timeout
    text: Optional[str] = None
    ts:   float = field(default_factory=time.time)


# ══════════════════════════════════════════════
# GPT Planner (명세서 §12)
# ══════════════════════════════════════════════
PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "action": {
            "type": "string",
            "enum": [
                "noop", "propose", "ask_again", "disambiguate",
                "resume_propose", "abort_propose",
                "change_dest_request", "change_dest_propose",
            ],
        },
        "destination": {"type": ["string", "null"]},
        "candidates":  {"type": "array", "items": {"type": "string"}},
        "need_button": {"type": "boolean"},
        "button_action": {
            "type": "string",
            "enum": ["none", "start", "resume", "abort", "change_ready", "change_start"],
        },
        "confidence": {"type": "string", "enum": ["high", "medium", "low"]},
        "reason": {
            "type": "string",
            "enum": ["none", "out_of_list", "ambiguous", "low_confidence", "stt_failed"],
        },
    },
    "required": ["action","destination","candidates","need_button",
                 "button_action","confidence","reason"],
    "additionalProperties": False,
}

class GPTPlanner:
    def __init__(self, api_key: str, model: str = "gpt-4o-mini", debug: bool = False):
        self.api_key = api_key
        self.model   = model
        self.debug   = debug

    def _system(self, locations: List[str], state_name: str) -> str:
        joined = ", ".join(locations) if locations else "(없음)"
        return (
            "너는 시각장애인 안내 로봇의 의도 해석기다.\n"
            "반드시 JSON schema만 출력해라. 자유 문장 금지.\n"
            f"공식 목적지 목록(이 외는 모두 out_of_list): {joined}\n"
            f"현재 상태: {state_name}\n\n"
            "[후보 수 규칙]\n"
            "- 후보 1개 → action=propose\n"
            "- 후보 2~3개 → action=disambiguate\n"
            "- 후보 4개 이상 또는 confidence=low → action=ask_again\n\n"
            "[READY 허용 action]: propose, ask_again, disambiguate\n"
            "[PAUSED 허용 action]: resume_propose, abort_propose, "
            "change_dest_request, change_dest_propose, ask_again\n\n"
            "[PAUSED 해석 규칙]\n"
            "- '다시 가/계속/재개' → resume_propose, button_action=resume\n"
            "- '종료/그만/끝' → abort_propose, button_action=abort\n"
            "- '바꾸고 싶어/다른 곳' (새 목적지 없음) → change_dest_request, button_action=change_ready\n"
            "- '3호로 바꿔/편의점으로 가자' (새 목적지 포함) → change_dest_propose, button_action=change_start\n"
            "- 불명확 → ask_again\n\n"
            "[reason 규칙]\n"
            "- out_of_list: 목록 밖 목적지\n"
            "- ambiguous: 후보 복수 또는 불명확\n"
            "- low_confidence: 확신 부족\n"
            "- stt_failed: STT 실패\n"
            "- none: 해당 없음\n"
        )

    def plan(self, *, state_name: str, locations: List[str], user_text: str) -> Dict[str, Any]:
        payload = {
            "model": self.model,
            "instructions": self._system(locations, state_name),
            "input": user_text,
            "max_output_tokens": 300,
            "text": {
                "format": {
                    "type": "json_schema",
                    "name": "voice_plan",
                    "schema": PLAN_SCHEMA,
                    "strict": True,
                }
            },
        }
        req = urllib.request.Request(
            "https://api.openai.com/v1/responses",
            data=json.dumps(payload).encode(),
            method="POST",
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=35) as r:
                resp = json.loads(r.read().decode())
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"HTTP {e.code}: {e.read().decode(errors='ignore')}")
        except Exception as e:
            raise RuntimeError(str(e))

        # output 추출
        for item in resp.get("output", []):
            if item.get("type") == "message" and item.get("role") == "assistant":
                for c in item.get("content", []):
                    if c.get("type") == "output_text":
                        raw = c["text"].strip()
                        if self.debug:
                            print(f"[GPT] {raw}", file=sys.stderr)
                        return json.loads(raw)
        raise RuntimeError("empty GPT output")


# ══════════════════════════════════════════════
# MicSession – 수음 1회 (§8 순서 엄수)
# ══════════════════════════════════════════════
class MicSession:
    """
    명세서 §8 절대 순서:
    TTS 완전 종료 → AudioGate 열림 확인 → 0.35초 대기
    → 시작 beep → 수음 → 종료 beep → STT
    """
    def __init__(
        self,
        tts:    TTS,
        beeper: Beeper,
        gate:   AudioGate,
        debug:  bool = False,
    ):
        self.tts    = tts
        self.beeper = beeper
        self.gate   = gate
        self.debug  = debug

        self._r = sr.Recognizer()
        self._r.pause_threshold          = LISTEN_PAUSE_THRESHOLD_SEC
        self._r.non_speaking_duration    = 0.4
        self._r.dynamic_energy_threshold = True

        self._last_text    = ""
        self._last_text_ts = 0.0

    # 중복 체크 (§9.2)
    def _is_dup(self, text: str) -> bool:
        now = time.time()
        t   = re.sub(r"\s+", "", text)
        if t == self._last_text and (now - self._last_text_ts) <= DEDUP_WINDOW_SEC:
            return True
        self._last_text    = t
        self._last_text_ts = now
        return False

    def listen(self) -> Tuple[Optional[str], str]:
        """
        Returns (text, status)
        status: 'ok' | 'empty' | 'timeout' | 'dup'
        """
        _suppress_alsa()

        # Gate 열릴 때까지 대기
        waited = 0.0
        while not self.gate.is_open():
            time.sleep(0.05)
            waited += 0.05
            if waited > 60:
                return None, "timeout"

        # 0.35초 대기 후 시작 beep
        time.sleep(LISTEN_OPEN_DELAY_SEC)
        self.beeper.start()

        # 수음
        try:
            mic = sr.Microphone()
            with mic as src:
                self._r.adjust_for_ambient_noise(src, duration=0.3)
                audio = self._r.listen(
                    src,
                    timeout=LISTEN_TIMEOUT_SEC,
                    phrase_time_limit=LISTEN_PHRASE_TIME_LIMIT_SEC,
                )
        except sr.WaitTimeoutError:
            self.beeper.end()
            return None, "timeout"
        except Exception as e:
            if self.debug:
                print(f"[MIC] listen error: {e}", file=sys.stderr)
            self.beeper.end()
            return None, "timeout"

        # 종료 beep
        self.beeper.end()

        # STT (§9.3)
        try:
            text = self._r.recognize_google(audio, language="ko-KR").strip()
        except sr.UnknownValueError:
            return "", "empty"
        except Exception as e:
            if self.debug:
                print(f"[MIC] STT error: {e}", file=sys.stderr)
            return "", "empty"

        if not text:
            return "", "empty"
        if self._is_dup(text):
            return None, "dup"
        print(f"[MIC] {text}")
        return text, "ok"


# ══════════════════════════════════════════════
# NavSimulator – 20초 뒤 arrive 이벤트
# ══════════════════════════════════════════════
class NavSimulator:
    def __init__(self, on_arrive: Callable[[], None], duration: float = NAV_SIM_DURATION_SEC):
        self._on_arrive = on_arrive
        self._duration  = duration
        self._timer: Optional[threading.Timer] = None
        self._lock = threading.Lock()

    def start(self, dest: str) -> bool:
        self.stop()
        print(f"[NAV] 이동 시작 → {dest} ({self._duration:.0f}초 후 도착)")
        with self._lock:
            self._timer = threading.Timer(self._duration, self._on_arrive)
            self._timer.daemon = True
            self._timer.start()
        return True

    def stop(self) -> None:
        with self._lock:
            if self._timer:
                self._timer.cancel()
                self._timer = None
        print("[NAV] 이동 정지")


# ══════════════════════════════════════════════
# 하드웨어 브리지 (시리얼 버튼 / Pull 센서)
# ══════════════════════════════════════════════
_GRIP_ARM   = 3000
_PULL_TRIG  = 3700
_QUICK_SEC  = 0.25
_GRIP_RESET = 2900
_HW_DEBOUNCE_SEC = 0.25


class _PullDetector:
    def __init__(self) -> None:
        self.armed = False
        self.armed_time = 0.0
        self.triggered_this_arm = False

    def update(self, val: int) -> Optional[str]:
        now = time.monotonic()
        if val < _GRIP_RESET:
            self.armed = False
            self.triggered_this_arm = False
            return None
        if (not self.armed) and val >= _GRIP_ARM:
            self.armed = True
            self.armed_time = now
            self.triggered_this_arm = False
            return None
        if self.armed:
            if self.triggered_this_arm:
                return None
            dt = now - self.armed_time
            if val >= _PULL_TRIG and dt <= _QUICK_SEC:
                self.triggered_this_arm = True
                return "PULL"
            if val >= _PULL_TRIG and dt > _QUICK_SEC:
                self.triggered_this_arm = True
                return None
        return None


class HardwareKeyBridge:
    """
    시리얼 포트에서 CSV 라인을 읽어 버튼/pull 이벤트를 콜백으로 전달.
    - 'TRIG' 태그        → on_o() (버튼)
    - val >= PULL_TRIG  → on_p() (pull)
    없거나 열기 실패 시 자동으로 None 반환 후 비활성화.
    """
    def __init__(self, on_o: Callable, on_p: Callable,
                 port: str = "/dev/ttyUSB0", baud: int = 115200,
                 debug: bool = False) -> None:
        self.on_o  = on_o
        self.on_p  = on_p
        self.port  = port
        self.baud  = baud
        self.debug = debug

        self._stop  = threading.Event()
        self._thr: Optional[threading.Thread] = None
        self._ser   = None
        self._buf   = ""
        self._pull  = _PullDetector()
        self._last_o_t = 0.0
        self._last_p_t = 0.0

    def start(self) -> bool:
        try:
            import serial
            self._ser = serial.Serial(self.port, self.baud, timeout=0)
        except Exception as e:
            if self.debug:
                print(f"[HW] serial open failed ({self.port}): {e}", file=sys.stderr)
            return False
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()
        print(f"[HW] 시리얼 연결됨: {self.port} @ {self.baud}")
        return True

    def stop(self) -> None:
        self._stop.set()
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass

    def _debounced(self, kind: str) -> None:
        now = time.monotonic()
        if kind == "o":
            if now - self._last_o_t >= _HW_DEBOUNCE_SEC:
                self._last_o_t = now
                self.on_o()
        elif kind == "p":
            if now - self._last_p_t >= _HW_DEBOUNCE_SEC:
                self._last_p_t = now
                self.on_p()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                if self._ser.in_waiting > 0:
                    self._buf += self._ser.read(self._ser.in_waiting).decode("utf-8", errors="ignore")
                    if "\n" in self._buf:
                        lines = self._buf.split("\n")
                        self._buf = lines.pop()
                        for line in lines:
                            line = line.strip()
                            if not line:
                                continue
                            parts = line.split(",")
                            if len(parts) < 3:
                                continue
                            tag = parts[0].strip()
                            try:
                                press_val = int(parts[2].strip())
                            except Exception:
                                continue
                            if tag == "TRIG":
                                self._debounced("o")
                            evt = self._pull.update(press_val)
                            if evt == "PULL":
                                self._debounced("p")
            except Exception as e:
                if self.debug:
                    print(f"[HW] loop error: {e}", file=sys.stderr)
            time.sleep(0.005)


# ══════════════════════════════════════════════
# 메인 앱
# ══════════════════════════════════════════════
class InterfaceApp:
    def __init__(
        self,
        *,
        locations_path: Path,
        env_path:       Path,
        debug:          bool  = False,
        no_tts:         bool  = False,
        no_mic:         bool  = False,
        no_hw:          bool  = False,
        hw_port:        str   = "/dev/ttyUSB0",
        hw_baud:        int   = 115200,
        model:          str   = "gpt-4o-mini",
    ):
        self.debug          = debug
        self.locations_path = locations_path
        self.locations      = load_locations(locations_path)

        # 출력
        self.tts    = TTS(debug=debug, enabled=not no_tts)
        self.beeper = Beeper(debug=debug, enabled=not no_tts)
        self.gate   = AudioGate()
        self.no_mic = no_mic

        # GPT
        api_key = load_openai_key(env_path)
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY 없음 – .env 파일 또는 환경변수를 설정하세요.")
        self.gpt = GPTPlanner(api_key=api_key, model=model, debug=debug)

        # 마이크 세션
        self.mic_session = MicSession(tts=self.tts, beeper=self.beeper, gate=self.gate, debug=debug)

        # 내비게이션 시뮬레이터
        self.nav = NavSimulator(on_arrive=self._on_arrive)

        # 상태
        self.state:   State            = State.LOCKED
        self.phase:   Phase            = Phase.NONE
        self.pending: Optional[PendingConfirm] = None
        self.last_target: Optional[str] = None

        # retry 카운터
        self.ready_retry  = 0
        self.paused_retry = 0

        # 무응답 세션 카운터
        self._no_resp_sessions = 0

        # guard
        self._btn_guard_until:  float = 0.0
        self._pull_guard_until: float = 0.0

        # 이벤트 큐
        self._ev_q: queue.Queue[Event] = queue.Queue()
        self._stop = threading.Event()

        # 하드웨어 버튼/pull (시리얼)
        self.hw: Optional[HardwareKeyBridge] = None
        if not no_hw:
            self.hw = HardwareKeyBridge(
                on_o=lambda: self._ev_q.put(Event(kind="button")),
                on_p=lambda: self._ev_q.put(Event(kind="pull")),
                port=hw_port,
                baud=hw_baud,
                debug=debug,
            )
            ok = self.hw.start()
            if not ok:
                print("[HW] 시리얼 연결 실패 – stdin(/button, /pull)으로 테스트하세요.")
                self.hw = None

        # stdin 테스트용 (항상 켜둠)
        threading.Thread(target=self._stdin_worker, daemon=True).start()

    # ──────────────────────────────────────────
    # 유틸
    # ──────────────────────────────────────────
    def destinations(self) -> List[str]:
        return sorted(self.locations.keys())

    def _say(self, text: str) -> None:
        """TTS 재생 + AudioGate 차단. TTS는 동기이므로 재생 끝나면 바로 gate 해제."""
        # 말하는 동안 gate 차단 (대략 추산: 글자당 0.09초 + 여유)
        est = max(1.5, len(text) * 0.09 + 1.0)
        self.gate.block(est)
        self.tts.say(text)
        # 실제로는 say()가 동기이므로 여기서 gate는 이미 만료됨
        # 하지만 beep/다음 액션 전 짧은 쿨다운 보장
        self.gate.block(0.3)

    def _state_log(self) -> None:
        p = self.phase.name if self.phase != Phase.NONE else ""
        print(f"[STATE] {self.state.name}" + (f" / {p}" if p else ""))

    # ──────────────────────────────────────────
    # PendingConfirm
    # ──────────────────────────────────────────
    def _set_pending(self, action: str, destination: Optional[str]) -> None:
        self.pending = PendingConfirm(
            action=action,
            destination=destination,
            deadline=time.time() + CONFIRM_TIMEOUT_SEC,
        )

    def _reset_pending_deadline(self) -> None:
        if self.pending:
            self.pending.deadline = time.time() + CONFIRM_TIMEOUT_SEC

    def _clear_pending(self) -> None:
        self.pending = None

    # ──────────────────────────────────────────
    # 도착 콜백 (NavSimulator → 메인 스레드)
    # ──────────────────────────────────────────
    def _on_arrive(self) -> None:
        self._ev_q.put(Event(kind="arrive"))

    # ──────────────────────────────────────────
    # stdin 테스트 입력
    # ──────────────────────────────────────────
    def _stdin_worker(self) -> None:
        """
        /button  → 버튼 이벤트
        /pull    → pull 이벤트
        그 외    → text 이벤트
        """
        while not self._stop.is_set():
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                break
            s = line.strip()
            if not s:
                continue
            if s == "/button":
                self._ev_q.put(Event(kind="button"))
            elif s == "/pull":
                self._ev_q.put(Event(kind="pull"))
            else:
                self._ev_q.put(Event(kind="text", text=s))

    # ──────────────────────────────────────────
    # 마이크 세션 실행 (별도 스레드)
    # 명세서 §8, §10 구조: 최대 3세션 + 리마인드 2회
    # ──────────────────────────────────────────
    def _run_mic_sessions(self, context: str) -> None:
        """
        context: 'ready' | 'paused'
        3세션, 리마인드 2회 구조.
        결과를 ev_q에 넣는다.
        """
        if self.no_mic:
            return

        def worker():
            for session_idx in range(NO_RESPONSE_MAX_SESSIONS):
                # 리마인드 (2차, 3차 수음 전)
                if session_idx == 1:
                    self._say(MSG[19])   # 리마인드 1
                elif session_idx == 2:
                    self._say(MSG[20])   # 리마인드 2

                text, status = self.mic_session.listen()

                if status == "ok" and text:
                    self._ev_q.put(Event(kind="text", text=text))
                    return
                elif status == "empty":
                    # 인식 실패 → retry count 증가 (메인에서 처리)
                    self._ev_q.put(Event(kind="stt_empty"))
                    return
                elif status == "dup":
                    # 중복이면 조용히 재시도
                    continue
                else:
                    # timeout → 무응답 세션 소모, 다음 세션으로
                    continue

            # 3세션 모두 무응답
            self._ev_q.put(Event(kind="stt_timeout"))

        threading.Thread(target=worker, daemon=True).start()

    # ──────────────────────────────────────────
    # 상태 전이 헬퍼
    # ──────────────────────────────────────────
    def _go_locked(self, reason: str = "") -> None:
        self.state  = State.LOCKED
        self.phase  = Phase.NONE
        self._clear_pending()
        self.nav.stop()
        self._state_log()

    def _go_ready_dest_input(self) -> None:
        self.state       = State.READY
        self.phase       = Phase.READY_DEST_INPUT
        self.ready_retry = 0
        self._clear_pending()
        self._state_log()
        self._run_mic_sessions("ready")

    def _go_ready_disambiguate(self) -> None:
        self.phase = Phase.READY_DISAMBIGUATE
        self._state_log()
        self._run_mic_sessions("ready")

    def _go_ready_confirm(self, dest: str) -> None:
        self.phase = Phase.READY_CONFIRM
        self._set_pending("start", dest)
        self._state_log()
        # 수음 열기 (음성 긍정/부정 가능)
        self._run_mic_sessions("ready")

    def _go_nav(self, dest: str) -> None:
        ok = self.nav.start(dest)
        if not ok:
            self._say(MSG[14])
            # READY 유지
            self.state = State.READY
            self.phase = Phase.READY_DEST_INPUT
            self._state_log()
            return
        self.last_target = dest
        self.state = State.NAV
        self.phase = Phase.NONE
        self._clear_pending()
        self._state_log()
        self._say(f"{dest}로 안내를 시작합니다.")

    def _go_paused(self) -> None:
        self.nav.stop()
        self.state        = State.PAUSED
        self.phase        = Phase.PAUSED_INTENT_INPUT
        self.paused_retry = 0
        self._clear_pending()
        self._state_log()
        self._say(MSG[8])
        self._run_mic_sessions("paused")

    def _go_paused_confirm(self, action: str, dest: Optional[str], prompt_msg: str) -> None:
        self.phase = Phase.PAUSED_CONFIRM
        self._set_pending(action, dest)
        self._state_log()
        self._say(prompt_msg)
        self._run_mic_sessions("paused")

    # ──────────────────────────────────────────
    # 버튼 처리 (명세서 §14)
    # ──────────────────────────────────────────
    def _handle_button(self) -> None:
        now = time.time()
        if now < self._btn_guard_until:
            return
        self._btn_guard_until = now + BUTTON_GUARD_SEC

        s, ph = self.state, self.phase

        # ── LOCKED ──────────────────────────────
        if s == State.LOCKED:
            self._say(MSG[1])
            self._go_ready_dest_input()
            return

        # ── READY_CONFIRM ────────────────────────
        if s == State.READY and ph == Phase.READY_CONFIRM:
            if self.pending and self.pending.action == "start" and self.pending.destination:
                dest = self.pending.destination
                self._clear_pending()
                self._go_nav(dest)
            return

        # ── READY (그 외) ─────────────────────────
        if s == State.READY:
            # 입력 무시 (§11: READY_DEST_INPUT, READY_DISAMBIGUATE는 button 무시)
            return

        # ── NAV ──────────────────────────────────
        if s == State.NAV:
            self._go_paused()
            return

        # ── PAUSED_INTENT_INPUT ──────────────────
        if s == State.PAUSED and ph == Phase.PAUSED_INTENT_INPUT:
            # 버튼 → 재개 확인 (§14.4)
            self._go_paused_confirm("resume", self.last_target, MSG[9])
            return

        # ── PAUSED_CONFIRM ───────────────────────
        if s == State.PAUSED and ph == Phase.PAUSED_CONFIRM:
            if not self.pending:
                return
            self._exec_paused_confirm_button()
            return

    def _exec_paused_confirm_button(self) -> None:
        """PAUSED_CONFIRM에서 버튼 → 확정 실행"""
        pc = self.pending
        if not pc:
            return
        self._clear_pending()

        if pc.action == "resume":
            # PAUSED → READY → NAV
            self.state = State.READY
            self.phase = Phase.NONE
            self._state_log()
            if self.last_target:
                self._go_nav(self.last_target)
            else:
                self._say(MSG[14])
                self._go_ready_dest_input()

        elif pc.action == "abort":
            self._say("안내를 종료합니다.")
            self.last_target = None
            self._go_locked()

        elif pc.action == "change_ready":
            self._clear_pending()
            # last_target 유지
            self.state       = State.READY
            self.phase       = Phase.READY_DEST_INPUT
            self.ready_retry = 0
            self._state_log()
            self._say(MSG[13])
            self._run_mic_sessions("ready")

        elif pc.action == "change_start":
            if pc.destination:
                self.last_target = pc.destination
                self.state = State.READY
                self.phase = Phase.NONE
                self._state_log()
                self._go_nav(pc.destination)
            else:
                self._say(MSG[14])

    # ──────────────────────────────────────────
    # Pull 처리 (§14.3)
    # ──────────────────────────────────────────
    def _handle_pull(self) -> None:
        now = time.time()
        if now < self._pull_guard_until:
            return
        self._pull_guard_until = now + PULL_GUARD_SEC

        if self.state == State.NAV:
            self._go_paused()

    # ──────────────────────────────────────────
    # Text (음성 인식 결과) 처리
    # ──────────────────────────────────────────
    def _handle_text(self, text: str) -> None:
        s, ph = self.state, self.phase

        # 입력 무시 규칙 (§11)
        if s == State.LOCKED:
            return
        if s == State.NAV:
            return
        if s == State.READY and ph in (Phase.READY_DEST_INPUT, Phase.READY_DISAMBIGUATE):
            self._process_ready_text(text)
            return
        if s == State.READY and ph == Phase.READY_CONFIRM:
            self._process_ready_confirm_voice(text)
            return
        if s == State.PAUSED and ph == Phase.PAUSED_INTENT_INPUT:
            self._process_paused_text(text)
            return
        if s == State.PAUSED and ph == Phase.PAUSED_CONFIRM:
            self._process_paused_confirm_voice(text)
            return

    # ── READY: 목적지 / 후보 입력 ─────────────
    def _process_ready_text(self, text: str) -> None:
        try:
            plan = self.gpt.plan(
                state_name="READY",
                locations=self.destinations(),
                user_text=text,
            )
        except Exception as e:
            print(f"[GPT-ERR] {e}", file=sys.stderr)
            self._say(MSG[2])
            self._run_mic_sessions("ready")
            return

        action     = plan.get("action", "noop")
        dest       = plan.get("destination")
        candidates = plan.get("candidates", [])
        reason     = plan.get("reason", "none")

        if action == "propose" and dest and dest in self.destinations():
            self._say(msg5(dest))
            self._go_ready_confirm(dest)

        elif action == "disambiguate":
            self.ready_retry += 1
            if self.ready_retry > READY_RETRY_MAX:
                self._say(MSG[18])
                self._go_locked()
                return
            self._say(msg4(candidates))
            self._go_ready_disambiguate()

        elif action == "ask_again":
            self.ready_retry += 1
            if self.ready_retry > READY_RETRY_MAX:
                self._say(MSG[18])
                self._go_locked()
                return
            if reason == "out_of_list":
                self._say(MSG[3])
            else:
                self._say(MSG[2])
            self._run_mic_sessions("ready")

        else:
            # noop 등
            self._say(MSG[2])
            self._run_mic_sessions("ready")

    # ── READY_CONFIRM: 음성 긍정/부정 ──────────
    def _process_ready_confirm_voice(self, text: str) -> None:
        affirmative = {"네", "예", "응", "맞아요", "맞아", "좋아", "그래"}
        negative    = {"아니요", "아니", "틀렸어요", "아니에요"}

        t = text.strip()
        if t in affirmative:
            self._reset_pending_deadline()
            self._say(MSG[6])
            # 버튼 대기 유지 (수음은 다시 열어 둠)
            self._run_mic_sessions("ready")
            return

        if t in negative:
            self._clear_pending()
            self.phase = Phase.READY_DEST_INPUT
            self._say(MSG[7])
            self._run_mic_sessions("ready")
            return

        # 수정 발화 ("아니요 2동이요" 등) → 새 목적지로 재처리
        self._clear_pending()
        self.phase = Phase.READY_DEST_INPUT
        self._process_ready_text(t)

    # ── PAUSED: 의도 입력 ─────────────────────
    def _process_paused_text(self, text: str) -> None:
        try:
            plan = self.gpt.plan(
                state_name="PAUSED",
                locations=self.destinations(),
                user_text=text,
            )
        except Exception as e:
            print(f"[GPT-ERR] {e}", file=sys.stderr)
            self._say(MSG[21])
            self._run_mic_sessions("paused")
            return

        action = plan.get("action", "noop")
        dest   = plan.get("destination")

        if action == "resume_propose":
            self._go_paused_confirm("resume", self.last_target, MSG[9])

        elif action == "abort_propose":
            self._go_paused_confirm("abort", None, MSG[10])

        elif action == "change_dest_request":
            self._go_paused_confirm("change_ready", None, MSG[11])

        elif action == "change_dest_propose" and dest and dest in self.destinations():
            self._go_paused_confirm("change_start", dest, msg12(dest))

        elif action == "ask_again":
            self.paused_retry += 1
            if self.paused_retry > PAUSED_RETRY_MAX:
                self._say(MSG[18])
                self._go_locked()
                return
            self._say(MSG[21])
            self._run_mic_sessions("paused")

        else:
            self.paused_retry += 1
            if self.paused_retry > PAUSED_RETRY_MAX:
                self._say(MSG[18])
                self._go_locked()
                return
            self._say(MSG[21])
            self._run_mic_sessions("paused")

    # ── PAUSED_CONFIRM: 음성 긍정/부정 ─────────
    def _process_paused_confirm_voice(self, text: str) -> None:
        affirmative = {"네", "예", "응", "맞아요", "맞아", "좋아", "그래"}
        negative    = {"아니요", "아니", "틀렸어요", "아니에요"}

        t = text.strip()
        if t in affirmative:
            self._reset_pending_deadline()
            self._say(MSG[23])
            self._run_mic_sessions("paused")
            return

        if t in negative:
            self._clear_pending()
            self.phase = Phase.PAUSED_INTENT_INPUT
            self._say(MSG[22])
            self._run_mic_sessions("paused")
            return

        # 수정 발화 → PAUSED_INTENT_INPUT으로 재처리
        self._clear_pending()
        self.phase = Phase.PAUSED_INTENT_INPUT
        self._process_paused_text(t)

    # ──────────────────────────────────────────
    # STT 결과: 빈 텍스트 (인식 실패)
    # ──────────────────────────────────────────
    def _handle_stt_empty(self) -> None:
        s, ph = self.state, self.phase
        if s == State.READY:
            self.ready_retry += 1
            if self.ready_retry > READY_RETRY_MAX:
                self._say(MSG[18])
                self._go_locked()
                return
            self._say(MSG[2])
            self._run_mic_sessions("ready")
        elif s == State.PAUSED:
            self.paused_retry += 1
            if self.paused_retry > PAUSED_RETRY_MAX:
                self._say(MSG[18])
                self._go_locked()
                return
            self._say(MSG[21])
            self._run_mic_sessions("paused")

    # ──────────────────────────────────────────
    # STT 결과: 3세션 무응답
    # ──────────────────────────────────────────
    def _handle_stt_timeout(self) -> None:
        self._say(MSG[17])
        self._go_locked()

    # ──────────────────────────────────────────
    # 도착 이벤트 (§14.3)
    # ──────────────────────────────────────────
    def _handle_arrive(self) -> None:
        if self.state == State.NAV:
            self.last_target = None
            self._say(MSG[16])
            self._go_locked()

    # ──────────────────────────────────────────
    # Tick: confirm timeout, READY idle (§14)
    # ──────────────────────────────────────────
    def _tick(self) -> None:
        now = time.time()

        # confirm timeout
        if self.pending and now >= self.pending.deadline:
            self._clear_pending()
            self._say(MSG[17])
            self._go_locked()
            return

    # ──────────────────────────────────────────
    # 메인 루프
    # ──────────────────────────────────────────
    def run(self) -> None:
        print("[APP] 시작. LOCKED 상태.")
        print("      stdin: /button | /pull | <텍스트>")
        while not self._stop.is_set():
            self._tick()
            try:
                ev = self._ev_q.get(timeout=0.1)
            except queue.Empty:
                continue

            if ev.kind == "button":
                self._handle_button()
            elif ev.kind == "pull":
                self._handle_pull()
            elif ev.kind == "text" and ev.text:
                self._handle_text(ev.text)
            elif ev.kind == "stt_empty":
                self._handle_stt_empty()
            elif ev.kind == "stt_timeout":
                self._handle_stt_timeout()
            elif ev.kind == "arrive":
                self._handle_arrive()

    def close(self) -> None:
        self._stop.set()
        self.nav.stop()
        if self.hw:
            self.hw.stop()


# ══════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════
def main() -> None:
    ap = argparse.ArgumentParser(description="Hello Robot Interface v3")
    ap.add_argument("--locations",  type=Path, default=DEFAULT_LOCATIONS)
    ap.add_argument("--env-file",   type=Path, default=DEFAULT_ENV_FILE)
    ap.add_argument("--model",      default=os.environ.get("OPENAI_MODEL", "gpt-4o-mini"))
    ap.add_argument("--no-tts",     action="store_true", help="TTS 비활성 (콘솔 출력만)")
    ap.add_argument("--no-mic",     action="store_true", help="마이크 비활성 (stdin만)")
    ap.add_argument("--no-hw",      action="store_true", help="하드웨어 브리지 비활성")
    ap.add_argument("--hw-port",    default="/dev/ttyUSB0", help="시리얼 포트 (기본: /dev/ttyUSB0)")
    ap.add_argument("--hw-baud",    type=int, default=115200, help="시리얼 보드레이트 (기본: 115200)")
    ap.add_argument("--debug",      action="store_true")
    args = ap.parse_args()

    app = InterfaceApp(
        locations_path=args.locations,
        env_path=args.env_file,
        debug=args.debug,
        no_tts=args.no_tts,
        no_mic=args.no_mic,
        no_hw=args.no_hw,
        hw_port=args.hw_port,
        hw_baud=args.hw_baud,
        model=args.model,
    )

    def _sig(*_):
        app.close()
        raise SystemExit(0)

    signal.signal(signal.SIGINT,  _sig)
    signal.signal(signal.SIGTERM, _sig)

    try:
        app.run()
    finally:
        app.close()


if __name__ == "__main__":
    main()
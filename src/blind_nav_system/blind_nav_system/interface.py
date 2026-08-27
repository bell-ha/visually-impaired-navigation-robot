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

DEFAULT_ENV_FILE = Path("/home/hello-robot/GitHub/visually-impaired-navigation-robot/src/.env")

# ──────────────────────────────────────────────
# 숫자 파라미터 (명세서 §6)
# ──────────────────────────────────────────────
BUTTON_GUARD_SEC            = 1.5
PULL_GUARD_SEC              = 1.0

LISTEN_OPEN_DELAY_SEC       = 0.6
LISTEN_TIMEOUT_SEC          = 10.0
LISTEN_PHRASE_TIME_LIMIT_SEC= 5.0
LISTEN_PAUSE_THRESHOLD_SEC  = 0.5
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

TTS_MAX_CHARS               = 250

_TTS_CACHE_DIR   = Path("/tmp/blind_nav_tts_cache")
_VOSK_MODEL_PATH = Path.home() / ".cache/vosk/vosk-model-small-ko-0.22"

# 강제 오프라인 모드 (/offline · /online stdin 명령으로 전환)
_force_offline: bool = False

# 동적 문구 중 항상 같은 내용 — MSG와 함께 시작 시 캐싱
_EXTRA_CACHE_TEXTS = [
    "앞에 장애물이 있습니다. 잠시 비켜주세요.",
    "왼쪽으로 회전합니다.",
    "오른쪽으로 회전합니다.",
    "잠시 뒤로 이동합니다.",
    "안내를 종료합니다.",
]

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

def _normalize_tts(text: str) -> str:
    """TTS 발음 보정: 영어 약어 → 한국어 발음"""
    text = text.replace("KAIST", "카이스트")
    text = text.replace("POSTECH", "포스텍")
    text = text.replace("UNIST", "유니스트")
    text = text.replace("ITRC", "아이티알씨")
    return text

def _dest_tts(name: str) -> str:
    """목적지 TTS 표기. '부스'는 행사 부스(ITRC ...)에만 붙임 —
    방 호수(523호 등)에 '부스'가 붙던 행사용 하드코딩 제거
    (2026-07-24, location.yaml 5층 방 목록 복원과 함께)."""
    d = _normalize_tts(name)
    return f"{d} 부스" if "ITRC" in name else d

def msg4(candidates: List[str]) -> str:
    c = [_dest_tts(n) for n in candidates[:3]]
    if len(c) == 1:
        joined = c[0]
    elif len(c) == 2:
        joined = f"{c[0]}, 그리고 {c[1]}"
    else:
        joined = f"{c[0]}, {c[1]}, 그리고 {c[2]}"
    return f"후보가 여러 개 있습니다. {joined} 중 어디로 가실까요?"

def msg5(dest: str) -> str:
    d = _dest_tts(dest)
    return f"{d}로 이동할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요."

def msg12(dest: str) -> str:
    d = _dest_tts(dest)
    return f"{d}로 목적지를 변경할까요? 맞으면 버튼을 눌러주세요. 아니면 말씀해 주세요."


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
    if not path.exists():
        return {}
    import yaml
    data = yaml.safe_load(path.read_text("utf-8")) or {}
    locs: Dict[str, Dict[str, float]] = {}
    for name, vals in (data.get("locations") or {}).items():
        try:
            locs[str(name)] = {
                "x": float(vals["x"]),
                "y": float(vals["y"]),
                "w": float(vals["w"]),
            }
        except Exception:
            pass
    return locs


# ══════════════════════════════════════════════
# vosk 오프라인 STT 모델 (lazy-init)
# ══════════════════════════════════════════════
_vosk_model_instance = None
_vosk_rec_instance   = None
_vosk_rec_lock       = threading.Lock()
_vosk_model_lock     = threading.Lock()

def _get_vosk_model():
    global _vosk_model_instance, _vosk_rec_instance
    if _vosk_model_instance is not None:
        return _vosk_model_instance
    with _vosk_model_lock:
        if _vosk_model_instance is not None:
            return _vosk_model_instance
        if not _VOSK_MODEL_PATH.exists():
            return None
        try:
            from vosk import Model, KaldiRecognizer
            import logging
            logging.getLogger("vosk").setLevel(logging.ERROR)
            _vosk_model_instance = Model(str(_VOSK_MODEL_PATH))
            _vosk_rec_instance   = KaldiRecognizer(_vosk_model_instance, 16000)
            print("[STT] vosk 한국어 모델 로드 완료")
            return _vosk_model_instance
        except Exception as e:
            print(f"[STT] vosk 모델 로드 실패: {e}", file=sys.stderr)
            return None


# ══════════════════════════════════════════════
# 오프라인 의도 파악 (difflib 퍼지 매칭)
# ══════════════════════════════════════════════
def _offline_plan(text: str, locations: List[str], state_name: str) -> Dict[str, Any]:
    """인터넷 없을 때 키워드 + 유사도 기반 의도 파악."""
    import difflib

    _base: Dict[str, Any] = {
        "destination": None, "candidates": [],
        "need_button": False, "button_action": "none",
        "confidence": "low", "reason": "none",
    }

    t = _normalize_tts(text).lower().strip()

    # ── PAUSED 상태: 재개/종료/변경 키워드 우선 처리 ──────
    if state_name == "PAUSED":
        resume_kw = ["계속", "다시", "재개", "그냥 가", "출발", "가자"]
        abort_kw  = ["종료", "그만", "끝", "멈춰", "취소"]
        change_kw = ["바꿔", "변경", "다른"]
        if any(k in t for k in resume_kw):
            return {**_base, "action": "resume_propose",   "button_action": "resume"}
        if any(k in t for k in abort_kw):
            return {**_base, "action": "abort_propose",    "button_action": "abort"}
        if any(k in t for k in change_kw):
            return {**_base, "action": "change_dest_request", "button_action": "change_ready"}

    # ── 목적지 매칭 ────────────────────────────────────────
    norm_map = {_normalize_tts(l).lower(): l for l in locations}

    # 1단계: 단어 단위 포함 검색 (예: "카이스트" → KAIST 포함 목적지)
    hits = [
        orig for norm, orig in norm_map.items()
        if any(w in norm for w in t.split() if len(w) >= 2)
    ]

    # 2단계: difflib 유사도 (1단계 결과 없을 때)
    if not hits:
        close = difflib.get_close_matches(t, list(norm_map.keys()), n=3, cutoff=0.3)
        hits = [norm_map[c] for c in close]

    if not hits:
        return {**_base, "action": "ask_again", "reason": "out_of_list"}

    if len(hits) == 1:
        action = "change_dest_propose" if state_name == "PAUSED" else "propose"
        btn    = "change_start"        if state_name == "PAUSED" else "none"
        return {**_base, "action": action, "destination": hits[0],
                "button_action": btn, "confidence": "medium"}

    return {**_base, "action": "disambiguate", "candidates": hits[:3],
            "confidence": "medium", "reason": "ambiguous"}


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
# TTS  (gTTS + ffmpeg + pyaudio, 동기)
# ══════════════════════════════════════════════
class TTS:
    def __init__(self, debug: bool = False, enabled: bool = True, audio_device: int = 0):
        self.debug = debug
        self.enabled = enabled
        self.audio_device = audio_device
        self._lock = threading.Lock()
        self._ok = False
        self._cache: Dict[str, str] = {}   # normalized_text → wav 절대경로
        if enabled:
            try:
                from gtts import gTTS as _gTTS
                import pyaudio as _pyaudio
                self._gTTS = _gTTS
                self._pa = _pyaudio.PyAudio()
                self._ok = True
            except Exception as e:
                if debug:
                    print(f"[TTS] init failed: {e}", file=sys.stderr)

    def _play_wav(self, wav_path: str) -> None:
        """WAV 파일을 pyaudio로 재생."""
        with wave.open(wav_path, "rb") as wf:
            stream = self._pa.open(
                format=self._pa.get_format_from_width(wf.getsampwidth()),
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True,
                output_device_index=self.audio_device,
            )
            data = wf.readframes(1024)
            while data:
                stream.write(data)
                data = wf.readframes(1024)
            stream.stop_stream()
            stream.close()

    def prime_cache(self, texts: List[str]) -> None:
        """앱 시작 시 고정 메시지를 백그라운드로 미리 캐싱 (온라인 중에 생성)."""
        if not self._ok:
            return
        def _worker():
            import hashlib, subprocess as _sp
            _TTS_CACHE_DIR.mkdir(parents=True, exist_ok=True)
            for raw in texts:
                t = _normalize_tts(raw.strip())[:TTS_MAX_CHARS]
                if not t or t in self._cache:
                    continue
                h = hashlib.md5(t.encode()).hexdigest()[:12]
                wav_path = str(_TTS_CACHE_DIR / f"{h}.wav")
                if Path(wav_path).exists():
                    self._cache[t] = wav_path
                    continue
                mp3_path = str(_TTS_CACHE_DIR / f"{h}.mp3")
                try:
                    self._gTTS(text=t, lang="ko").save(mp3_path)
                    _sp.run(
                        ["ffmpeg", "-y", "-i", mp3_path,
                         "-filter:a", "atempo=1.5",
                         "-ar", "44100", "-ac", "1", wav_path],
                        capture_output=True, timeout=15,
                    )
                    Path(mp3_path).unlink(missing_ok=True)
                    self._cache[t] = wav_path
                    print(f"[TTS-CACHE] 캐싱: {t[:40]}")
                except Exception as e:
                    if self.debug:
                        print(f"[TTS-CACHE] 실패: {t[:40]} – {e}", file=sys.stderr)
                    for p in [mp3_path, wav_path]:
                        try:
                            Path(p).unlink(missing_ok=True)
                        except Exception:
                            pass
        threading.Thread(target=_worker, daemon=True, name="tts-cache").start()

    def say(self, text: str) -> None:
        t = _normalize_tts((text or "").strip())[:TTS_MAX_CHARS]
        if not t:
            return
        print(f"[ROBOT] {t}")
        if not self._ok:
            return
        with self._lock:
            # 오프라인 강제 모드: gTTS 타임아웃 없이 캐시 직행
            if _force_offline:
                cached = self._cache.get(t)
                if cached and Path(cached).exists():
                    try:
                        self._play_wav(cached)
                    except Exception as e:
                        if self.debug:
                            print(f"[TTS] 캐시 재생 실패: {e}", file=sys.stderr)
                return

            mp3_path = wav_path = None
            try:
                import subprocess
                fd, mp3_path = tempfile.mkstemp(suffix=".mp3")
                os.close(fd)
                wav_path = mp3_path.replace(".mp3", ".wav")
                self._gTTS(text=t, lang="ko").save(mp3_path)
                try:
                    speed = float(Path("/tmp/tts_speed").read_text().strip())
                    speed = max(0.5, min(2.0, speed))
                except Exception:
                    speed = 1.5
                subprocess.run(
                    ["ffmpeg", "-y", "-i", mp3_path,
                     "-filter:a", f"atempo={speed}",
                     "-ar", "44100", "-ac", "1", wav_path],
                    capture_output=True, timeout=15,
                )
                self._play_wav(wav_path)
            except Exception as e:
                if self.debug:
                    print(f"[TTS] gTTS 실패: {e}", file=sys.stderr)
                # ── 오프라인 캐시 폴백 ──────────────────────
                cached = self._cache.get(t)
                if cached and Path(cached).exists():
                    print("[TTS] 오프라인 캐시 재생")
                    try:
                        self._play_wav(cached)
                    except Exception as e2:
                        if self.debug:
                            print(f"[TTS] 캐시 재생 실패: {e2}", file=sys.stderr)
            finally:
                for p in [mp3_path, wav_path]:
                    if p:
                        try:
                            os.remove(p)
                        except Exception:
                            pass


# ══════════════════════════════════════════════
# Beeper (wav 생성 후 pygame 재생)
# ══════════════════════════════════════════════
class Beeper:
    def __init__(self, debug: bool = False, enabled: bool = True, audio_device: int = 0):
        self.debug = debug
        self.enabled = enabled
        self.audio_device = audio_device
        self._ok = False
        self._cache: Dict[Tuple[int,int], str] = {}
        self._tmpdir = tempfile.mkdtemp(prefix="beep_")
        if enabled:
            try:
                import pyaudio as _pyaudio
                self._pa = _pyaudio.PyAudio()
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
            with wave.open(path, "rb") as wf:
                stream = self._pa.open(
                    format=self._pa.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True,
                    output_device_index=self.audio_device,
                )
                data = wf.readframes(1024)
                while data:
                    stream.write(data)
                    data = wf.readframes(1024)
                stream.stop_stream()
                stream.close()
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
            "- '다시 가/계속/재개/그냥 가' → resume_propose, button_action=resume\n"
            "- '종료/그만/끝/멈춰' → abort_propose, button_action=abort\n"
            "- '바꾸고 싶어/다른 곳/변경' (새 목적지 없음) → change_dest_request, button_action=change_ready\n"
            "- 목적지 이름이 발화에 포함된 경우 (예: '삼백 오호', '301호로 바꿔', '그냥 숭실대로', '세종대 부스로') → change_dest_propose, button_action=change_start, destination=해당목적지\n"
            "- 위 어디에도 해당 안 되면 → ask_again\n\n"
            "[중요] PAUSED에서 목적지 이름이 들리면 반드시 change_dest_propose로 처리할 것. ask_again 금지.\n\n"
            "[유사 목적지 매칭 규칙]\n"
            "- 발화가 목록의 목적지와 완전히 일치하지 않아도, 숫자/발음이 유사하면 가장 가까운 목적지로 매칭할 것.\n"
            "- 예: '아이티알씨 이백일' → 'ITRC 201 AI융합연구원 (숭실대)', '삼백십삼' → 'ITRC 313 인공지능반도체시스템 연구센터 (KAIST)'\n"
            "- 목적지 이름 괄호 안 내용은 별칭이다. 예: 'ITRC 217 지능통감융합 연구센터 (KAIST)'는 '카이스트' 또는 '지능통감'으로 불릴 수 있다.\n"
            "- [중요] 발화가 별칭과 일치할 때 해당 별칭을 가진 목적지가 2개 이상이면 반드시 action=disambiguate로 처리하고 candidates에 전부 넣어라. propose 금지.\n"
            "  예: '카이스트'라 하면 목록에 KAIST가 포함된 목적지가 여러 개이므로 → action=disambiguate, candidates=['ITRC 217 지능통감융합 연구센터 (KAIST)', 'ITRC 301 XR 워크스테이션 HCI 기술 연구센터 (KAIST)', 'ITRC 313 인공지능반도체시스템 연구센터 (KAIST)']\n"
            "- [중요] destination 필드에는 반드시 목록에 있는 정확한 전체 이름을 그대로 써야 한다. "
            "예: 사용자가 '숭실대'라고 해도 목록에 'ITRC 201 AI융합연구원 (숭실대)'이 있으면 destination='ITRC 201 AI융합연구원 (숭실대)'로 반환.\n"
            "- 단, 후보가 2개 이상으로 애매하면 disambiguate 또는 ask_again 사용.\n\n"
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
            raise RuntimeError(str(e))  # plan() 호출부에서 _offline_plan으로 폴백

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
        tts:       TTS,
        beeper:    Beeper,
        gate:      AudioGate,
        debug:     bool = False,
        mic_index: Optional[int] = 5,
    ):
        self.tts       = tts
        self.beeper    = beeper
        self.gate      = gate
        self.debug     = debug
        self.mic_index = mic_index

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
            mic = sr.Microphone(device_index=self.mic_index)
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

        # STT (§9.3) — 온라인: Google / 오프라인 강제: vosk 직행
        def _vosk_stt(audio_data) -> Optional[str]:
            if _get_vosk_model() is None or _vosk_rec_instance is None:
                return None
            try:
                import json
                with _vosk_rec_lock:
                    _vosk_rec_instance.Reset()
                    raw = audio_data.get_raw_data(convert_rate=16000, convert_width=2)
                    _vosk_rec_instance.AcceptWaveform(raw)
                    result = json.loads(_vosk_rec_instance.FinalResult()).get("text", "").strip()
                if result:
                    print(f"[MIC-vosk] {result}")
                return result or None
            except Exception as e2:
                if self.debug:
                    print(f"[MIC] vosk 실패: {e2}", file=sys.stderr)
                return None

        if _force_offline:
            text_result = _vosk_stt(audio)
            if not text_result:
                return "", "empty"
            text = text_result
        else:
            try:
                text = self._r.recognize_google(audio, language="ko-KR").strip()
            except sr.UnknownValueError:
                return "", "empty"
            except Exception as e:
                if self.debug:
                    print(f"[MIC] Google STT 실패: {e}", file=sys.stderr)
                text_result = _vosk_stt(audio)
                if not text_result:
                    return "", "empty"
                text = text_result

        if not text:
            return "", "empty"
        if self._is_dup(text):
            return None, "dup"
        print(f"[MIC] {text}")
        return text, "ok"


# ══════════════════════════════════════════════
# ROS2 Navigation (GuidanceStateMachine + NavigationClient)
# ══════════════════════════════════════════════
# rclpy / ROS2 import – 없으면 시뮬레이션 모드로 폴백
try:
    import rclpy as _rclpy
    from rclpy.executors import SingleThreadedExecutor as _SingleThreadedExecutor
    from rclpy.node import Node as _Node
    from geometry_msgs.msg import Twist as _Twist
    from action_msgs.srv import CancelGoal as _CancelGoal
    from action_msgs.msg import GoalInfo as _GoalInfo
    from unique_identifier_msgs.msg import UUID as _UUID
    from builtin_interfaces.msg import Time as _BuiltinTime
    _ROS_OK = True
except ImportError:
    _ROS_OK = False


class GuidanceStateMachine:
    """
    ROS2가 있으면 NavigationClient로 실제 로봇 제어.
    없으면 NavSimulator(20초 타이머)로 폴백.
    interface.py의 _go_nav / _go_paused / _go_locked 에서 직접 호출.
    """
    def __init__(self, on_arrive: Callable[[], None],
                 on_turn: Callable[[str], None],
                 on_obstacle: Callable[[], None] = lambda: None,
                 on_nav_failed: Callable[[], None] = lambda: None,
                 sim_duration: float = NAV_SIM_DURATION_SEC,
                 debug: bool = False):
        self._on_arrive     = on_arrive
        self._on_turn       = on_turn
        self._on_obstacle   = on_obstacle
        self._on_nav_failed = on_nav_failed
        self._sim_duration = sim_duration
        self._debug      = debug
        self._ros        = _ROS_OK

        # ── 공통 상태 ──
        self._lock       = threading.Lock()
        self._nav_node   = None          # NavigationClient 인스턴스
        self._executor   = None
        self._spin_thr   = None
        self._arrive_thr: Optional[threading.Thread] = None
        self._stop_arrive = threading.Event()

        # ── 시뮬레이션 타이머 ──
        self._sim_timer: Optional[threading.Timer] = None

        if self._ros:
            if not _rclpy.ok():
                _rclpy.init()
            self._executor = _SingleThreadedExecutor()
            self._spin_thr = threading.Thread(
                target=self._executor.spin, daemon=True)
            self._spin_thr.start()
            if debug:
                print("[NAV] ROS2 모드")
        else:
            print("[NAV] ROS2 없음 – 시뮬레이션 모드")

    # ── public API ────────────────────────────
    def start(self, dest_key: str) -> bool:
        self.stop()
        if self._ros:
            return self._ros_start(dest_key)
        else:
            return self._sim_start(dest_key)

    def stop(self) -> None:
        """정지: goal cancel + stop 퍼블리시 + 노드 정리"""
        self._stop_arrive.set()
        if self._ros:
            self._ros_stop()
        else:
            self._sim_stop()
        self._stop_arrive = threading.Event()   # 다음 start를 위해 리셋

    def shutdown(self) -> None:
        self.stop()
        if self._ros and self._executor:
            try:
                self._executor.shutdown()
            except Exception:
                pass

    # ── ROS2 경로 ─────────────────────────────
    def _ros_start(self, dest_key: str) -> bool:
        try:
            # NavigationClient import (같은 디렉터리)
            import importlib, sys as _sys
            _mod_name = "navigation_client"
            if _mod_name not in _sys.modules:
                import importlib.util, pathlib
                _p = pathlib.Path(__file__).resolve().parent / "navigation_client.py"
                spec = importlib.util.spec_from_file_location(_mod_name, str(_p))
                mod  = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                _sys.modules[_mod_name] = mod
            NavClient = _sys.modules[_mod_name].NavigationClient

            self._nav_node = NavClient(dest_key)
            # ROS2 subscription은 등록 시점에 콜백이 바인딩되므로
            # 기존 subscription을 destroy하고 새로 등록해야 패치가 적용됨
            _on_turn     = self._on_turn
            _on_obstacle = self._on_obstacle
            _nav         = self._nav_node
            _nav.last_stuck_announce = 0.0
            _nav.still_since         = None
            try:
                _nav.destroy_subscription(_nav.vel_sub)
            except Exception:
                pass
            def _patched_vel(msg):
                import time as _t
                now = _t.time()
                az  = msg.angular.z
                lx  = msg.linear.x
                # 회전 안내 (기존)
                if abs(az) > _nav.turn_threshold:
                    if now - _nav.last_turn_announcement > 5.0:
                        direction = "왼쪽" if az > 0 else "오른쪽"
                        _on_turn(direction)
                        _nav.last_turn_announcement = now
                # 장애물 감지: 속도가 3초 이상 거의 0 → "비켜주세요" 안내
                if abs(lx) < 0.02 and abs(az) < 0.05:
                    if _nav.still_since is None:
                        _nav.still_since = now
                    elif now - _nav.still_since >= 3.0:
                        if now - _nav.last_stuck_announce > 30.0:
                            _on_obstacle()
                            _nav.last_stuck_announce = now
                        _nav.still_since = None
                else:
                    _nav.still_since = None
            from geometry_msgs.msg import Twist as _TwistMsg
            _nav.vel_sub = _nav.create_subscription(
                _TwistMsg, "/stretch/cmd_vel", _patched_vel, 10)

            self._executor.add_node(self._nav_node)
            ok = bool(self._nav_node.start_navigation())
            if not ok:
                self._ros_cleanup_node()
                return False

            print(f"[NAV] ROS2 이동 시작 → {dest_key}")
            # 도착 감지 스레드
            stop_ev = self._stop_arrive
            def _watch():
                while not stop_ev.is_set():
                    if getattr(self._nav_node, "is_arrived", False):
                        self._on_arrive()
                        return
                    if getattr(self._nav_node, "nav_failed", False):
                        self._on_nav_failed()   # 1-B: Nav2 abort → 큐 경유 안내
                        return
                    threading.Event().wait(0.1)
            self._arrive_thr = threading.Thread(target=_watch, daemon=True)
            self._arrive_thr.start()
            return True

        except Exception as e:
            # 실패는 무조건 로그에 — debug 플래그 뒤에 숨기면 "안내를 시작하지
            # 못했습니다"의 원인을 영영 알 수 없음 (2026-07-24 실측)
            import traceback
            print(f"[NAV-ERR] ROS2 start 실패: {type(e).__name__}: {e}", flush=True)
            traceback.print_exc(file=__import__("sys").stdout)
            return False

    def _ros_stop(self) -> None:
        try:
            if self._nav_node:
                self._nav_node.cleanup()
        except Exception:
            pass
        self._ros_cleanup_node()

    def _ros_cleanup_node(self) -> None:
        if self._nav_node and self._executor:
            try:
                self._executor.remove_node(self._nav_node)
            except Exception:
                pass
            try:
                self._nav_node.destroy_node()
            except Exception:
                pass
        self._nav_node = None

    # ── 시뮬레이션 경로 ───────────────────────
    def _sim_start(self, dest_key: str) -> bool:
        print(f"[NAV] 시뮬 이동 시작 → {dest_key} ({self._sim_duration:.0f}초 후 도착)")
        stop_ev = self._stop_arrive
        def _fire():
            if not stop_ev.is_set():
                self._on_arrive()
        self._sim_timer = threading.Timer(self._sim_duration, _fire)
        self._sim_timer.daemon = True
        self._sim_timer.start()
        return True

    def _sim_stop(self) -> None:
        if self._sim_timer:
            self._sim_timer.cancel()
            self._sim_timer = None
        print("[NAV] 시뮬 이동 정지")


# ══════════════════════════════════════════════
# 하드웨어 브리지 (시리얼 버튼 / Pull 센서)
# ══════════════════════════════════════════════
_GRIP_ARM   = 3731
_PULL_TRIG  = 4095
_QUICK_SEC  = 0.80
_GRIP_RESET = 3158
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
                            tag = parts[0].strip()

                            # TRIG,<btn>,... → 버튼 이벤트 (btn==1만 처리)
                            if tag == "TRIG":
                                if len(parts) >= 2 and parts[1].strip() == "1":
                                    self._debounced("o")
                                continue

                            # DATA,s1,s2,pressure
                            if tag == "DATA" and len(parts) >= 4:
                                try:
                                    press_val = int(parts[3].strip())
                                except Exception:
                                    continue
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
        hw_port:        str   = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0",
        hw_baud:        int   = 115200,
        model:          str   = "gpt-4o-mini",
        mic_index:      int   = 5,
        audio_device:   int   = 0,
    ):
        self.debug          = debug
        self.locations_path = locations_path
        self.locations      = load_locations(locations_path)

        # 출력
        self.tts    = TTS(debug=debug, enabled=not no_tts, audio_device=audio_device)
        self.beeper = Beeper(debug=debug, enabled=not no_tts, audio_device=audio_device)
        self.gate   = AudioGate()
        # 고정 메시지 오프라인 캐시 예열 (백그라운드)
        self.tts.prime_cache(list(MSG.values()) + _EXTRA_CACHE_TEXTS)
        self.no_mic = no_mic

        # GPT
        api_key = load_openai_key(env_path)
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY 없음 – .env 파일 또는 환경변수를 설정하세요.")
        self.gpt = GPTPlanner(api_key=api_key, model=model, debug=debug)

        # 마이크 세션
        self.mic_session = MicSession(tts=self.tts, beeper=self.beeper, gate=self.gate, debug=debug, mic_index=mic_index)

        # 내비게이션 (ROS2 있으면 실제, 없으면 시뮬레이션)
        self.nav = GuidanceStateMachine(
            on_arrive=self._on_arrive,
            on_turn=self._on_turn_announce,
            on_obstacle=self._on_obstacle_announce,
            on_nav_failed=self._on_nav_failed,
            sim_duration=NAV_SIM_DURATION_SEC,
            debug=debug,
        )

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

        # 마이크 세션 취소 토큰 (새 세션 시작 시 이전 스레드를 무효화)
        self._mic_cancel = threading.Event()

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

    def _alias_candidates(self, text: str) -> List[str]:
        """발화에 포함된 별칭이 2개 이상의 목적지와 일치하면 해당 후보 목록 반환."""
        alias_map: Dict[str, List[str]] = {}
        for dest in self.destinations():
            m = re.search(r'\((.+?)\)', dest)
            if m:
                alias_map.setdefault(m.group(1), []).append(dest)
        for alias, dests in alias_map.items():
            if alias in text and len(dests) > 1:
                return dests
        return []

    def _say(self, text: str) -> None:
        """TTS 재생 + AudioGate 차단. TTS는 동기이므로 재생 끝나면 바로 gate 해제."""
        # 말하는 동안 gate 차단 (대략 추산: 글자당 0.09초 + 여유)
        est = max(1.5, len(text) * 0.09 + 1.0)
        self.gate.block(est)
        self.tts.say(text)
        # TTS 잔향이 마이크에 잡히지 않도록 충분한 쿨다운
        self.gate.block(1.5)

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
    # 도착 / 회전 콜백 (GuidanceStateMachine → 메인 스레드)
    # ──────────────────────────────────────────
    def _on_arrive(self) -> None:
        self._ev_q.put(Event(kind="arrive"))

    def _on_nav_failed(self) -> None:
        # ROS/watch 스레드에서 호출 → TTS 직접 금지, 큐 경유 (메인 스레드가 처리)
        self._ev_q.put(Event(kind="nav_failed"))

    def _on_turn_announce(self, direction: str) -> None:
        """NAV 상태에서만 회전 안내 (vel_callback 스레드에서 호출되므로 큐 경유)"""
        if self.state == State.NAV:
            self._ev_q.put(Event(kind="turn", text=direction))

    def _on_obstacle_announce(self) -> None:
        """NAV 중 로봇이 멈췄을 때 장애물 안내 (vel_callback 스레드 → 큐 경유)"""
        if self.state == State.NAV:
            self._ev_q.put(Event(kind="obstacle"))

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
            elif s == "/cancel":
                self._ev_q.put(Event(kind="cancel"))
            elif s == "/backup":
                self._ev_q.put(Event(kind="backup_warn"))
            elif s.startswith("/goto "):
                # 대시보드 장소 클릭 — 정확한 목적지명으로 즉시 출발 (GPT·마이크 생략)
                self._ev_q.put(Event(kind="goto", text=s[6:].strip()))
            elif s in ("/offline", "/online"):
                global _force_offline
                _force_offline = (s == "/offline")
                print("[MODE] 오프라인 강제 모드 ON (vosk + difflib)"
                      if _force_offline else
                      "[MODE] 온라인 모드 복원 (Google STT + GPT)")
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
        새로 호출될 때마다 이전 스레드를 취소 토큰으로 무효화.
        """
        if self.no_mic:
            return

        # 이전 세션 취소 – 새 Event 객체로 교체
        self._mic_cancel.set()
        cancel = threading.Event()
        self._mic_cancel = cancel

        def worker():
            for session_idx in range(NO_RESPONSE_MAX_SESSIONS):
                if cancel.is_set():
                    return  # 새 세션이 시작됐으면 조용히 종료

                # 리마인드 (2차, 3차 수음 전)
                if session_idx == 1:
                    self._say(MSG[19])
                elif session_idx == 2:
                    self._say(MSG[20])

                if cancel.is_set():
                    return

                text, status = self.mic_session.listen()

                if cancel.is_set():
                    return  # 수음 중 새 세션이 시작됐으면 결과 버림

                if status == "ok" and text:
                    self._ev_q.put(Event(kind="text", text=text))
                    return
                elif status == "empty":
                    self._ev_q.put(Event(kind="stt_empty"))
                    return
                elif status == "dup":
                    continue
                else:
                    # timeout → 다음 세션으로
                    continue

            if not cancel.is_set():
                self._ev_q.put(Event(kind="stt_timeout"))

        threading.Thread(target=worker, daemon=True).start()

    # ──────────────────────────────────────────
    # 상태 전이 헬퍼
    # ──────────────────────────────────────────
    def _cancel_mic(self) -> None:
        """진행 중인 마이크 세션을 즉시 취소 (마이크가 필요 없는 상태로 전환 시 호출)."""
        self._mic_cancel.set()
        self._mic_cancel = threading.Event()

    def _go_locked(self, reason: str = "") -> None:
        self._cancel_mic()
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
        self._cancel_mic()          # READY_CONFIRM 등에서 열린 마이크 세션 취소
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
        # MSG[8] TTS 잔향이 마이크에 잡히지 않도록 추가 쿨다운
        self.gate.block(1.2)
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
        # TTS 재생 중이면 버튼 무시 (AudioGate가 닫혀있는 동안)
        if not self.gate.is_open():
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
    def _handle_goto(self, name: str) -> None:
        """대시보드 장소 클릭 → 즉시 출발.
        ★ 임시방편 (2026-07-24 사용자 결정): 개발 효율을 위해 확인 절차를 생략.
        정식 설계는 "되묻기"가 맞음 — 최종판에서는 msg5 확인(READY_CONFIRM)으로
        복원할 것: self._say(msg5(name)); self._go_ready_confirm(name)
        이름이 목록과 정확히 일치할 때만 동작 — GPT·별칭 해석·마이크 전부 생략.
        (음성 흐름의 확인 절차는 그대로 유지 — 이 직행 통로는 클릭 전용)"""
        if name not in self.destinations():
            print(f"[GOTO] 목록에 없는 목적지 무시: {name}", flush=True)
            return
        if self.state == State.NAV:
            self.nav.stop()   # 주행 중 클릭 = 목적지 즉시 변경
        self._go_nav(name)

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
        # GPT 전 선처리: 별칭이 여러 목적지와 겹치면 바로 disambiguate
        conflicts = self._alias_candidates(text)
        if conflicts:
            self.ready_retry += 1
            if self.ready_retry > READY_RETRY_MAX:
                self._say(MSG[18])
                self._go_locked()
                return
            self._say(msg4(conflicts))
            self._go_ready_disambiguate()
            return

        # 별칭이 정확히 1개 목적지에만 매칭 → GPT 없이 바로 확인
        alias_map: Dict[str, List[str]] = {}
        for dest in self.destinations():
            m = re.search(r'\((.+?)\)', dest)
            if m:
                alias_map.setdefault(m.group(1), []).append(dest)
        for alias, dests in alias_map.items():
            if alias in text and len(dests) == 1:
                self._say(msg5(dests[0]))
                self._go_ready_confirm(dests[0])
                return

        if _force_offline:
            plan = _offline_plan(text, self.destinations(), "READY")
        else:
            try:
                plan = self.gpt.plan(
                    state_name="READY",
                    locations=self.destinations(),
                    user_text=text,
                )
            except Exception as e:
                print(f"[GPT-ERR] {e} → 오프라인 매칭 사용", file=sys.stderr)
                plan = _offline_plan(text, self.destinations(), "READY")

        action     = plan.get("action", "noop")
        dest       = plan.get("destination")
        candidates = plan.get("candidates", [])
        reason     = plan.get("reason", "none")

        if action in ("propose", "change_dest_propose") and dest and dest in self.destinations():
            self._say(msg5(dest))
            self._go_ready_confirm(dest)

        elif action in ("propose", "change_dest_propose") and dest and dest not in self.destinations():
            # 목적지 이름은 인식됐지만 목록에 없음
            self.ready_retry += 1
            if self.ready_retry > READY_RETRY_MAX:
                self._say(MSG[18])
                self._go_locked()
                return
            self._say(MSG[3])   # "현재 갈 수 있는 장소가 아닙니다."
            self._run_mic_sessions("ready")

        elif action == "disambiguate":
            # GPT가 단일 후보를 disambiguate로 반환한 경우 → propose로 처리
            if len(candidates) == 1 and candidates[0] in self.destinations():
                self._say(msg5(candidates[0]))
                self._go_ready_confirm(candidates[0])
                return
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
        if _force_offline:
            plan = _offline_plan(text, self.destinations(), "PAUSED")
        else:
            try:
                plan = self.gpt.plan(
                    state_name="PAUSED",
                    locations=self.destinations(),
                    user_text=text,
                )
            except Exception as e:
                print(f"[GPT-ERR] {e} → 오프라인 매칭 사용", file=sys.stderr)
                plan = _offline_plan(text, self.destinations(), "PAUSED")

        action = plan.get("action", "noop")
        dest   = plan.get("destination")

        if action == "resume_propose":
            self._go_paused_confirm("resume", self.last_target, MSG[9])

        elif action == "abort_propose":
            self._go_paused_confirm("abort", None, MSG[10])

        elif action == "change_dest_request":
            self._go_paused_confirm("change_ready", None, MSG[11])

        elif action == "change_dest_propose":
            if dest and dest in self.destinations():
                self._go_paused_confirm("change_start", dest, msg12(dest))
            else:
                # 목적지가 목록에 없음 → 안내 후 재입력
                self.paused_retry += 1
                if self.paused_retry > PAUSED_RETRY_MAX:
                    self._say(MSG[18])
                    self._go_locked()
                    return
                self._say(MSG[3])   # "현재 갈 수 있는 장소가 아닙니다."
                self._run_mic_sessions("paused")

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

    def _handle_nav_failed(self) -> None:
        # 1-B (§14.3 nav_runtime_failed): 주행 중 abort → 안내 후 LOCKED.
        # 상태 가드 필수 — 일시정지(PAUSED) 직후 늦게 온 abort는 무시해야
        # "멈췄는데 갑자기 종료 멘트"가 안 나온다.
        if self.state != State.NAV:
            return
        self.nav.stop()          # 도착감지 스레드(_watch) 종료 + goal 정리
        self.last_target = None
        self._say(MSG[15])       # "안내를 계속할 수 없어 중지했습니다..."
        self._go_locked("주행 실패")

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
            elif ev.kind == "goto" and ev.text:
                self._handle_goto(ev.text)
            elif ev.kind == "stt_empty":
                self._handle_stt_empty()
            elif ev.kind == "stt_timeout":
                self._handle_stt_timeout()
            elif ev.kind == "arrive":
                self._handle_arrive()
            elif ev.kind == "nav_failed":
                self._handle_nav_failed()
            elif ev.kind == "cancel":
                self._go_locked("수동 모드 전환")
            elif ev.kind == "obstacle":
                if self.state == State.NAV:
                    self._say("앞에 장애물이 있습니다. 잠시 비켜주세요.")
            elif ev.kind == "turn" and ev.text:
                # NAV 중 회전 안내 – 메인 스레드에서 안전하게 TTS
                if self.state == State.NAV:
                    self._say(f"{ev.text}으로 회전합니다.")
            elif ev.kind == "backup_warn":
                # 후진 안내 – 상태 무관하게 즉시 TTS
                threading.Thread(
                    target=self.tts.say,
                    args=("잠시 뒤로 이동합니다.",),
                    daemon=True,
                ).start()

    def close(self) -> None:
        self._stop.set()
        self.nav.shutdown()
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
    ap.add_argument("--hw-port",    default="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0", help="시리얼 포트 (기본: by-id 고정 경로)")
    ap.add_argument("--hw-baud",    type=int, default=115200, help="시리얼 보드레이트 (기본: 115200)")
    ap.add_argument("--mic-index",    type=int, default=5, help="마이크 장치 index (기본: 5, ReSpeaker)")
    ap.add_argument("--audio-device", type=int, default=0, help="스피커 출력 장치 index (기본: 0)")
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
        mic_index=args.mic_index,
        audio_device=args.audio_device,
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
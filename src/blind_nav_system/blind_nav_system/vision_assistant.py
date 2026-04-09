#!/usr/bin/env python3
"""
시각 보조 모듈 - "지금 뭐가 보여?" 를 말하면 카메라로 전방을 분석해 음성으로 답변
다른 코드와 독립적으로 실행 가능.

실행:
    python3 vision_assistant.py
    python3 vision_assistant.py --mic-index 5   # ReSpeaker 사용 시
"""

import argparse
import base64
import math
import os
import struct
import subprocess
import sys
import tempfile
import threading
from pathlib import Path

import numpy as np

# ── .env 로드 ─────────────────────────────────────────────────────────────────
_THIS_DIR = Path(__file__).resolve().parent
for _candidate in [
    _THIS_DIR / "../../.env",
    _THIS_DIR / "../../../.env",
    Path.home() / "GitHub/visually-impaired-navigation-robot/src/.env",
]:
    if _candidate.exists():
        from dotenv import load_dotenv
        load_dotenv(_candidate)
        break

OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")
if not OPENAI_API_KEY:
    sys.exit("[오류] OPENAI_API_KEY 없음 – .env 파일을 확인하세요.")

import requests
_OPENAI_HEADERS = {
    "Authorization": f"Bearer {OPENAI_API_KEY}",
    "Content-Type": "application/json",
}

# ── ROS2 + 카메라 ─────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image as ROSImage
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False
    print("[경고] rclpy 없음 – ROS2 카메라를 사용할 수 없습니다.")

# ── 오디오 ────────────────────────────────────────────────────────────────────
try:
    import pyaudio
    import speech_recognition as sr
    _AUDIO_OK = True
except ImportError:
    _AUDIO_OK = False
    print("[경고] pyaudio/speech_recognition 없음 – 마이크 비활성화, stdin 사용.")

# ─────────────────────────────────────────────────────────────────────────────
# 트리거 키워드 (이 중 하나가 들리면 카메라 캡처 + 분석)
TRIGGER_KEYWORDS = ["뭐가 보여", "뭐 보여", "지금 뭐", "what do you see", "what's there"]

# 분석 프롬프트
VISION_PROMPT = (
    "이 이미지에서 보이는 것을 한국어로 2~3문장으로 간결하게 설명하세요. "
    "사람, 장애물, 문, 복도 방향, 계단 등 눈에 띄는 것을 자연스럽게 말해주세요. "
    "안전 판단이나 경로 안내는 하지 않아도 됩니다. 그냥 보이는 장면을 묘사하세요."
)


# ─────────────────────────────────────────────────────────────────────────────
class CameraCapture(Node if _ROS_AVAILABLE else object):
    """최신 카메라 프레임을 보관하는 ROS2 노드."""

    def __init__(self):
        if _ROS_AVAILABLE:
            super().__init__("vision_assistant_capture")
            self._sub = self.create_subscription(
                ROSImage,
                "/camera/camera/color/image_raw",
                self._cb,
                10,
            )
        self._latest: bytes | None = None
        self._lock = threading.Lock()

    def _cb(self, msg: "ROSImage"):
        try:
            # encoding: bgr8 or rgb8
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, -1)
            )
            if msg.encoding in ("bgr8",):
                arr = arr[:, :, ::-1]  # BGR → RGB
            # JPEG 압축 후 base64
            import cv2
            ok, buf = cv2.imencode(".jpg", arr[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                with self._lock:
                    self._latest = base64.b64encode(buf.tobytes()).decode()
        except Exception as e:
            print(f"[카메라] 프레임 처리 오류: {e}")

    def get_frame_b64(self) -> str | None:
        with self._lock:
            return self._latest


# ─────────────────────────────────────────────────────────────────────────────
def analyze_image(b64_jpeg: str) -> str:
    """GPT-4o Vision으로 이미지 분석 후 텍스트 반환."""
    try:
        payload = {
            "model": "gpt-4o",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": VISION_PROMPT},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{b64_jpeg}",
                                "detail": "low",
                            },
                        },
                    ],
                }
            ],
            "max_tokens": 200,
        }
        resp = requests.post(
            "https://api.openai.com/v1/chat/completions",
            headers=_OPENAI_HEADERS,
            json=payload,
            timeout=15,
        )
        resp.raise_for_status()
        return resp.json()["choices"][0]["message"]["content"].strip()
    except Exception as e:
        return f"분석 오류: {e}"


# ─────────────────────────────────────────────────────────────────────────────
def speak(text: str, out_device_index: int = 0):
    """gTTS + pyaudio로 TTS 재생."""
    print(f"[ROBOT] {text}")
    if not _AUDIO_OK:
        return
    try:
        from gtts import gTTS
        tts = gTTS(text=text, lang="ko")
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
            tmp_mp3 = f.name
        tts.save(tmp_mp3)

        # mp3 → raw PCM via ffmpeg
        try:
            from pathlib import Path as _Path
            speed = float(_Path("/tmp/tts_speed").read_text().strip())
            speed = max(0.5, min(2.0, speed))
        except Exception:
            speed = 1.5
        tmp_pcm = tmp_mp3.replace(".mp3", ".raw")
        subprocess.run(
            ["ffmpeg", "-y", "-i", tmp_mp3,
             "-filter:a", f"atempo={speed}",
             "-f", "s16le", "-ar", "44100", "-ac", "2", tmp_pcm],
            check=True, capture_output=True,
        )

        pa = pyaudio.PyAudio()
        stream = pa.open(
            format=pyaudio.paInt16,
            channels=2,
            rate=44100,
            output=True,
            output_device_index=out_device_index,
        )
        with open(tmp_pcm, "rb") as f:
            chunk = f.read(4096)
            while chunk:
                stream.write(chunk)
                chunk = f.read(4096)
        stream.stop_stream()
        stream.close()
        pa.terminate()
    except Exception as e:
        print(f"[TTS 오류] {e}")
    finally:
        for p in [tmp_mp3, tmp_pcm]:
            try:
                os.remove(p)
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────────────────────
def contains_trigger(text: str) -> bool:
    t = text.lower()
    return any(kw in t for kw in TRIGGER_KEYWORDS)


def stdin_loop(camera: CameraCapture, out_device: int):
    """stdin에서 /vision 명령(또는 Enter)을 받으면 카메라 캡처 후 분석.
    main.py에서 버튼2 신호를 /vision으로 전달받거나, 직접 Enter로 테스트 가능."""
    print("[ 버튼2 또는 Enter 키를 누르면 전방을 분석합니다. Ctrl+C로 종료 ]")
    while True:
        try:
            line = sys.stdin.readline()
            if not line:  # EOF
                break
            if line.strip() in ("/vision", ""):
                _do_vision(camera, out_device)
        except (EOFError, KeyboardInterrupt):
            break


def _do_vision(camera: CameraCapture, out_device: int):
    """카메라 캡처 → GPT-4o 분석 → 음성 출력."""
    import time
    print("[VISION] 전방 분석 중...")
    # 최대 5초 대기
    b64 = None
    for _ in range(50):
        b64 = camera.get_frame_b64()
        if b64:
            break
        time.sleep(0.1)
    if b64 is None:
        speak("카메라에서 이미지를 받지 못했습니다.", out_device)
        return
    result = analyze_image(b64)
    speak(result, out_device)


# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="시각 보조 모듈")
    parser.add_argument("--out-index", type=int, default=0,
                        help="스피커 장치 index (기본: 0=내장 스피커)")
    args = parser.parse_args()

    # ROS2 초기화
    if _ROS_AVAILABLE:
        rclpy.init()
        camera = CameraCapture()
        ros_thread = threading.Thread(
            target=lambda: rclpy.spin(camera), daemon=True
        )
        ros_thread.start()
        print("[ROS2] 카메라 구독 시작 (/camera/camera/color/image_raw)")
    else:
        camera = CameraCapture()
        print("[카메라] ROS2 없음 – 이미지 캡처 불가")

    print("=" * 50)
    print("  시각 보조 모듈 시작")
    print(f"  스피커: index {args.out_index}")
    print("=" * 50)

    # 첫 프레임 수신 대기
    import time
    print("[카메라] 첫 프레임 대기 중...", end="", flush=True)
    for _ in range(50):
        if camera.get_frame_b64():
            print(" 연결됨!")
            break
        time.sleep(0.1)
    else:
        print(" 타임아웃 (카메라가 실행 중인지 확인하세요)")

    stdin_loop(camera, args.out_index)


if __name__ == "__main__":
    main()

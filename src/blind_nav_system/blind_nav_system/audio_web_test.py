#!/usr/bin/env python3
"""
오디오 진단 웹 툴
실행: python3 audio_web_test.py
브라우저 자동 오픈 또는 http://localhost:5000 접속
"""
import json
import math
import os
import struct
import subprocess
import tempfile
import threading
import time
import wave
import webbrowser

import pyaudio
from flask import Flask, Response, jsonify, request

pa = pyaudio.PyAudio()
app = Flask(__name__)
app.config['SECRET_KEY'] = 'audio'

# ── 디바이스 목록 ────────────────────────────────

def get_devices():
    inputs, outputs = [], []
    for i in range(pa.get_device_count()):
        try:
            info = pa.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                inputs.append({'index': i, 'name': info['name'],
                                'ch': int(info['maxInputChannels'])})
            if info['maxOutputChannels'] > 0:
                outputs.append({'index': i, 'name': info['name'],
                                 'ch': int(info['maxOutputChannels'])})
        except Exception:
            pass
    return inputs, outputs

def get_pulse_default():
    try:
        r = subprocess.run(['pactl', 'get-default-sink'],
                           capture_output=True, text=True, timeout=3)
        return r.stdout.strip()
    except Exception:
        return '확인 불가'

# ── 마이크 레벨 모니터 ───────────────────────────

mic_levels: dict = {}
mic_streams: dict = {}
mic_lock = threading.Lock()


def _start_mic(idx: int) -> bool:
    def cb(in_data, frame_count, ti, st):
        try:
            samples = struct.unpack(f'<{frame_count}h', in_data)
            rms = math.sqrt(sum(s * s for s in samples) / frame_count)
            with mic_lock:
                mic_levels[idx] = min(100, int(rms / 164))
        except Exception:
            pass
        return (None, pyaudio.paContinue)

    try:
        stream = pa.open(
            format=pyaudio.paInt16, channels=1, rate=16000,
            input=True, input_device_index=idx,
            frames_per_buffer=512, stream_callback=cb,
        )
        stream.start_stream()
        with mic_lock:
            mic_streams[idx] = stream
            mic_levels[idx] = 0
        return True
    except Exception:
        with mic_lock:
            mic_levels[idx] = -1
        return False


def _stop_mic(idx: int):
    with mic_lock:
        s = mic_streams.pop(idx, None)
        mic_levels.pop(idx, None)
    if s:
        try:
            s.stop_stream(); s.close()
        except Exception:
            pass

# ── 440 Hz 톤 재생 ───────────────────────────────

tone_streams: dict = {}
tone_lock = threading.Lock()


def _start_tone(idx: int, volume: float) -> bool:
    _stop_tone(idx)
    RATE, FREQ = 44100, 440
    phase = [0.0]
    vol = max(0.0, min(1.0, volume))

    def cb(in_data, frame_count, ti, st):
        data = bytearray(frame_count * 2)
        for i in range(frame_count):
            v = int(vol * 32767 * math.sin(phase[0]))
            struct.pack_into('<h', data, i * 2, max(-32768, min(32767, v)))
            phase[0] += 2 * math.pi * FREQ / RATE
            if phase[0] > 2 * math.pi:
                phase[0] -= 2 * math.pi
        return (bytes(data), pyaudio.paContinue)

    try:
        stream = pa.open(
            format=pyaudio.paInt16, channels=1, rate=RATE,
            output=True, output_device_index=idx,
            frames_per_buffer=1024, stream_callback=cb,
        )
        stream.start_stream()
        with tone_lock:
            tone_streams[idx] = stream
        return True
    except Exception:
        return False


def _stop_tone(idx: int):
    with tone_lock:
        s = tone_streams.pop(idx, None)
    if s:
        try:
            s.stop_stream(); s.close()
        except Exception:
            pass

# ── TTS 재생 (gTTS → ffmpeg → pyaudio) ──────────

tts_lock = threading.Lock()


def _play_tts(text: str, idx: int, vol: float) -> tuple:
    """gTTS로 음성 생성 후 지정 장치로 재생. (ok, error_msg) 반환"""
    try:
        from gtts import gTTS
    except ImportError:
        return False, 'gTTS 미설치 (pip install gtts)'

    mp3_path = wav_path = None
    try:
        fd, mp3_path = tempfile.mkstemp(suffix='.mp3')
        os.close(fd)
        wav_path = mp3_path.replace('.mp3', '.wav')

        # gTTS → MP3
        gTTS(text=text, lang='ko').save(mp3_path)

        # MP3 → WAV (ffmpeg 필요)
        result = subprocess.run(
            ['ffmpeg', '-y', '-i', mp3_path, '-ar', '44100', '-ac', '1', wav_path],
            capture_output=True, timeout=15
        )
        if result.returncode != 0:
            return False, 'ffmpeg 변환 실패 (ffmpeg 설치 필요)'

        # WAV → pyaudio 재생
        with wave.open(wav_path, 'rb') as wf:
            fmt = pa.get_format_from_width(wf.getsampwidth())
            stream = pa.open(
                format=fmt,
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True,
                output_device_index=idx,
            )
            chunk = 1024
            data = wf.readframes(chunk)
            while data:
                if vol != 1.0:
                    n = len(data) // 2
                    samples = struct.unpack(f'<{n}h', data)
                    data = struct.pack(f'<{n}h',
                                      *[max(-32768, min(32767, int(s * vol))) for s in samples])
                stream.write(data)
                data = wf.readframes(chunk)
            stream.stop_stream()
            stream.close()

        return True, ''
    except Exception as e:
        return False, str(e)
    finally:
        for p in [mp3_path, wav_path]:
            if p:
                try:
                    os.remove(p)
                except Exception:
                    pass

# ── Flask 라우트 ─────────────────────────────────

@app.route('/')
def index():
    return HTML

@app.route('/devices')
def devices():
    inp, out = get_devices()
    return jsonify({'inputs': inp, 'outputs': out,
                    'pulse_default': get_pulse_default()})

@app.route('/mic/start/<int:idx>')
def mic_start(idx):
    return jsonify({'ok': _start_mic(idx)})

@app.route('/mic/stop/<int:idx>')
def mic_stop(idx):
    _stop_mic(idx)
    return jsonify({'ok': True})

@app.route('/mic/levels')
def mic_levels_sse():
    def generate():
        while True:
            with mic_lock:
                data = dict(mic_levels)
            yield f"data: {json.dumps(data)}\n\n"
            time.sleep(0.05)
    return Response(generate(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'})

@app.route('/tone/start/<int:idx>')
def tone_start(idx):
    vol = float(request.args.get('vol', 0.3))
    return jsonify({'ok': _start_tone(idx, vol)})

@app.route('/tone/stop/<int:idx>')
def tone_stop(idx):
    _stop_tone(idx)
    return jsonify({'ok': True})

@app.route('/tts')
def tts_play():
    text = request.args.get('text', '안녕하세요')
    idx  = int(request.args.get('idx', 0))
    vol  = float(request.args.get('vol', 0.9))
    with tts_lock:
        ok, err = _play_tts(text, idx, vol)
    return jsonify({'ok': ok, 'error': err})

# ── HTML ─────────────────────────────────────────

HTML = """<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>오디오 진단</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: 'Segoe UI', sans-serif; background: #1a1a2e; color: #eee; padding: 20px; }
  h1 { text-align: center; margin-bottom: 8px; font-size: 1.4rem; color: #a8d8ea; }
  .pulse-info { text-align:center; font-size:0.78rem; color:#888; margin-bottom:20px; }
  .pulse-info span { color:#facc15; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
  h2 { font-size: 1rem; color: #a8d8ea; margin-bottom: 12px; border-bottom: 1px solid #444; padding-bottom: 6px; }
  .card { background: #16213e; border-radius: 10px; padding: 14px; margin-bottom: 10px; }
  .card-name { font-size: 0.82rem; color: #ccc; margin-bottom: 4px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .card-sub  { font-size: 0.72rem; color: #888; margin-bottom: 8px; }
  .meter-wrap { background: #0f3460; border-radius: 4px; height: 14px; overflow: hidden; margin-bottom: 8px; }
  .meter-bar  { height: 100%; width: 0%; background: linear-gradient(90deg,#4ade80,#facc15,#f87171); transition: width .05s; border-radius: 4px; }
  .meter-val  { font-size: 0.72rem; color: #aaa; margin-bottom: 8px; }
  .btn { border: none; border-radius: 6px; padding: 6px 14px; font-size: 0.8rem; cursor: pointer; transition: opacity .15s; margin-right: 4px; }
  .btn:hover { opacity: 0.8; }
  .btn-green  { background: #4ade80; color: #111; }
  .btn-red    { background: #f87171; color: #111; }
  .btn-blue   { background: #60a5fa; color: #111; }
  .btn-purple { background: #c084fc; color: #111; }
  .btn-sm { padding: 4px 10px; font-size: 0.75rem; }
  .vol-wrap { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
  .vol-wrap label { font-size: 0.75rem; color: #aaa; white-space: nowrap; }
  input[type=range] { flex: 1; accent-color: #60a5fa; }
  .vol-val { font-size: 0.75rem; color: #aaa; width: 34px; text-align: right; }
  .status { font-size: 0.72rem; color: #facc15; min-height: 16px; margin-top: 6px; }
  /* TTS 섹션 */
  .tts-section { background: #0f3460; border-radius: 10px; padding: 16px; margin-bottom: 20px; }
  .tts-section h2 { border-color: #1e5f9e; }
  .tts-row { display: flex; gap: 8px; align-items: center; margin-bottom: 10px; flex-wrap: wrap; }
  .tts-row input[type=text] {
    flex: 1; min-width: 200px; background: #16213e; border: 1px solid #444;
    color: #eee; border-radius: 6px; padding: 6px 10px; font-size: 0.85rem;
  }
  .tts-row select {
    background: #16213e; border: 1px solid #444; color: #eee;
    border-radius: 6px; padding: 6px 8px; font-size: 0.8rem;
  }
  .divider { border: none; border-top: 1px solid #2a2a4a; margin: 16px 0; }
</style>
</head>
<body>
<h1>오디오 진단 툴</h1>
<div class="pulse-info">PulseAudio 기본 출력: <span id="pulse-sink">확인 중...</span></div>

<!-- TTS 테스트 -->
<div class="tts-section">
  <h2>🗣 TTS 재생 테스트</h2>
  <div class="tts-row">
    <input type="text" id="tts-text" value="안녕하세요. 목적지를 말씀해 주세요." placeholder="재생할 텍스트">
    <select id="tts-device"><option value="">장치 로딩 중...</option></select>
    <span style="font-size:0.75rem;color:#aaa;white-space:nowrap">볼륨</span>
    <input type="range" min="0" max="100" value="90" id="tts-vol" style="width:80px"
           oninput="document.getElementById('tts-vol-val').textContent=this.value+'%'">
    <span id="tts-vol-val" style="font-size:0.75rem;color:#aaa;width:34px">90%</span>
    <button class="btn btn-purple" onclick="playTTS()">▶ 재생</button>
  </div>
  <div class="status" id="tts-status"></div>
</div>

<div class="grid">
  <div>
    <h2>🎤 마이크 입력</h2>
    <div id="inputs">불러오는 중...</div>
  </div>
  <div>
    <h2>🔊 스피커 출력 (440 Hz)</h2>
    <div id="outputs">불러오는 중...</div>
  </div>
</div>

<script>
const $ = id => document.getElementById(id);

fetch('/devices').then(r=>r.json()).then(data => {
  $('pulse-sink').textContent = data.pulse_default;
  renderInputs(data.inputs);
  renderOutputs(data.outputs);
  renderTTSDevices(data.outputs);
  startLevelSSE();
});

// ── TTS ────────────────────────────────────────
function renderTTSDevices(devices) {
  $('tts-device').innerHTML = devices.map(d =>
    `<option value="${d.index}">${d.name} (idx ${d.index})</option>`
  ).join('');
}

function playTTS() {
  const text = $('tts-text').value.trim();
  const idx  = $('tts-device').value;
  const vol  = $('tts-vol').value / 100;
  const st   = $('tts-status');
  if (!text) { st.textContent = '⚠ 텍스트를 입력하세요'; return; }
  st.textContent = '생성 중...';
  fetch(`/tts?text=${encodeURIComponent(text)}&idx=${idx}&vol=${vol}`)
    .then(r=>r.json()).then(res => {
      if (res.ok) {
        st.textContent = '✅ 재생 완료';
      } else {
        st.textContent = '⚠ 실패: ' + res.error;
      }
    });
}

// ── 마이크 ────────────────────────────────────
function renderInputs(devices) {
  $('inputs').innerHTML = devices.map(d => `
    <div class="card">
      <div class="card-name" title="${d.name}">${d.name}</div>
      <div class="card-sub">index ${d.index} · ${d.ch}ch</div>
      <div class="meter-wrap"><div class="meter-bar" id="bar-${d.index}"></div></div>
      <div class="meter-val" id="val-${d.index}">level: -</div>
      <button class="btn btn-green btn-sm" id="mic-btn-${d.index}" onclick="toggleMic(${d.index})">▶ 모니터</button>
      <div class="status" id="mic-st-${d.index}"></div>
    </div>`).join('');
}

const micActive = {};
function toggleMic(idx) {
  const btn = $(`mic-btn-${idx}`);
  const st  = $(`mic-st-${idx}`);
  if (micActive[idx]) {
    fetch(`/mic/stop/${idx}`);
    micActive[idx] = false;
    btn.textContent = '▶ 모니터';
    btn.className = 'btn btn-green btn-sm';
    st.textContent = '';
    $(`bar-${idx}`).style.width = '0%';
    $(`val-${idx}`).textContent = 'level: -';
  } else {
    fetch(`/mic/start/${idx}`).then(r=>r.json()).then(res => {
      if (res.ok) {
        micActive[idx] = true;
        btn.textContent = '■ 중지';
        btn.className = 'btn btn-red btn-sm';
        st.textContent = '모니터링 중...';
      } else {
        st.textContent = '⚠ 열기 실패';
      }
    });
  }
}

function startLevelSSE() {
  const es = new EventSource('/mic/levels');
  es.onmessage = e => {
    const levels = JSON.parse(e.data);
    for (const [idxStr, val] of Object.entries(levels)) {
      const idx = parseInt(idxStr);
      const bar = $(`bar-${idx}`);
      const lbl = $(`val-${idx}`);
      if (!bar) continue;
      if (val < 0) {
        lbl.textContent = '⚠ 장치 오류';
      } else {
        bar.style.width = val + '%';
        lbl.textContent = `level: ${val}`;
      }
    }
  };
}

// ── 스피커 ────────────────────────────────────
function renderOutputs(devices) {
  $('outputs').innerHTML = devices.map(d => `
    <div class="card">
      <div class="card-name" title="${d.name}">${d.name}</div>
      <div class="card-sub">index ${d.index} · ${d.ch}ch</div>
      <div class="vol-wrap">
        <label>볼륨</label>
        <input type="range" min="0" max="100" value="30" id="vol-${d.index}"
               oninput="$('vval-${d.index}').textContent=this.value+'%'">
        <span class="vol-val" id="vval-${d.index}">30%</span>
      </div>
      <button class="btn btn-blue btn-sm" id="tone-btn-${d.index}" onclick="toggleTone(${d.index})">▶ 440Hz</button>
      <div class="status" id="tone-st-${d.index}"></div>
    </div>`).join('');
}

const toneActive = {};
function toggleTone(idx) {
  const btn = $(`tone-btn-${idx}`);
  const st  = $(`tone-st-${idx}`);
  if (toneActive[idx]) {
    fetch(`/tone/stop/${idx}`);
    toneActive[idx] = false;
    btn.textContent = '▶ 440Hz';
    btn.className = 'btn btn-blue btn-sm';
    st.textContent = '';
  } else {
    const vol = $(`vol-${idx}`).value / 100;
    fetch(`/tone/start/${idx}?vol=${vol}`).then(r=>r.json()).then(res => {
      if (res.ok) {
        toneActive[idx] = true;
        btn.textContent = '■ 정지';
        btn.className = 'btn btn-red btn-sm';
        st.textContent = '440Hz 재생 중...';
      } else {
        st.textContent = '⚠ 재생 실패';
      }
    });
  }
}
</script>
</body>
</html>"""

# ── 진입점 ───────────────────────────────────────

if __name__ == '__main__':
    threading.Timer(1.2, lambda: webbrowser.open('http://localhost:5000')).start()
    print("오디오 진단 서버: http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)

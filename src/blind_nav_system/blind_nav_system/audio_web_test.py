#!/usr/bin/env python3
"""
오디오 진단 웹 툴
실행: python3 audio_web_test.py
브라우저 자동 오픈 또는 http://localhost:5000 접속
"""
import json
import math
import struct
import threading
import time
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
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            input_device_index=idx,
            frames_per_buffer=512,
            stream_callback=cb,
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
            format=pyaudio.paInt16,
            channels=1,
            rate=RATE,
            output=True,
            output_device_index=idx,
            frames_per_buffer=1024,
            stream_callback=cb,
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

# ── Flask 라우트 ─────────────────────────────────

@app.route('/')
def index():
    return HTML

@app.route('/devices')
def devices():
    inp, out = get_devices()
    return jsonify({'inputs': inp, 'outputs': out})

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

# ── HTML ─────────────────────────────────────────

HTML = """<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>오디오 진단</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: 'Segoe UI', sans-serif; background: #1a1a2e; color: #eee; padding: 20px; }
  h1 { text-align: center; margin-bottom: 24px; font-size: 1.4rem; color: #a8d8ea; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
  h2 { font-size: 1rem; color: #a8d8ea; margin-bottom: 12px; border-bottom: 1px solid #444; padding-bottom: 6px; }
  .card { background: #16213e; border-radius: 10px; padding: 14px; margin-bottom: 10px; }
  .card-name { font-size: 0.82rem; color: #ccc; margin-bottom: 8px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .card-sub  { font-size: 0.72rem; color: #888; margin-bottom: 8px; }
  /* 마이크 레벨 바 */
  .meter-wrap { background: #0f3460; border-radius: 4px; height: 14px; overflow: hidden; margin-bottom: 8px; }
  .meter-bar  { height: 100%; width: 0%; background: linear-gradient(90deg,#4ade80,#facc15,#f87171); transition: width .05s; border-radius: 4px; }
  .meter-val  { font-size: 0.72rem; color: #aaa; margin-bottom: 8px; }
  /* 버튼 */
  .btn { border: none; border-radius: 6px; padding: 6px 14px; font-size: 0.8rem; cursor: pointer; transition: opacity .15s; }
  .btn:hover { opacity: 0.8; }
  .btn-green { background: #4ade80; color: #111; }
  .btn-red   { background: #f87171; color: #111; }
  .btn-blue  { background: #60a5fa; color: #111; }
  /* 볼륨 슬라이더 */
  .vol-wrap { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
  .vol-wrap label { font-size: 0.75rem; color: #aaa; white-space: nowrap; }
  input[type=range] { flex: 1; accent-color: #60a5fa; }
  .vol-val { font-size: 0.75rem; color: #aaa; width: 34px; text-align: right; }
  .status { font-size: 0.72rem; color: #facc15; min-height: 14px; }
</style>
</head>
<body>
<h1>오디오 진단 툴</h1>
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

// ── 디바이스 로드 ──────────────────────────────
fetch('/devices').then(r=>r.json()).then(data => {
  renderInputs(data.inputs);
  renderOutputs(data.outputs);
  startLevelSSE();
});

// ── 마이크 카드 렌더 ───────────────────────────
function renderInputs(devices) {
  $('inputs').innerHTML = devices.map(d => `
    <div class="card" id="mic-card-${d.index}">
      <div class="card-name" title="${d.name}">${d.name}</div>
      <div class="card-sub">index ${d.index} · ${d.ch}ch</div>
      <div class="meter-wrap"><div class="meter-bar" id="bar-${d.index}"></div></div>
      <div class="meter-val" id="val-${d.index}">level: -</div>
      <button class="btn btn-green" id="mic-btn-${d.index}" onclick="toggleMic(${d.index})">▶ 모니터 시작</button>
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
    btn.textContent = '▶ 모니터 시작';
    btn.className = 'btn btn-green';
    st.textContent = '';
    $(`bar-${idx}`).style.width = '0%';
    $(`val-${idx}`).textContent = 'level: -';
  } else {
    fetch(`/mic/start/${idx}`).then(r=>r.json()).then(res => {
      if (res.ok) {
        micActive[idx] = true;
        btn.textContent = '■ 중지';
        btn.className = 'btn btn-red';
        st.textContent = '모니터링 중...';
      } else {
        st.textContent = '⚠ 열기 실패 (다른 앱이 사용 중?)';
      }
    });
  }
}

// ── SSE 레벨 수신 ──────────────────────────────
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

// ── 스피커 카드 렌더 ───────────────────────────
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
      <button class="btn btn-blue" id="tone-btn-${d.index}" onclick="toggleTone(${d.index})">▶ 440Hz 재생</button>
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
    btn.textContent = '▶ 440Hz 재생';
    btn.className = 'btn btn-blue';
    st.textContent = '';
  } else {
    const vol = $(`vol-${idx}`).value / 100;
    fetch(`/tone/start/${idx}?vol=${vol}`).then(r=>r.json()).then(res => {
      if (res.ok) {
        toneActive[idx] = true;
        btn.textContent = '■ 정지';
        btn.className = 'btn btn-red';
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

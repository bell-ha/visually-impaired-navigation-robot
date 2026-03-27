from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit
import serial
import threading
import time

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0'
BAUD = 115200

thresholds = {
    'GRIP_ARM':   3731,
    'PULL_TRIG':  4095,
    'QUICK_SEC':  0.80,
    'GRIP_RESET': 3158,
}

state = {
    'armed': False,
    'armed_time': 0.0,
    'triggered_this_arm': False,
    'pull_count': 0,
}

def pull_bang(val):
    now = time.monotonic()
    t = thresholds
    s = state

    if val < t['GRIP_RESET']:
        s['armed'] = False
        s['triggered_this_arm'] = False
        return None

    if not s['armed'] and val >= t['GRIP_ARM']:
        s['armed'] = True
        s['armed_time'] = now
        s['triggered_this_arm'] = False
        return None

    if s['armed']:
        dt = now - s['armed_time']
        if s['triggered_this_arm']:
            return None
        if val >= t['PULL_TRIG'] and dt <= t['QUICK_SEC']:
            s['triggered_this_arm'] = True
            s['pull_count'] += 1
            return ('PULL', s['pull_count'], dt)
        if val >= t['PULL_TRIG'] and dt > t['QUICK_SEC']:
            s['triggered_this_arm'] = True
            return ('SLOW', dt)

    return None

def serial_reader():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"[OK] 시리얼 연결: {PORT}")
    except Exception as e:
        print(f"[FAIL] 시리얼 연결 실패: {e}")
        return

    buf = ""
    while True:
        try:
            buf += ser.read(ser.in_waiting or 1).decode('utf-8', errors='ignore')
        except:
            break

        if '\n' not in buf:
            continue

        lines = buf.split('\n')
        buf = lines.pop()

        for line in lines:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            tag = parts[0] if parts else ''

            press_val = None
            btn1, btn2 = 0, 0

            try:
                if tag == 'TRIG':
                    btn_num = int(parts[1])
                    press_val = int(parts[2])
                    btn1 = 1 if btn_num == 1 else 0
                    btn2 = 1 if btn_num == 2 else 0
                elif tag == 'DATA':
                    btn1 = int(parts[1])
                    btn2 = int(parts[2])
                    press_val = int(parts[3])
            except:
                continue

            if press_val is None:
                continue

            evt = pull_bang(press_val)
            event_label = None
            event_dt = None
            if evt:
                event_label = evt[0]
                event_dt = round(evt[1] * 1000) if evt[0] == 'SLOW' else round(evt[2] * 1000)

            socketio.emit('data', {
                'pressure':    press_val,
                'btn1':        btn1,
                'btn2':        btn2,
                'armed':       state['armed'],
                'event':       event_label,
                'event_dt':    event_dt,
                'pull_count':  state['pull_count'],
                'grip_arm':    thresholds['GRIP_ARM'],
                'pull_trig':   thresholds['PULL_TRIG'],
                'grip_reset':  thresholds['GRIP_RESET'],
            })

@socketio.on('update_thresholds')
def handle_thresholds(data):
    thresholds['GRIP_ARM']   = int(float(data.get('GRIP_ARM',   thresholds['GRIP_ARM'])))
    thresholds['PULL_TRIG']  = int(float(data.get('PULL_TRIG',  thresholds['PULL_TRIG'])))
    thresholds['QUICK_SEC']  = float(data.get('QUICK_SEC',  thresholds['QUICK_SEC']))
    thresholds['GRIP_RESET'] = int(float(data.get('GRIP_RESET', thresholds['GRIP_RESET'])))
    print(f"[임계값 변경] {thresholds}")

HTML = """
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>압력 센서 테스트</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<script src="https://cdn.jsdelivr.net/npm/chartjs-plugin-annotation@3"></script>
<script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: #111; color: #eee; font-family: monospace; padding: 20px; }
  h1 { font-size: 1.2rem; margin-bottom: 16px; color: #adf; }

  .layout { display: flex; gap: 20px; align-items: flex-start; }
  .chart-area { flex: 1; }
  canvas { background: #1a1a1a; border-radius: 8px; }

  .panel { width: 280px; display: flex; flex-direction: column; gap: 12px; }

  .card { background: #1e1e1e; border-radius: 8px; padding: 14px; }
  .card h2 { font-size: 0.85rem; color: #888; margin-bottom: 10px; text-transform: uppercase; letter-spacing: 1px; }

  .slider-row { margin-bottom: 10px; }
  .slider-row label { display: flex; justify-content: space-between; font-size: 0.85rem; margin-bottom: 4px; }
  .slider-row label span { color: #7ef; font-weight: bold; }
  input[type=range] { width: 100%; accent-color: #7ef; }

  .status-row { display: flex; justify-content: space-between; font-size: 0.9rem; padding: 4px 0; border-bottom: 1px solid #333; }
  .status-row:last-child { border-bottom: none; }
  .val { color: #7ef; font-weight: bold; }
  .armed-y { color: #f90; }
  .armed-n { color: #555; }

  .log { height: 160px; overflow-y: auto; font-size: 0.8rem; }
  .log-item { padding: 3px 0; border-bottom: 1px solid #222; }
  .log-item.pull { color: #4f4; }
  .log-item.slow { color: #fa0; }
  .log-item.btn  { color: #7ef; }
</style>
</head>
<body>
<h1>압력 센서 실시간 모니터</h1>
<div class="layout">
  <div class="chart-area">
    <canvas id="chart" height="100"></canvas>
  </div>
  <div class="panel">

    <div class="card">
      <h2>상태</h2>
      <div class="status-row"><span>압력값</span><span class="val" id="s-pressure">-</span></div>
      <div class="status-row"><span>당김 횟수</span><span class="val" id="s-pull">0</span></div>
      <div class="status-row"><span>감시중(armed)</span><span id="s-armed" class="armed-n">NO</span></div>
      <div class="status-row"><span>버튼1</span><span class="val" id="s-btn1">0</span></div>
      <div class="status-row"><span>버튼2</span><span class="val" id="s-btn2">0</span></div>
    </div>

    <div class="card">
      <h2>임계값 조절</h2>
      <div class="slider-row">
        <label>GRIP_ARM <span id="v-grip-arm">3731</span></label>
        <input type="range" id="grip-arm" min="0" max="4095" value="3731">
      </div>
      <div class="slider-row">
        <label>PULL_TRIG <span id="v-pull-trig">4095</span></label>
        <input type="range" id="pull-trig" min="0" max="4095" value="4095">
      </div>
      <div class="slider-row">
        <label>GRIP_RESET <span id="v-grip-reset">3158</span></label>
        <input type="range" id="grip-reset" min="0" max="4095" value="3158">
      </div>
      <div class="slider-row">
        <label>QUICK_SEC <span id="v-quick-sec">0.80</span>s</label>
        <input type="range" id="quick-sec" min="0.05" max="2.0" step="0.05" value="0.80">
      </div>
    </div>

    <div class="card">
      <h2>이벤트 로그</h2>
      <div class="log" id="log"></div>
    </div>

  </div>
</div>

<script>
const MAX_POINTS = 200;
const labels = [];
const pressureData = [];
let gripArm   = {{ grip_arm }};
let pullTrig  = {{ pull_trig }};
let gripReset = {{ grip_reset }};

Chart.register(window['chartjs-plugin-annotation']);

const ctx = document.getElementById('chart').getContext('2d');
const chart = new Chart(ctx, {
  type: 'line',
  data: {
    labels: labels,
    datasets: [{
      label: '압력',
      data: pressureData,
      borderColor: '#7ef',
      borderWidth: 1.5,
      pointRadius: 0,
      tension: 0.2,
    }]
  },
  options: {
    animation: false,
    responsive: true,
    scales: {
      x: { display: false },
      y: { min: 0, max: 4095, grid: { color: '#333' }, ticks: { color: '#888' } }
    },
    plugins: {
      legend: { display: false },
      annotation: {
        annotations: {
          lineGripArm: {
            type: 'line', yMin: gripArm, yMax: gripArm,
            borderColor: '#fa0', borderWidth: 1.5, borderDash: [6,3],
            label: { display: true, content: 'GRIP_ARM', color: '#fa0', position: 'start', font: { size: 11 } }
          },
          linePullTrig: {
            type: 'line', yMin: pullTrig, yMax: pullTrig,
            borderColor: '#f44', borderWidth: 1.5, borderDash: [6,3],
            label: { display: true, content: 'PULL_TRIG', color: '#f44', position: 'start', font: { size: 11 } }
          },
          lineGripReset: {
            type: 'line', yMin: gripReset, yMax: gripReset,
            borderColor: '#555', borderWidth: 1, borderDash: [4,4],
            label: { display: true, content: 'RESET', color: '#666', position: 'start', font: { size: 10 } }
          },
        }
      }
    }
  }
});

function updateAnnotations() {
  chart.options.plugins.annotation.annotations.lineGripArm.yMin = gripArm;
  chart.options.plugins.annotation.annotations.lineGripArm.yMax = gripArm;
  chart.options.plugins.annotation.annotations.linePullTrig.yMin = pullTrig;
  chart.options.plugins.annotation.annotations.linePullTrig.yMax = pullTrig;
  chart.options.plugins.annotation.annotations.lineGripReset.yMin = gripReset;
  chart.options.plugins.annotation.annotations.lineGripReset.yMax = gripReset;
  chart.update('none');
}

// 슬라이더
const sliders = {
  'grip-arm':   { display: 'v-grip-arm',   key: 'GRIP_ARM' },
  'pull-trig':  { display: 'v-pull-trig',  key: 'PULL_TRIG' },
  'grip-reset': { display: 'v-grip-reset', key: 'GRIP_RESET' },
  'quick-sec':  { display: 'v-quick-sec',  key: 'QUICK_SEC' },
};

function sendThresholds() {
  socket.emit('update_thresholds', {
    GRIP_ARM:   parseFloat(document.getElementById('grip-arm').value),
    PULL_TRIG:  parseFloat(document.getElementById('pull-trig').value),
    GRIP_RESET: parseFloat(document.getElementById('grip-reset').value),
    QUICK_SEC:  parseFloat(document.getElementById('quick-sec').value),
  });
}

Object.entries(sliders).forEach(([id, cfg]) => {
  const el = document.getElementById(id);
  el.addEventListener('input', () => {
    const v = parseFloat(el.value);
    const isFloat = id === 'quick-sec';
    document.getElementById(cfg.display).textContent = isFloat ? v.toFixed(2) : v;
    if (id === 'grip-arm')   { gripArm   = v; updateAnnotations(); }
    if (id === 'pull-trig')  { pullTrig  = v; updateAnnotations(); }
    if (id === 'grip-reset') { gripReset = v; updateAnnotations(); }
    sendThresholds();
  });
});

// 이벤트 로그
function addLog(msg, cls) {
  const log = document.getElementById('log');
  const div = document.createElement('div');
  div.className = 'log-item ' + cls;
  div.textContent = new Date().toLocaleTimeString('ko') + '  ' + msg;
  log.prepend(div);
  if (log.children.length > 50) log.lastChild.remove();
}

// 소켓
const socket = io();
socket.on('data', d => {
  // 그래프
  pressureData.push(d.pressure);
  labels.push('');
  if (pressureData.length > MAX_POINTS) { pressureData.shift(); labels.shift(); }
  chart.update('none');

  // 상태
  document.getElementById('s-pressure').textContent = d.pressure;
  document.getElementById('s-pull').textContent = d.pull_count;
  document.getElementById('s-btn1').textContent = d.btn1;
  document.getElementById('s-btn2').textContent = d.btn2;
  const armedEl = document.getElementById('s-armed');
  if (d.armed) { armedEl.textContent = 'YES'; armedEl.className = 'armed-y'; }
  else          { armedEl.textContent = 'NO';  armedEl.className = 'armed-n'; }

  // 이벤트
  if (d.event === 'PULL') addLog(`당김! #${d.pull_count}  (${d.event_dt}ms)`, 'pull');
  if (d.event === 'SLOW') addLog(`천천히 눌림 (당김 아님)  (${d.event_dt}ms)`, 'slow');
  if (d.btn1 === 1) addLog('버튼1 눌림', 'btn');
  if (d.btn2 === 1) addLog('버튼2 눌림', 'btn');
});
</script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML,
        grip_arm=thresholds['GRIP_ARM'],
        pull_trig=thresholds['PULL_TRIG'],
        grip_reset=thresholds['GRIP_RESET'],
    )

if __name__ == '__main__':
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    print("[서버 시작] http://localhost:5001 열기")
    socketio.run(app, host='0.0.0.0', port=5001, debug=False, allow_unsafe_werkzeug=True)

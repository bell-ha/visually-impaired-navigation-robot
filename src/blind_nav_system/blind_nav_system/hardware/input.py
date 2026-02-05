import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collections
import time

# --- 설정 ---
PORT = '/dev/ttyUSB0'
BAUD = 115200
pressure_data = collections.deque(maxlen=100)

# =========================
# PULL 규칙(네가 말한 조건)
# =========================
GRIP_ARM = 3000          # 이 값 이상이면 "잡았다/눌렀다"로 보고 타이머 시작
PULL_TRIG = 3700         # 이 값 이상이면 "당김 후보"
QUICK_SEC = 0.25         # GRIP_ARM 넘긴 뒤 이 시간 안에 PULL_TRIG 넘으면 '당김'
GRIP_RESET = 2900        # 다시 이 값 아래로 내려가야 재무장(히스테리시스)

# 상태
armed = False            # GRIP_ARM을 넘겨 "감시(타이머 시작)" 상태
armed_time = 0.0         # GRIP_ARM 처음 넘긴 시간
armed_start_val = 0      # GRIP_ARM 처음 넘긴 값
triggered_this_arm = False

pull_count = 0

# 디버그 표시용
last_val = 0
last_dt = 0.0

def pull_bang(val: int):
    """
    return:
      None
      ("PULL", count, dt)
      ("SLOW", dt)  # (원하면 로그 찍을 수도 있는) 천천히 올라간 케이스
    """
    global armed, armed_time, armed_start_val, triggered_this_arm, pull_count, last_dt

    now = time.monotonic()

    # 1) 재무장 조건: 충분히 내려오면 상태 리셋
    if val < GRIP_RESET:
        armed = False
        triggered_this_arm = False
        last_dt = 0.0
        return None

    # 2) 아직 armed가 아니고, GRIP_ARM을 넘기면 "감시 시작"
    if not armed and val >= GRIP_ARM:
        armed = True
        armed_time = now
        armed_start_val = val
        triggered_this_arm = False
        last_dt = 0.0
        return None

    # 3) armed 상태면 시간 체크
    if armed:
        dt = now - armed_time
        last_dt = dt

        # 이미 이번 grip에서 트리거를 한 번 냈으면 더는 안 냄 (놓을 때까지 대기)
        if triggered_this_arm:
            return None

        # 3-A) 빠르게 3700을 넘으면 => PULL Bang 1회
        if val >= PULL_TRIG and dt <= QUICK_SEC:
            triggered_this_arm = True
            pull_count += 1
            return ("PULL", pull_count, dt)

        # 3-B) 3700은 넘었지만, 시간이 오래 걸렸다 => 천천히 눌러서 올라간 케이스 (pull 아님)
        #      (원치 않으면 아래 반환을 None으로 바꿔도 됨)
        if val >= PULL_TRIG and dt > QUICK_SEC:
            # 여기서는 "당김"으로 처리하지 않음. 재트리거 방지하려면
            #  - 계속 triggered_this_arm=False로 둬도 되고(손을 조금 놨다 다시 빨리 당기면 잡힘),
            #  - 혹은 천천히라도 3700 넘으면 이번 arm은 종료로 보고 막을 수도 있음.
            # 네가 헷갈리기 싫다고 했으니, 천천히 3700 넘은 순간부터는 이번 arm에서 pull 금지로 잠금:
            triggered_this_arm = True
            return ("SLOW", dt)

    return None


# =========================
# Serial 연결
# =========================
try:
    ser = serial.Serial(PORT, BAUD, timeout=0)
except Exception as e:
    print(f"❌ 연결 실패: {e}")
    exit()

# =========================
# Plot 설정
# =========================
fig, ax = plt.subplots()
line_plot, = ax.plot([], [], lw=2, color='green')
ax.set_ylim(0, 4095)
ax.set_xlim(0, 100)

status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va='top', fontsize=10)
data_buffer = ""

def on_key(event):
    """실행 중 튜닝 키"""
    global GRIP_ARM, PULL_TRIG, QUICK_SEC

    if event.key == 'a':        # GRIP_ARM +
        GRIP_ARM += 20
    elif event.key == 'z':      # GRIP_ARM -
        GRIP_ARM = max(0, GRIP_ARM - 20)

    elif event.key == 's':      # PULL_TRIG +
        PULL_TRIG += 20
    elif event.key == 'x':      # PULL_TRIG -
        PULL_TRIG = max(0, PULL_TRIG - 20)

    elif event.key == 'w':      # QUICK_SEC +
        QUICK_SEC = min(2.0, QUICK_SEC + 0.05)
    elif event.key == 'e':      # QUICK_SEC -
        QUICK_SEC = max(0.05, QUICK_SEC - 0.05)

    elif event.key == 'p':
        print(f"\n[PARAM] GRIP_ARM={GRIP_ARM}, PULL_TRIG={PULL_TRIG}, QUICK_SEC={QUICK_SEC:.2f}, RESET={GRIP_RESET}", flush=True)

    elif event.key in ('escape',):
        plt.close(fig)

fig.canvas.mpl_connect('key_press_event', on_key)

def update(frame):
    global data_buffer, last_val

    if ser.in_waiting > 0:
        data_buffer += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')

        if '\n' in data_buffer:
            lines = data_buffer.split('\n')
            data_buffer = lines.pop()

            for line in lines:
                line = line.strip()
                if not line:
                    continue

                parts = line.split(',')
                if len(parts) < 3:
                    continue

                tag = parts[0].strip()
                try:
                    press_val = int(parts[2].strip())
                except:
                    continue

                last_val = press_val

                if tag == "TRIG":
                    print("\n⚡ 버튼(TRIG)!", flush=True)

                evt = pull_bang(press_val)
                if evt:
                    if evt[0] == "PULL":
                        _, n, dt = evt
                        print(f"\n🧲 PULL #{n}  (dt={dt*1000:.0f}ms)", flush=True)
                    elif evt[0] == "SLOW":
                        # 천천히 눌러서 3700 넘은 케이스를 굳이 로그로 보고 싶지 않으면 아래 줄을 지워도 됨.
                        _, dt = evt
                        print(f"\n🟡 SLOW-PRESS (no pull)  (dt={dt*1000:.0f}ms)", flush=True)

                pressure_data.append(press_val)

    line_plot.set_data(range(len(pressure_data)), list(pressure_data))

    status_text.set_text(
        "keys: a/z GRIP_ARM, s/x PULL_TRIG, w/e QUICK_SEC, p print, ESC quit\n"
        f"GRIP_ARM={GRIP_ARM}  PULL_TRIG={PULL_TRIG}  QUICK_SEC={QUICK_SEC:.2f}s  RESET<{GRIP_RESET}\n"
        f"val={last_val}  armed={armed}  dt={last_dt:.2f}s  pull_count={pull_count}"
    )

    return line_plot, status_text

ani = FuncAnimation(fig, update, interval=5, blit=True, cache_frame_data=False)
plt.show()

try:
    ser.close()
except:
    pass

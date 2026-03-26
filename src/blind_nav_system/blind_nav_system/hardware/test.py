import serial
import sys
import time

PORT = '/dev/tty.usbserial-A5069RR4'
BAUD = 115200

# =====================
# 여기서 임계값 조절
# =====================
GRIP_ARM   = 3000   # 이 값 이상이면 감시 시작 (잡기 시작)
PULL_TRIG  = 3700   # 이 값 이상이면 당김 후보
QUICK_SEC  = 0.25   # GRIP_ARM 넘긴 후 이 시간(초) 안에 PULL_TRIG 넘으면 당김
GRIP_RESET = 2900   # 이 값 아래로 내려가야 재감지

# 상태
armed = False
armed_time = 0.0
triggered_this_arm = False
pull_count = 0

def pull_bang(val):
    global armed, armed_time, triggered_this_arm, pull_count
    now = time.monotonic()

    if val < GRIP_RESET:
        armed = False
        triggered_this_arm = False
        return None

    if not armed and val >= GRIP_ARM:
        armed = True
        armed_time = now
        triggered_this_arm = False
        return None

    if armed:
        dt = now - armed_time
        if triggered_this_arm:
            return None
        if val >= PULL_TRIG and dt <= QUICK_SEC:
            triggered_this_arm = True
            pull_count += 1
            return ("PULL", pull_count, dt)
        if val >= PULL_TRIG and dt > QUICK_SEC:
            triggered_this_arm = True
            return ("SLOW", dt)

    return None

print(f"[연결 시도] {PORT} @ {BAUD}")
print(f"[임계값] GRIP_ARM={GRIP_ARM}  PULL_TRIG={PULL_TRIG}  QUICK_SEC={QUICK_SEC}s  RESET<{GRIP_RESET}\n")

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"[OK] 연결 성공! 신호 기다리는 중... (종료: Ctrl+C)\n")
except Exception as e:
    print(f"[FAIL] 연결 실패: {e}")
    sys.exit(1)

try:
    buf = ""
    while True:
        buf += ser.read(ser.in_waiting or 1).decode('utf-8', errors='ignore')
        if '\n' in buf:
            lines = buf.split('\n')
            buf = lines.pop()
            for line in lines:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                tag = parts[0] if parts else ''

                if tag == 'TRIG':
                    btn = parts[1] if len(parts) > 1 else '?'
                    raw_val = parts[2] if len(parts) > 2 else '?'
                    print(f"[TRIG] 버튼{btn} 눌림  |  압력값: {raw_val}")
                    try:
                        evt = pull_bang(int(raw_val))
                        if evt:
                            if evt[0] == "PULL":
                                print(f"  >>> 당김! #{evt[1]}  (dt={evt[2]*1000:.0f}ms)")
                            elif evt[0] == "SLOW":
                                print(f"  >>> 천천히 눌림 (당김 아님)  (dt={evt[1]*1000:.0f}ms)")
                    except:
                        pass

                elif tag == 'DATA':
                    b1  = parts[1] if len(parts) > 1 else '?'
                    b2  = parts[2] if len(parts) > 2 else '?'
                    raw_val = parts[3] if len(parts) > 3 else '?'
                    try:
                        press = int(raw_val)
                        evt = pull_bang(press)
                        status = f"armed={'Y' if armed else 'N'}"
                        if evt:
                            if evt[0] == "PULL":
                                status = f">>> 당김! #{evt[1]}  (dt={evt[2]*1000:.0f}ms)"
                            elif evt[0] == "SLOW":
                                status = f">>> 천천히 눌림 (당김 아님)"
                        print(f"[DATA] btn1={b1}  btn2={b2}  pressure={press}  {status}")
                    except:
                        print(f"[DATA] btn1={b1}  btn2={b2}  pressure={raw_val}")
                else:
                    print(f"[RAW]  {line}")
except KeyboardInterrupt:
    print("\n[종료]")
finally:
    ser.close()

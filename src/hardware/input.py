import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collections

# --- 설정 ---
PORT = '/dev/ttyUSB0' 
BAUD = 115200
pressure_data = collections.deque(maxlen=100) 

try:
    # 근거: timeout을 0으로 설정하여 기다리지 않고 즉시 현재 버퍼를 읽게 합니다.
    ser = serial.Serial(PORT, BAUD, timeout=0) 
except Exception as e:
    print(f"❌ 연결 실패: {e}")
    exit()

fig, ax = plt.subplots()
line_plot, = ax.plot([], [], lw=2, color='green') # 색상을 초록으로 변경 (안정성)
ax.set_ylim(0, 4095)
ax.set_xlim(0, 100)

# 버퍼를 누적할 변수
data_buffer = ""

def update(frame):
    global data_buffer
    
    # 근거: 버퍼에 쌓인 모든 데이터를 한꺼번에 가져와서 지연을 없앱니다.
    if ser.in_waiting > 0:
        # 현재 들어온 모든 바이트를 읽어 문자열로 변환
        data_buffer += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        
        # 줄바꿈 문자가 포함되어 있다면 (한 줄이 완성되었다면)
        if '\n' in data_buffer:
            lines = data_buffer.split('\n')
            # 마지막 미완성 줄은 다시 버퍼에 저장
            data_buffer = lines.pop()
            
            # 완성된 줄들 중 가장 최신 데이터 위주로 처리
            for line in lines:
                line = line.strip()
                if not line: continue
                
                parts = line.split(',')
                if len(parts) < 3: continue

                tag = parts[0]
                press_val = int(parts[2].strip())

                if tag == "TRIG":
                    # 근거: flush=True를 써야 터미널 버퍼링 없이 즉시 보입니다.
                    print("\n⚡ [FAST] 버튼 트리거!", flush=True)

                pressure_data.append(press_val)
                line_plot.set_data(range(len(pressure_data)), list(pressure_data))

    return line_plot,

# interval을 1ms로 설정하여 CPU가 허용하는 가장 빠른 속도로 체크합니다.
ani = FuncAnimation(fig, update, interval=1, blit=True, cache_frame_data=False)
plt.show()
import serial
import threading
import time

# --- 설정 ---
PORT = '/dev/ttyUSB0'
BAUD = 115200

# =========================
# PULL 규칙(너가 쓰던 값 그대로)
# =========================
GRIP_ARM = 3000
PULL_TRIG = 3700
QUICK_SEC = 0.25
GRIP_RESET = 2900

# 디바운스(같은 이벤트 연속 방지)
DEBOUNCE_SEC = 0.25


class PullDetector:
    def __init__(self):
        self.armed = False
        self.armed_time = 0.0
        self.triggered_this_arm = False

    def update(self, val: int):
        """
        return:
          None
          "PULL"
        """
        now = time.monotonic()

        if val < GRIP_RESET:
            self.armed = False
            self.triggered_this_arm = False
            return None

        if (not self.armed) and val >= GRIP_ARM:
            self.armed = True
            self.armed_time = now
            self.triggered_this_arm = False
            return None

        if self.armed:
            dt = now - self.armed_time
            if self.triggered_this_arm:
                return None

            if val >= PULL_TRIG and dt <= QUICK_SEC:
                self.triggered_this_arm = True
                return "PULL"

            # 천천히 올라간 건 pull로 안 침(원 코드와 동일한 의도)
            if val >= PULL_TRIG and dt > QUICK_SEC:
                self.triggered_this_arm = True
                return None

        return None


class HardwareKeyBridge:
    """
    시리얼 입력을 읽어서:
      - TRIG  -> on_o() 호출  (MainManager의 'o'와 매칭)
      - PULL  -> on_p() 호출  (MainManager의 'p'와 매칭)
    """
    def __init__(self, on_o, on_p, port=PORT, baud=BAUD):
        self.on_o = on_o
        self.on_p = on_p
        self.port = port
        self.baud = baud

        self._stop = threading.Event()
        self._thr = None
        self._ser = None
        self._buf = ""
        self._pull = PullDetector()

        self._last_o_t = 0.0
        self._last_p_t = 0.0

    def start(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=0)
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()

    def stop(self):
        self._stop.set()
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass

    def _debounced_call(self, kind: str):
        now = time.monotonic()
        if kind == "o":
            if now - self._last_o_t >= DEBOUNCE_SEC:
                self._last_o_t = now
                self.on_o()
        elif kind == "p":
            if now - self._last_p_t >= DEBOUNCE_SEC:
                self._last_p_t = now
                self.on_p()

    def _loop(self):
        while not self._stop.is_set():
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

                        # 1) 버튼(TRIG) -> o
                        if tag == "TRIG":
                            self._debounced_call("o")

                        # 2) 당김(PULL) -> p
                        evt = self._pull.update(press_val)
                        if evt == "PULL":
                            self._debounced_call("p")

            time.sleep(0.005)

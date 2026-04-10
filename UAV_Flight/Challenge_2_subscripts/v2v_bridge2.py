import serial
import struct
import threading
import time

# ==========================================
# V2V BRIDGE LIBRARY - CHALLENGE 2 VERSION
# ==========================================

# Packet headers
SOF = 0xAA
TYPE_TELEM = 1
TYPE_CMD = 2
TYPE_MSG = 3

# Existing command codes
CMD_ARM          = 1
CMD_DISARM       = 2
CMD_TAKEOFF      = 3
CMD_LAND         = 4
CMD_MOVE_FORWARD = 5
CMD_MOVE_2FT     = 6
CMD_TURN_RIGHT   = 7
CMD_TURN_LEFT    = 8
CMD_CIRCLE       = 9
CMD_MISSION_1    = 10
CMD_MISSION_2    = 11
CMD_MISSION_3    = 12
CMD_STOP         = 13

# New Challenge 2 specific command codes
CMD_CHALLENGE2_START    = 14
CMD_CHALLENGE2_COORDS   = 15
CMD_CHALLENGE2_COMPLETE = 16
CMD_CHALLENGE2_ABORT    = 17

# Mode identifiers
MODE_INITIAL  = 0
MODE_GUIDED   = 1
MODE_AUTO     = 2
MODE_LAND     = 3
MODE_DISARMED = 4

# Binary formats
TELEM_FMT = "<IIffBB"
CMD_FMT   = "<IBB"


class V2VBridge:
    def __init__(self, port, baud=115200, name="Bridge"):
        self.port = port
        self.baud = baud
        self.name = name
        self.ser = None

        self.latest_telemetry = None
        self.latest_command = None
        self.latest_msg = None
        self._running = False
        self._lock = threading.Lock()
        self._thread = None

    def connect(self):
        print(f"[{self.name}] Connecting to {self.port} at {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print(f"[{self.name}] Bridge Thread Running... Listening for the radio.")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser:
            self.ser.close()

    def _chk_xor(self, type_b, len_b, payload: bytes) -> int:
        c = (type_b ^ len_b) & 0xFF
        for b in payload:
            c ^= b
        return c & 0xFF

    def _read_loop(self):
        while self._running:
            if self.ser.in_waiting == 0:
                time.sleep(0.001)
                continue

            b = self.ser.read(1)
            if not b or b[0] != SOF:
                continue

            hdr = self.ser.read(2)
            if len(hdr) < 2:
                continue

            f_type, f_len = hdr[0], hdr[1]

            payload = self.ser.read(f_len)
            if len(payload) < f_len:
                continue

            chk_byte = self.ser.read(1)
            if not chk_byte:
                continue

            if chk_byte[0] == self._chk_xor(f_type, f_len, payload):
                with self._lock:
                    if f_type == TYPE_TELEM and f_len == struct.calcsize(TELEM_FMT):
                        self.latest_telemetry = struct.unpack(TELEM_FMT, payload)
                    elif f_type == TYPE_CMD and f_len == struct.calcsize(CMD_FMT):
                        self.latest_command = struct.unpack(CMD_FMT, payload)
                    elif f_type == TYPE_MSG:
                        self.latest_msg = payload.decode("ascii", errors="ignore")

    def get_telemetry(self):
        with self._lock:
            return self.latest_telemetry

    def get_command(self, consume=True):
        with self._lock:
            val = self.latest_command
            if consume:
                self.latest_command = None
            return val

    def get_message(self, consume=True):
        with self._lock:
            val = self.latest_msg
            if consume:
                self.latest_msg = None
            return val

    def send_telemetry(self, seq, t_ms, vx, vy, marker, estop):
        payload = struct.pack(TELEM_FMT, seq, t_ms, vx, vy, marker, estop)
        chk = self._chk_xor(TYPE_TELEM, len(payload), payload)
        self.ser.write(bytes([SOF, TYPE_TELEM, len(payload)]) + payload + bytes([chk]))
        self.ser.flush()

    def send_command(self, cmdSeq, cmd, estop=0):
        payload = struct.pack(CMD_FMT, cmdSeq, cmd, estop)
        chk = self._chk_xor(TYPE_CMD, len(payload), payload)
        self.ser.write(bytes([SOF, TYPE_CMD, len(payload)]) + payload + bytes([chk]))
        self.ser.flush()

    def send_message(self, text: str):
        payload = text[:60].encode("ascii", errors="ignore")
        chk = self._chk_xor(TYPE_MSG, len(payload), payload)
        self.ser.write(bytes([SOF, TYPE_MSG, len(payload)]) + payload + bytes([chk]))
        self.ser.flush()

    # ==========================================
    # Challenge 2 helper API
    # ==========================================

    def send_challenge2_start(self, seq=0):
        self.send_command(seq, CMD_CHALLENGE2_START, 0)

    def send_challenge2_abort(self, seq=0):
        self.send_command(seq, CMD_CHALLENGE2_ABORT, 1)

    def send_challenge2_coords(self, x_m, y_m):
        # Ground station expects GOTO:x,y
        self.send_message(f"GOTO:{x_m:.3f},{y_m:.3f}")

    def send_challenge2_coords_ft(self, x_ft, y_ft):
        ft_to_m = 0.3048
        x_m = x_ft * ft_to_m
        y_m = y_ft * ft_to_m
        self.send_challenge2_coords(x_m, y_m)

    def is_challenge2_complete_message(self, msg: str) -> bool:
        if not msg:
            return False
        return msg.startswith("challenge2_complete:")

    def parse_challenge2_complete_message(self, msg: str):
        # Example: challenge2_complete:2.44,2.44
        if not self.is_challenge2_complete_message(msg):
            return None

        try:
            payload = msg.split(":", 1)[1]
            x_str, y_str = payload.split(",")
            return float(x_str.strip()), float(y_str.strip())
        except Exception:
            return None
"""
jetson_uav_sender.py  —  UAV side (Jetson Nano)
------------------------------------------------
Reads LOCAL_POSITION_NED from the UAV flight controller via MAVLink
and streams the position as a newline-terminated JSON packet over
UART to the ESP32 (server.cpp), which forwards it via ESP-NOW to
the UGV.

No GPS required — LOCAL_POSITION_NED is supplied by the flight
controller's EKF using IMU + barometer + optical flow / encoders.

Wiring:
    Jetson UART TX (e.g. /dev/ttyTHS1 pin 8) --> ESP32 RX
    Jetson UART RX                            --> ESP32 TX
    (or use USB cable: /dev/ttyACM0 to flight controller,
     and a second serial port to the ESP32)

JSON packet format (sent at SEND_HZ):
    {"x":<float>,"y":<float>,"z":<float>,"vx":<float>,"vy":<float>,"t":<int>}
    x,y  = North/East metres in LOCAL NED frame (origin = EKF init point)
    z    = Down metres (negative = above origin)
    vx,vy= North/East velocity m/s
    t    = boot timestamp ms (for staleness checks on UGV side)
"""

import serial
import time
import json
import sys
from pymavlink import mavutil

# ─────────────────────────────────────────
# CONFIG  — adjust to your hardware
# ─────────────────────────────────────────
FC_PORT      = "/dev/ttyACM0"   # Flight controller USB or "/dev/ttyTHS1" GPIO
FC_BAUD      = 115200

ESP_PORT     = "/dev/ttyTHS1"   # UART to ESP32 server.cpp
                                 # Change to a second ACM port if using USB-serial adapter
ESP_BAUD     = 115200

SEND_HZ      = 2                 # How often to transmit position (max useful ≈ 2 Hz given ESP-NOW 500 ms pacing in server.cpp)
STREAM_HZ    = 20                # Rate to request LOCAL_POSITION_NED from FC

# ─────────────────────────────────────────
# CONNECT TO FLIGHT CONTROLLER
# ─────────────────────────────────────────
print(f"[UAV] Connecting to flight controller on {FC_PORT} @ {FC_BAUD} baud...")
master = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
master.wait_heartbeat()
print(f"[UAV] Heartbeat OK  sys={master.target_system}  comp={master.target_component}")

# Request LOCAL_POSITION_NED at STREAM_HZ
def set_msg_interval(msg_id, hz):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        int(1e6 / hz),
        0, 0, 0, 0, 0
    )

set_msg_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, STREAM_HZ)

# Also enable the legacy stream as a fallback
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    STREAM_HZ, 1
)
print(f"[UAV] Requested LOCAL_POSITION_NED at {STREAM_HZ} Hz")

# ─────────────────────────────────────────
# OPEN ESP32 UART
# ─────────────────────────────────────────
print(f"[UAV] Opening ESP32 serial on {ESP_PORT} @ {ESP_BAUD} baud...")
try:
    esp_serial = serial.Serial(ESP_PORT, ESP_BAUD, timeout=0.1)
    print("[UAV] ESP32 serial open.")
except serial.SerialException as e:
    print(f"[ERROR] Cannot open ESP32 serial port: {e}")
    sys.exit(1)

# ─────────────────────────────────────────
# HELPER: get latest LOCAL_POSITION_NED
# ─────────────────────────────────────────
def get_local_position(timeout=0.1):
    """Drain the MAVLink buffer and return the freshest LOCAL_POSITION_NED or None."""
    latest = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            latest = msg
        else:
            time.sleep(0.005)
    return latest

# ─────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────
print(f"[UAV] Streaming position at {SEND_HZ} Hz. Press Ctrl-C to stop.\n")
interval = 1.0 / SEND_HZ
last_send = 0.0
sent_count = 0

try:
    while True:
        now = time.time()
        if now - last_send < interval:
            time.sleep(0.01)
            continue

        lp = get_local_position(timeout=0.08)
        if lp is None:
            print("[WARN] No LOCAL_POSITION_NED — waiting for EKF...")
            time.sleep(0.5)
            continue

        packet = {
            "x":  round(float(lp.x),  3),   # North  (m)
            "y":  round(float(lp.y),  3),   # East   (m)
            "z":  round(float(lp.z),  3),   # Down   (m, negative = above origin)
            "vx": round(float(lp.vx), 3),   # North velocity (m/s)
            "vy": round(float(lp.vy), 3),   # East  velocity (m/s)
            "t":  lp.time_boot_ms            # boot timestamp (ms)
        }

        line = json.dumps(packet, separators=(',', ':')) + '\n'
        esp_serial.write(line.encode('ascii'))
        esp_serial.flush()

        sent_count += 1
        last_send = now
        print(f"[TX #{sent_count}] {line.strip()}")

except KeyboardInterrupt:
    print("\n[UAV] Stopped by user.")
finally:
    esp_serial.close()
    print("[UAV] Serial closed.")

"""
rpi_ugv_navigator.py  —  UGV side (Raspberry Pi 5)
----------------------------------------------------
Reads UAV position JSON packets forwarded by the UGV ESP32 (client.cpp)
over UART, then commands the UGV (ArduRover in GUIDED mode) to drive
to that location using its own LOCAL_POSITION_NED — no GPS required.

Assumptions:
  • Both UAV and UGV EKFs are initialised at (or very near) the same
    physical origin so that LOCAL_POSITION_NED frames are aligned.
    In practice: power them up side-by-side before the UAV takes off.
  • ArduRover is configured for GPS-denied GUIDED using wheel-encoder
    odometry (AHRS_EKF_TYPE=3, EK3_SRC1_POSXY=5 "wheel encoders" or
    EK3_SRC1_VELXY=5, WHEEL_ENC_ENABLE=1, etc.).

Wiring:
    RPi5 UART RX (GPIO 15, pin 10) <-- ESP32 TX  (pin 4 in client.cpp)
    RPi5 UART TX (GPIO 14, pin 8)  --> ESP32 RX  (pin 5 in client.cpp)
    Use /dev/ttyAMA0 or /dev/ttyS0 — enable via raspi-config.

    Flight controller connected to RPi5 via USB:
    /dev/ttyACM0  (or /dev/ttyACM1 if ESP32 also USB)
"""

import serial
import threading
import time
import json
import math
import sys
from pymavlink import mavutil

# ─────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────
ESP_PORT        = "/dev/ttyAMA0"   # UART from ESP32 client.cpp Serial2
ESP_BAUD        = 115200

FC_PORT         = "/dev/ttyACM0"   # Flight controller USB
FC_BAUD         = 115200

# Navigation tuning
SPEED_MPS       = 0.5      # UGV travel speed towards UAV
ARRIVE_DIST_M   = 0.4      # Stop when within this radius of target (m)
CMD_HZ          = 10       # Velocity command resend rate
STALE_TIMEOUT_S = 3.0      # Stop moving if no fresh packet received for this long
MOVE_TIMEOUT_S  = 120      # Hard safety timeout per navigation leg

# Whether to continuously track the UAV (True) or just go once and stop (False)
CONTINUOUS_TRACK = True

# ─────────────────────────────────────────
# SHARED STATE  (written by serial thread,
#                read by navigation thread)
# ─────────────────────────────────────────
target_lock  = threading.Lock()
target_x     = None   # UAV North position (m)
target_y     = None   # UAV East  position (m)
target_stamp = 0.0    # wall-clock time of last valid packet

# ─────────────────────────────────────────
# SERIAL READER THREAD
# ─────────────────────────────────────────
def serial_reader(port, baud):
    """
    Continuously reads newline-terminated JSON from the ESP32 and
    updates the shared target position.
    """
    global target_x, target_y, target_stamp
    print(f"[SERIAL] Opening {port} @ {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open ESP serial: {e}")
        sys.exit(1)

    print("[SERIAL] ESP32 UART open — waiting for packets...")
    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            if not line:
                continue

            data = json.loads(line)
            x = float(data["x"])
            y = float(data["y"])

            with target_lock:
                target_x     = x
                target_y     = y
                target_stamp = time.time()

            print(f"[RX] UAV position  x={x:.3f} m N  y={y:.3f} m E")

        except json.JSONDecodeError:
            print(f"[WARN] Bad JSON: {line!r}")
        except KeyError as e:
            print(f"[WARN] Missing field {e} in: {line!r}")
        except Exception as e:
            print(f"[WARN] Serial error: {e}")
            time.sleep(0.5)

# ─────────────────────────────────────────
# MAVLink HELPERS
# ─────────────────────────────────────────
def connect_fc(port, baud):
    print(f"[FC] Connecting to flight controller on {port} @ {baud} baud...")
    m = mavutil.mavlink_connection(port, baud=baud)
    m.wait_heartbeat()
    print(f"[FC] Heartbeat OK  sys={m.target_system}  comp={m.target_component}")
    return m

def set_msg_interval(master, msg_id, hz):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0
    )

def change_mode(master, mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise RuntimeError(f"Unknown mode '{mode}'. Available: {list(mapping.keys())}")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode]
    )
    print(f"[FC] Mode → {mode}")
    time.sleep(1.0)

def arm(master):
    print("[FC] Arming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < 10:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[FC] Armed.")
            return
    print("[WARN] Arm confirmation not received — proceeding cautiously.")

def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("[FC] Disarmed.")
    time.sleep(1.0)

def get_local_xy(master, timeout=0.3):
    """Return (x_north, y_east) from LOCAL_POSITION_NED or None."""
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg is None:
        return None
    return float(msg.x), float(msg.y)

def send_velocity_ned(master, vx, vy, vz=0.0):
    """
    Send a LOCAL NED velocity command to ArduRover.
    Type-mask: ignore position + acceleration + yaw, control velocity only.
    """
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE  |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE  |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE  |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0,        # position (ignored)
        vx, vy, vz,     # velocity (m/s NED)
        0, 0, 0,        # acceleration (ignored)
        0, 0            # yaw, yaw_rate (ignored)
    )

def stop_ugv(master, duration=1.0):
    """Send repeated zero-velocity commands to halt the UGV."""
    end = time.time() + duration
    while time.time() < end:
        send_velocity_ned(master, 0.0, 0.0, 0.0)
        time.sleep(0.1)

# ─────────────────────────────────────────
# NAVIGATION LOOP
# ─────────────────────────────────────────
def navigate_to_uav(master):
    """
    Continuously drives the UGV toward the latest received UAV position.
    Stops when within ARRIVE_DIST_M.  If CONTINUOUS_TRACK=True it will
    resume tracking if the UAV moves.
    """
    cmd_interval = 1.0 / CMD_HZ
    t_mission_start = time.time()
    arrived = False

    print("[NAV] Navigation loop started.")

    while True:
        loop_start = time.time()

        # ── Check for stale / missing data ──────────────────────────
        with target_lock:
            tx = target_x
            ty = target_y
            age = time.time() - target_stamp if target_stamp > 0 else 999.0

        if tx is None:
            print("[NAV] Waiting for first UAV position packet...")
            time.sleep(0.5)
            continue

        if age > STALE_TIMEOUT_S:
            print(f"[NAV] STALE data ({age:.1f}s old) — stopping UGV.")
            stop_ugv(master, duration=0.5)
            arrived = False
            time.sleep(0.2)
            continue

        # ── Get UGV current position ─────────────────────────────────
        pos = get_local_xy(master, timeout=0.15)
        if pos is None:
            print("[WARN] No LOCAL_POSITION_NED from UGV FC — waiting...")
            time.sleep(0.2)
            continue
        ux, uy = pos

        # ── Compute vector to target ─────────────────────────────────
        ex = tx - ux
        ey = ty - uy
        dist = math.hypot(ex, ey)

        print(f"[NAV] UGV=({ux:.2f},{uy:.2f})  UAV=({tx:.2f},{ty:.2f})  dist={dist:.2f}m  data_age={age:.1f}s")

        # ── Arrival check ────────────────────────────────────────────
        if dist <= ARRIVE_DIST_M:
            if not arrived:
                print(f"[NAV] *** Arrived at UAV position (within {ARRIVE_DIST_M} m) ***")
                stop_ugv(master, duration=1.0)
                arrived = True
            if not CONTINUOUS_TRACK:
                print("[NAV] Task complete — single-shot mode.")
                break
            # In continuous mode: hover at position and wait for UAV to move
            send_velocity_ned(master, 0.0, 0.0, 0.0)
        else:
            arrived = False
            # Proportional velocity toward target, clamped to SPEED_MPS
            vx = (ex / dist) * SPEED_MPS
            vy = (ey / dist) * SPEED_MPS
            send_velocity_ned(master, vx, vy, 0.0)

        # ── Safety hard timeout ──────────────────────────────────────
        if (time.time() - t_mission_start) > MOVE_TIMEOUT_S:
            print("[NAV] HARD TIMEOUT reached — stopping.")
            stop_ugv(master, duration=2.0)
            break

        # ── Pace to CMD_HZ ───────────────────────────────────────────
        elapsed = time.time() - loop_start
        sleep_t = cmd_interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

# ─────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────
if __name__ == "__main__":
    # 1. Start serial reader in background thread
    reader_thread = threading.Thread(
        target=serial_reader, args=(ESP_PORT, ESP_BAUD), daemon=True
    )
    reader_thread.start()

    # 2. Connect to UGV flight controller
    master = connect_fc(FC_PORT, FC_BAUD)

    # Request LOCAL_POSITION_NED at 20 Hz
    set_msg_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 20, 1
    )

    try:
        change_mode(master, "GUIDED")
        arm(master)

        print("\n[INFO] UGV ready.  Waiting for UAV position before moving.\n")

        navigate_to_uav(master)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
        stop_ugv(master, duration=2.0)
    except Exception as e:
        print(f"\n[ERROR] {e}")
        try:
            stop_ugv(master, duration=2.0)
        except Exception:
            pass
    finally:
        try:
            disarm(master)
        except Exception as e:
            print(f"[WARN] Disarm: {e}")
        print("[INFO] Navigator exited.")

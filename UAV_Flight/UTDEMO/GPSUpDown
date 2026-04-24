"""
VLand GPS – Takeoff, hover (GPS hold), then land
Robust version with timeouts and safety checks
"""

from pymavlink import mavutil
import time

# ─── CONNECTION ─────────────────────────────────────────────
CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE = 57600

# ─── FLIGHT PARAMS ──────────────────────────────────────────
TARGET_ALT_M = 3.0
HOVER_TIMEOUT_S = 10
LAND_TIMEOUT_S = 90

# ─── LOGGING ────────────────────────────────────────────────
def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")

# ─── MAVLINK SETUP ──────────────────────────────────────────
def connect():
    log(f"Connecting to {CONNECTION_STRING}")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    log("Heartbeat received")

    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1
    )
    return master

# ─── GPS CHECK ──────────────────────────────────────────────
def wait_for_gps(master, timeout=30):
    log("Waiting for strong GPS fix...")
    start = time.time()

    while time.time() - start < timeout:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if msg and msg.fix_type >= 3 and msg.satellites_visible >= 8:
            log(f"GPS OK (fix={msg.fix_type}, sats={msg.satellites_visible})")
            return True

    log("GPS timeout!")
    return False

# ─── POSITION CHECK ─────────────────────────────────────────
def wait_for_position(master, timeout=30):
    log("Waiting for position estimate (EKF)...")
    start = time.time()

    while time.time() - start < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            log("Position estimate acquired")
            return True

    log("Position timeout!")
    return False

# ─── MODE CONTROL ───────────────────────────────────────────
def set_mode(master, mode):
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise Exception(f"Mode {mode} not supported. Available: {list(mapping.keys())}")

    mode_id = mapping[mode]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    log(f"Mode -> {mode}")

    # Confirm mode change
    deadline = time.time() + 5
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb:
            current_mode = mavutil.mode_string_v10(hb)
            if current_mode == mode:
                log(f"Confirmed mode: {mode}")
                return True

    log(f"Warning: Mode change to {mode} not confirmed")
    return False

# ─── ARMING ─────────────────────────────────────────────────
def arm(master):
    log("Arming...")
    master.arducopter_arm()

    deadline = time.time() + 15
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log("Armed successfully")
            return True

    log("Failed to arm (check pre-arm errors)")
    return False

# ─── TAKEOFF ────────────────────────────────────────────────
def takeoff(master, alt):
    log(f"Taking off to {alt} m")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt
    )

    start = time.time()

    while time.time() - start < 25:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg:
            rel_alt = msg.relative_alt / 1000.0
            print(f"Altitude: {rel_alt:.2f} m", end="\r")

            if rel_alt >= alt * 0.95:
                print()
                log("Reached target altitude")
                return True

    print()
    log("Takeoff timeout!")
    return False

# ─── LANDING ────────────────────────────────────────────────
def land(master):
    log("Landing...")
    set_mode(master, "LAND")

    deadline = time.time() + LAND_TIMEOUT_S

    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb:
            armed = hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:
                log("Disarmed — landed")
                return

    log("Landing timeout — forcing disarm")
    master.arducopter_disarm()

# ─── MAIN ───────────────────────────────────────────────────
def main():
    master = connect()

    try:
        if not wait_for_gps(master):
            return

        if not wait_for_position(master):
            return

        if not set_mode(master, "GUIDED"):
            log("Mode change failed — aborting")
            return

        if not arm(master):
            return

        time.sleep(2)  # stabilization delay

        if not takeoff(master, TARGET_ALT_M):
            log("Takeoff failed — landing")
            land(master)
            return

        log(f"Hovering for {HOVER_TIMEOUT_S} seconds (GPS hold)")
        time.sleep(HOVER_TIMEOUT_S)

        land(master)

    except KeyboardInterrupt:
        log("Emergency interrupt — landing")
        land(master)

if __name__ == "__main__":
    main()

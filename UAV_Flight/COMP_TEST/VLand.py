"""
VLand – Vertical takeoff, hover for a configurable timeout, then land.
Standalone script (no UAVcommander dependency).

Features:
  - USE_VELOCITY_COMMANDS boolean toggles between RC-override and GUIDED
    velocity setpoints for climb/hover control.
  - Continuously prints optical-flow X/Y position and LiDAR rangefinder height.
  - KeyboardInterrupt instantly commands LAND mode.
  - RC emergency-land switch override checked every loop iteration.
"""

from pymavlink import mavutil
import time

# ─── CONNECTION ──────────────────────────────────────────────────────────────────
CONNECTION_STRING = "/dev/ttyACM0"     # or "udp:127.0.0.1:14551" for SITL
BAUD_RATE         = 57600

# ─── FLIGHT PARAMS ───────────────────────────────────────────────────────────────
TARGET_ALT_M       = 2.3              # metres to climb to
HOVER_TIMEOUT_S    = 10.0             # seconds to hover before auto-landing
ALT_TOLERANCE_M    = 0.25             # how close to target counts as "reached"
CLIMB_LOOP_DT      = 0.10             # loop period during climb
HOVER_LOOP_DT      = 0.10             # loop period during hover
LAND_TIMEOUT_S     = 60.0             # safety timeout waiting for disarm after LAND

# ─── CONTROL MODE TOGGLE ────────────────────────────────────────────────────────
# Both modes always use ALT_HOLD – no mid-air mode switches except LAND.
# True  → proportional throttle controller (altitude error → throttle PWM trim)
# False → fixed PWM steps (THROTTLE_CLIMB to climb, THROTTLE_HOVER to hold)
USE_VELOCITY_COMMANDS = True

# ─── RC OVERRIDE TUNING (only used when USE_VELOCITY_COMMANDS is False) ──────────
THROTTLE_MIN   = 1000
THROTTLE_IDLE  = 1150
THROTTLE_CLIMB = 1650
THROTTLE_HOVER = 1500

# ─── VELOCITY TUNING (only used when USE_VELOCITY_COMMANDS is True) ──────────────
CLIMB_SPEED_MPS  = 0.5               # upward velocity while climbing (m/s)
HOVER_VZ_TRIM    = 0.0               # small trim if it drifts (m/s, positive = down)

# ─── RC EMERGENCY LAND SWITCH ───────────────────────────────────────────────────
EMERGENCY_LAND_CH            = 7      # RC channel (1-indexed)
EMERGENCY_LAND_PWM_THRESHOLD = 1800   # PWM above which the switch is "flipped"

# ─── LOGGING ─────────────────────────────────────────────────────────────────────
LOG_FILE = "vland_log.txt"


def log_event(text):
    ts = time.strftime("%H:%M:%S")
    line = f"[{ts}] {text}"
    print(line)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")


# ─── MAVLINK HELPERS ─────────────────────────────────────────────────────────────
def connect():
    log_event(f"Connecting to {CONNECTION_STRING} ...")
    if CONNECTION_STRING.startswith(("udp:", "tcp:")):
        master = mavutil.mavlink_connection(CONNECTION_STRING)
    else:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    hb = master.wait_heartbeat(timeout=10)
    if not hb:
        raise RuntimeError("No heartbeat – check connection")
    log_event("Heartbeat received.")
    request_message_streams(master)
    return master


def request_message_streams(master):
    try:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    except Exception:
        pass

    def set_interval(msg_id, hz):
        try:
            us = int(1e6 / hz)
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, us, 0, 0, 0, 0, 0)
        except Exception:
            pass

    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 15)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 5)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW, 10)


def change_mode(master, *mode_names):
    mapping = master.mode_mapping()
    for mode in mode_names:
        if mode in mapping:
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mapping[mode])
            log_event(f"Mode → {mode}")
            time.sleep(1)
            return mode
    raise RuntimeError(f"No valid mode in {mode_names}. Available: {list(mapping.keys())}")


def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    log_event("Arm command sent.")
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack:
        log_event(f"Arm ACK result: {ack.result}")


def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)
    log_event("Disarm command sent.")


# ─── RC OVERRIDE (PWM) ──────────────────────────────────────────────────────────
def set_throttle(master, pwm):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, int(pwm), 0, 0, 0, 0, 0)


def set_rc_override(master, roll=THROTTLE_HOVER, pitch=THROTTLE_HOVER,
                    throttle=THROTTLE_HOVER, yaw=THROTTLE_HOVER):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        int(roll), int(pitch), int(throttle), int(yaw), 0, 0, 0, 0)


def clear_rc_override(master):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0)


# ─── VELOCITY COMMANDS (GUIDED MODE) ────────────────────────────────────────────
def send_body_velocity(master, vx, vy, vz):
    """Body-frame velocity: vx=forward, vy=right, vz=down (positive descends)."""
    TYPE_MASK = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system, master.target_component,
        8,  # MAV_FRAME_BODY_NED
        TYPE_MASK,
        0, 0, 0,
        float(vx), float(vy), float(vz),
        0, 0, 0,
        0, 0)


def send_takeoff_command(master, alt):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, alt)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack is None:
        log_event("[WARN] No ACK for takeoff command")
    elif ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        log_event(f"[WARN] Takeoff rejected (result={ack.result})")
    else:
        log_event("Takeoff command accepted.")


# ─── SENSOR READERS ─────────────────────────────────────────────────────────────
def get_rangefinder_alt(master):
    """LiDAR / rangefinder altitude in metres (returns None if unavailable)."""
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    while msg:
        alt = msg.current_distance / 100.0
        if alt > 0.01:
            return alt
        msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    return None


def get_baro_relative_alt(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg:
        return msg.relative_alt / 1000.0
    return None


def get_altitude(master):
    """Returns (alt_m, source_str) preferring rangefinder over baro."""
    rng = get_rangefinder_alt(master)
    if rng is not None:
        return rng, "lidar"
    baro = get_baro_relative_alt(master)
    if baro is not None:
        return baro, "baro"
    return None, "none"


def get_optical_flow(master):
    """Returns (flow_x, flow_y, quality) or (None, None, None)."""
    msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
    if msg:
        return msg.flow_x, msg.flow_y, msg.quality
    return None, None, None


def print_sensors(master):
    """Print optical-flow position and LiDAR height on a single updating line."""
    fx, fy, qual = get_optical_flow(master)
    alt, src = get_altitude(master)

    flow_str = (f"Flow X:{fx:7.2f}  Y:{fy:7.2f}  Q:{qual}"
                if fx is not None else "Flow: no data")
    alt_str = (f"Alt:{alt:5.2f}m ({src})"
               if alt is not None else "Alt: no data")
    print(f"  {flow_str}  |  {alt_str}", end="\r", flush=True)
    return alt


def check_rc_emergency_switch(master):
    """Returns True if the designated RC channel exceeded the land threshold."""
    msg = master.recv_match(type='RC_CHANNELS', blocking=False)
    if msg is not None:
        ch_val = getattr(msg, f'chan{EMERGENCY_LAND_CH}_raw', None)
        if ch_val is not None and int(ch_val) > EMERGENCY_LAND_PWM_THRESHOLD:
            return True
    return False


# ─── LANDING ─────────────────────────────────────────────────────────────────────
def land_safely(master):
    log_event("Commanding LAND mode...")
    change_mode(master, "LAND")
    clear_rc_override(master)

    log_event("Waiting for auto-disarm (heartbeat confirmation)...")
    deadline = time.time() + LAND_TIMEOUT_S
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
        if hb is not None:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                print()
                log_event("Motors disarmed – touchdown confirmed.")
                return
        print_sensors(master)
        time.sleep(0.25)

    print()
    log_event("[WARN] Land timeout – sending disarm fallback.")
    disarm(master)
    time.sleep(2.0)


# ─── MAIN MISSION ───────────────────────────────────────────────────────────────
def main():
    master = connect()

    try:
        # ── ARM & INITIAL MODE ────────────────────────────────────────────
        change_mode(master, "ALT_HOLD", "ALTHOLD")
        arm(master)
        # if not master.motors_armed():
        #     log_event("Failed to arm – aborting.")
        #     return

        log_event(
            f"{'VELOCITY (proportional throttle)' if USE_VELOCITY_COMMANDS else 'RC OVERRIDE (fixed PWM)'} "
            "mode active – ALT_HOLD throughout")

        # ── CLIMB ─────────────────────────────────────────────────────────
        log_event(f"Climbing to {TARGET_ALT_M:.2f} m ...")
        stable_start = None

        while True:
            if check_rc_emergency_switch(master):
                log_event("RC EMERGENCY SWITCH – landing immediately")
                land_safely(master)
                return

            alt = print_sensors(master)

            if USE_VELOCITY_COMMANDS:
                # Proportional throttle – in ALT_HOLD the throttle channel commands
                # vertical rate, so this is a velocity-proportional altitude controller.
                if alt is not None:
                    err = TARGET_ALT_M - alt
                    thr = int(THROTTLE_HOVER + 150 * err)
                    thr = max(THROTTLE_MIN, min(1800, thr))
                else:
                    thr = THROTTLE_CLIMB
                set_throttle(master, thr)
                if alt is not None and alt >= (TARGET_ALT_M - ALT_TOLERANCE_M):
                    if stable_start is None:
                        stable_start = time.time()
                    elif (time.time() - stable_start) >= 1.0:
                        print()
                        log_event(f"Target altitude reached: {alt:.2f} m")
                        break
                else:
                    stable_start = None
            else:
                if alt is not None and alt >= (TARGET_ALT_M - ALT_TOLERANCE_M):
                    set_throttle(master, THROTTLE_HOVER)
                    if stable_start is None:
                        stable_start = time.time()
                    elif (time.time() - stable_start) >= 1.2:
                        print()
                        log_event(f"Target altitude reached: {alt:.2f} m")
                        break
                else:
                    set_throttle(master, THROTTLE_CLIMB)
                    stable_start = None

            time.sleep(CLIMB_LOOP_DT)

        # ── HOVER ─────────────────────────────────────────────────────────
        log_event(f"Hovering for {HOVER_TIMEOUT_S:.1f} s ...")
        hover_start = time.time()

        while (time.time() - hover_start) < HOVER_TIMEOUT_S:
            if check_rc_emergency_switch(master):
                log_event("RC EMERGENCY SWITCH – landing immediately")
                land_safely(master)
                return

            alt = print_sensors(master)

            if USE_VELOCITY_COMMANDS:
                # Proportional throttle to hold target altitude (gentler gain in hover)
                if alt is not None:
                    err = TARGET_ALT_M - alt
                    thr = int(THROTTLE_HOVER + 80 * err)
                    thr = max(1400, min(1600, thr))
                else:
                    thr = THROTTLE_HOVER
                set_throttle(master, thr)
            else:
                thr = THROTTLE_HOVER
                if alt is not None:
                    if alt < TARGET_ALT_M - 0.20:
                        thr = THROTTLE_HOVER + 40
                    elif alt > TARGET_ALT_M + 0.20:
                        thr = THROTTLE_HOVER - 40
                set_throttle(master, thr)

            time.sleep(HOVER_LOOP_DT)

        print()
        log_event("Hover timeout reached – landing.")

        # ── LAND ──────────────────────────────────────────────────────────
        land_safely(master)

    except KeyboardInterrupt:
        print()
        log_event("KEYBOARD INTERRUPT – commanding immediate LAND")
        try:
            land_safely(master)
        except Exception:
            pass
    finally:
        clear_rc_override(master)
        log_event("Script finished.")


if __name__ == "__main__":
    main()

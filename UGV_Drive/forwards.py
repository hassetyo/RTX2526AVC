# forwards.py
# Simple ArduRover/UGV script:
#   1) Connect via MAVLink (Mission Planner/SITL)
#   2) GUIDED mode
#   3) Arm
#   4) Drive forward ~5 meters (no GPS; integrates reported ground speed if available)
#   5) Stop
#   6) Disarm
#
# Notes:
# - “No GPS” means we do NOT use GLOBAL_POSITION_INT / GPS. Distance is estimated from
#   speed feedback (VFR_HUD.groundspeed) when available; otherwise it falls back to time*commanded_speed.
# - In SITL + Mission Planner, this is typically good enough for a simple test.

from pymavlink import mavutil
import time

# ---- CONFIG ----
CONNECTION_STRING = "/dev/ttyACM0"   # common for Rover SITL; change if needed
FORWARD_DISTANCE_M = 5.0
TARGET_SPEED_MS = 1.0
CONTROL_HZ = 20
TIMEOUT_S = 30.0

print(f"Connecting to {CONNECTION_STRING} ...")
master = mavutil.mavlink_connection(CONNECTION_STRING)
master.wait_heartbeat()
print(f"Heartbeat found (sys={master.target_system}, comp={master.target_component})")


def request_message_intervals():
    """Ask autopilot to stream messages we care about (best-effort)."""
    def set_interval(msg_id, hz):
        us = int(1e6 / max(hz, 1e-3))
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            us,
            0, 0, 0, 0, 0
        )

    try:
        set_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)
        set_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 10)  # groundspeed
    except Exception:
        pass


request_message_intervals()


def change_mode(mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise RuntimeError(f"Unknown mode '{mode}'. Available: {list(mapping.keys())}")
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode: {mode}")
    time.sleep(1.0)


def is_armed() -> bool:
    hb = master.recv_match(type="HEARTBEAT", blocking=False)
    if hb is None:
        return False
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def wait_armed_state(want_armed: bool, timeout_s: float = 10.0):
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if hb is None:
            continue
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed == want_armed:
            return
    raise RuntimeError(f"Timed out waiting for armed={want_armed}")


def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming...")
    wait_armed_state(True, timeout_s=15.0)
    print("ARMED")


def disarm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Disarming...")
    wait_armed_state(False, timeout_s=15.0)
    print("DISARMED")


def get_groundspeed(timeout=0.0):
    """
    Returns groundspeed (m/s) from VFR_HUD if available.
    Does NOT use GPS.
    """
    msg = master.recv_match(type="VFR_HUD", blocking=(timeout > 0), timeout=timeout)
    if not msg:
        return None
    try:
        return float(msg.groundspeed)
    except Exception:
        return None


def send_velocity_body_forward(vx_ms: float):
    """
    Send forward velocity command in BODY_NED frame.
    For a ground rover, 'forward' is +X in body frame.
    """
    # Use velocity only, ignore position/accel/yaw/yaw_rate
    type_mask = 0b0000111111000111
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (0 is fine for offboard)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0, 0, 0,          # x, y, z (ignored)
        vx_ms, 0.0, 0.0,  # vx, vy, vz
        0, 0, 0,          # ax, ay, az (ignored)
        0, 0              # yaw, yaw_rate (ignored)
    )


def stop_rover(duration_s: float = 1.0):
    end = time.time() + duration_s
    while time.time() < end:
        send_velocity_body_forward(0.0)
        time.sleep(0.05)


def drive_forward_distance(distance_m: float, speed_ms: float, timeout_s: float):
    """
    Drives forward until estimated distance reached.
    Estimation priority:
      1) integrate VFR_HUD.groundspeed (no GPS)
      2) fallback integrate commanded speed if no VFR_HUD data arrives
    """
    print(f"Driving forward ~{distance_m:.2f} m at {speed_ms:.2f} m/s (no GPS)")

    dt = 1.0 / CONTROL_HZ
    t_end = time.time() + timeout_s
    dist = 0.0
    last = time.time()

    while time.time() < t_end and dist < distance_m:
        now = time.time()
        loop_dt = max(now - last, dt)
        last = now

        gs = get_groundspeed(timeout=0.0)
        used_speed = gs if (gs is not None and gs > 0.01) else abs(speed_ms)

        dist += used_speed * loop_dt

        send_velocity_body_forward(speed_ms)
        time.sleep(dt)

    stop_rover(1.0)

    if dist >= distance_m:
        print(f"Reached target (estimated dist={dist:.2f} m)")
    else:
        raise RuntimeError(f"Timeout before reaching distance (estimated dist={dist:.2f} m)")


if __name__ == "__main__":
    # For Rover, GUIDED is typical for offboard control
    change_mode("GUIDED")
    arm()

    try:
        drive_forward_distance(FORWARD_DISTANCE_M, TARGET_SPEED_MS, TIMEOUT_S)
    finally:
        # Always try to stop + disarm
        stop_rover(1.0)
        disarm()
from pymavlink import mavutil
import time
import math

# =========================
# CONFIG
# =========================
CONNECTION_STRING = "udp:127.0.0.1:14551"     # e.g. "COM6" on Windows, or "udp:127.0.0.1:14551" in SITL
BAUD_RATE = 57600

TARGET_DISTANCE_M = 5.0
SPEED_MPS = 0.6                # conservative start; increase later if needed
DIST_TOL_M = 0.10              # stop tolerance
COMMAND_HZ = 10                # resend guided velocity command at 10 Hz
MOVE_TIMEOUT_S = 20            # safety timeout

# If LOCAL_POSITION_NED is unavailable, fallback to integrating groundspeed
USE_GROUNDSPEED_FALLBACK = True

# =========================
# CONNECT
# =========================
print(f"Connecting to {CONNECTION_STRING} at {BAUD_RATE} baud...")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
master.wait_heartbeat()
print(f"Heartbeat found (sys={master.target_system}, comp={master.target_component})")


# =========================
# HELPERS
# =========================
def request_message_streams():
    """Request telemetry streams so LOCAL_POSITION_NED / VFR_HUD updates arrive regularly."""
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # rate Hz
            1    # start
        )
    except Exception:
        pass

    # Also try MAVLink2 message intervals (not all firmwares honor this, but harmless)
    def set_interval(msg_id, hz):
        us = int(1e6 / hz)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            us,
            0, 0, 0, 0, 0
        )

    try:
        set_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
        set_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 10)
    except Exception:
        pass


def change_mode(mode: str):
    mapping = master.mode_mapping()
    if mapping is None:
        raise RuntimeError("Mode mapping unavailable from vehicle.")
    if mode not in mapping:
        raise RuntimeError(f"Unknown mode '{mode}'. Available: {list(mapping.keys())}")

    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode change sent: {mode}")

    # Wait a bit and verify heartbeat mode if possible
    t0 = time.time()
    while time.time() - t0 < 5:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None:
            continue
        # base_mode/custom_mode verification can be firmware-specific; heartbeat receipt is enough here
        print("Heartbeat received after mode change.")
        break
    time.sleep(1.0)


def arm():
    print("Arming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

    # Wait for motors armed flag
    t0 = time.time()
    while time.time() - t0 < 10:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("Vehicle armed.")
            return
    print("Warning: Arm not confirmed via heartbeat (continuing cautiously).")


def disarm():
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)


def send_body_velocity(vx_mps: float, vy_mps: float = 0.0, vz_mps: float = 0.0, yaw_rate_rads: float = 0.0):
    """
    Send velocity command in BODY_NED frame.
    For rover, vx forward is what matters.
    """
    # Type mask bits set to IGNORE fields we are NOT controlling.
    # We want velocity + yaw_rate only.
    # Ignore position (x,y,z), acceleration (afx,afy,afz), yaw
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        # yaw_rate is used (not ignored)
    )

    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,   # time_boot_ms (best-effort)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0, 0, 0,                  # x, y, z (ignored)
        vx_mps, vy_mps, vz_mps,    # velocity
        0, 0, 0,                   # acceleration (ignored)
        0,                         # yaw (ignored)
        yaw_rate_rads              # yaw_rate
    )


def stop_vehicle():
    # Send multiple stop commands to ensure rover halts
    print("Stopping vehicle...")
    for _ in range(5):
        send_body_velocity(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.1)


def get_local_position_ned(timeout=0.2):
    """Return latest LOCAL_POSITION_NED msg or None."""
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    return msg


def get_vfr_hud(timeout=0.05):
    """Return latest VFR_HUD msg or None."""
    msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=timeout)
    return msg


# =========================
# MAIN MOVE LOGIC
# =========================
def move_forward_distance_guided_no_gps(distance_m: float, speed_mps: float):
    """
    Drive forward in GUIDED mode for `distance_m`.
    Primary distance source: LOCAL_POSITION_NED (EKF local position from encoders/odom)
    Fallback: integrate VFR_HUD.groundspeed
    """
    request_message_streams()

    # Wait briefly for position/telemetry
    print("Waiting for telemetry (LOCAL_POSITION_NED / VFR_HUD)...")
    start_lp = None
    t_wait = time.time()
    while time.time() - t_wait < 3.0:
        lp = get_local_position_ned(timeout=0.2)
        if lp is not None:
            start_lp = lp
            print(f"LOCAL_POSITION_NED available: x={lp.x:.3f}, y={lp.y:.3f}, z={lp.z:.3f}")
            break

    if start_lp is None:
        if not USE_GROUNDSPEED_FALLBACK:
            raise RuntimeError("LOCAL_POSITION_NED not available and fallback disabled.")
        print("LOCAL_POSITION_NED not available. Falling back to VFR_HUD ground speed integration.")

    traveled = 0.0
    t0 = time.time()
    last_time = t0

    # For LOCAL_POSITION_NED
    x0 = y0 = None
    if start_lp is not None:
        x0, y0 = start_lp.x, start_lp.y

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # Keep sending guided velocity command (required continuously)
        send_body_velocity(speed_mps, 0.0, 0.0, 0.0)

        # Measure distance
        lp = get_local_position_ned(timeout=0.01)
        if lp is not None and x0 is not None:
            # NED local frame: x/y displacement magnitude in plane
            dx = lp.x - x0
            dy = lp.y - y0
            traveled = math.sqrt(dx * dx + dy * dy)
        else:
            # Fallback: integrate ground speed (no GPS required if speed estimate exists from wheel encoders/EKF)
            vfr = get_vfr_hud(timeout=0.01)
            if vfr is not None:
                gs = float(vfr.groundspeed)
                traveled += max(gs, 0.0) * dt
            # else no update this cycle; keep last traveled

        print(f"\rTraveled: {traveled:.2f} / {distance_m:.2f} m", end="", flush=True)

        # Stop condition
        if traveled >= (distance_m - DIST_TOL_M):
            print()
            print("Target distance reached.")
            break

        # Timeout safety
        if (now - t0) > MOVE_TIMEOUT_S:
            print()
            print("Timeout reached before target distance.")
            break

        time.sleep(1.0 / COMMAND_HZ)

    stop_vehicle()


if __name__ == "__main__":
    try:
        # IMPORTANT: Rover must be configured to allow GUIDED without GPS using wheel encoders/odometry EKF
        change_mode("GUIDED")
        arm()

        print("\n*** Ensure safety switch / arming safety is satisfied before motion ***\n")
        time.sleep(2)

        move_forward_distance_guided_no_gps(TARGET_DISTANCE_M, SPEED_MPS)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        stop_vehicle()
    except Exception as e:
        print(f"\nERROR: {e}")
        try:
            stop_vehicle()
        except Exception:
            pass
    finally:
        try:
            disarm()
        except Exception as e:
            print(f"Disarm warning: {e}")
        print("Test complete.")
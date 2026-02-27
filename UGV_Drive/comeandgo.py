from pymavlink import mavutil
import time
import math

# =========================
# CONFIG
# =========================
CONNECTION_STRING = 'COM3'   # Use 'udp:127.0.0.1:14551' for SITL, or 'COM3' for your telemetry radio
BAUD_RATE = 57600

TARGET_DISTANCE_M = 5.0
SPEED_MPS = 0.6
TURN_SPEED_MPS = 0.3         # Slightly slower for better turning precision
MAX_YAW_RATE = 0.6           # Maximum turn speed (rad/s)
MIN_YAW_RATE = 0.15          # Minimum speed to ensure motors keep moving
KP_YAW = 0.03                # Proportional gain: higher = more aggressive turn

DIST_TOL_M = 0.10
HEADING_TOL_DEG = 2.0
COMMAND_HZ = 10
MOVE_TIMEOUT_S = 30

USE_GROUNDSPEED_FALLBACK = True

# =========================
# CONNECT & INITIALIZE
# =========================
print(f"Connecting to {CONNECTION_STRING} at {BAUD_RATE} baud...")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
master.wait_heartbeat()
print(f"Heartbeat found (sys={master.target_system}, comp={master.target_component})")

def request_message_streams():
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1
        )
    except Exception:
        pass

def change_mode(mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise RuntimeError(f"Unknown mode '{mode}'")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode]
    )
    time.sleep(1.0)

def arm():
    print("Arming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2.0)

def disarm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

def send_body_velocity(vx_mps: float, yaw_rate_rads: float = 0.0):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )

    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0, 0, 0,
        vx_mps, 0, 0,
        0, 0, 0,
        0,
        yaw_rate_rads
    )

def stop_vehicle():
    for _ in range(5):
        send_body_velocity(0.0, 0.0)
        time.sleep(0.1)

def get_vfr_hud():
    return master.recv_match(type='VFR_HUD', blocking=True, timeout=0.2)

def get_local_position_ned():
    return master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.2)

# =========================
# SMART LOGIC
# =========================

def turn_heading_smart(relative_angle_deg: float):
    """
    Turns the vehicle using proportional control (Smart Stop).
    Automatically determines the shortest direction to turn.
    """
    request_message_streams()
    msg = get_vfr_hud()
    if msg is None:
        raise RuntimeError("No VFR_HUD message received for heading.")

    start_heading = float(msg.heading)
    target_heading = (start_heading + relative_angle_deg) % 360
    print(f"Start Heading: {start_heading:.1f} | Target: {target_heading:.1f}")

    t0 = time.time()
    target_yaw_rate = 0.0  # Fix: initialize so it always exists

    while True:
        msg = get_vfr_hud()

        if msg is not None:
            current_heading = float(msg.heading)

            # Shortest signed heading error in range [-180, 180]
            error = (target_heading - current_heading + 180) % 360 - 180

            # Proportional yaw-rate command
            target_yaw_rate = error * KP_YAW

            # Clamp max yaw rate
            if abs(target_yaw_rate) > MAX_YAW_RATE:
                target_yaw_rate = math.copysign(MAX_YAW_RATE, target_yaw_rate)

            # Clamp min yaw rate only if still outside heading tolerance
            if abs(error) > HEADING_TOL_DEG and abs(target_yaw_rate) < MIN_YAW_RATE:
                target_yaw_rate = math.copysign(MIN_YAW_RATE, target_yaw_rate)

            print(
                f"\rHeading: {current_heading:.1f} | Error: {error:.1f} deg | Rate: {target_yaw_rate:.2f}    ",
                end="",
                flush=True
            )

            if abs(error) <= HEADING_TOL_DEG:
                print("\nTarget heading reached.")
                break

            send_body_velocity(TURN_SPEED_MPS, target_yaw_rate)
        else:
            print("\rWaiting for heading update...    ", end="", flush=True)

        if (time.time() - t0) > MOVE_TIMEOUT_S:
            print("\nTurn timeout reached.")
            break

        time.sleep(1.0 / COMMAND_HZ)

    stop_vehicle()

def move_forward_distance_smart(distance_m: float, speed_mps: float):
    request_message_streams()
    start_lp = get_local_position_ned()
    x0, y0 = (start_lp.x, start_lp.y) if start_lp else (0, 0)

    traveled = 0.0
    t0 = last_time = time.time()

    while traveled < (distance_m - DIST_TOL_M):
        now = time.time()
        send_body_velocity(speed_mps, 0.0)

        lp = get_local_position_ned()
        if lp and start_lp:
            traveled = math.sqrt((lp.x - x0) ** 2 + (lp.y - y0) ** 2)
        else:
            vfr = get_vfr_hud()
            if vfr and USE_GROUNDSPEED_FALLBACK:
                traveled += float(vfr.groundspeed) * (now - last_time)

        print(f"\rTraveled: {traveled:.2f} / {distance_m:.2f} m    ", end="", flush=True)

        if (now - t0) > MOVE_TIMEOUT_S:
            print("\nMove timeout reached.")
            break

        last_time = now
        time.sleep(1.0 / COMMAND_HZ)

    print("\nDistance reached.")
    stop_vehicle()

# =========================
# MAIN EXECUTION
# =========================
if __name__ == "__main__":
    try:
        change_mode("GUIDED")
        arm()
        time.sleep(2)

        print(">>> STEP 1: FORWARD 5m <<<")
        move_forward_distance_smart(TARGET_DISTANCE_M, SPEED_MPS)

        print("\n>>> STEP 2: SMART TURN 180 <<<")
        turn_heading_smart(180)

        print("\n>>> STEP 3: RETURN 5m <<<")
        move_forward_distance_smart(TARGET_DISTANCE_M, SPEED_MPS)

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
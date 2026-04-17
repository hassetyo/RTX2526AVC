from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math
import v2v_bridge

# ==========================================
# ARUCO GROUND STATION
# Behavior:
#   1) arm and enter GUIDED at startup
#   2) receive turn / move commands from UAV bridge
#   3) turn using compass-based heading control
#   4) move forward using body-frame velocity
#   5) stop cleanly
# ==========================================

# ----------------------------
# Ports / connections
# ----------------------------
UGV_CONTROL_PORT = "/dev/ttyACM0"
UGV_BAUD_RATE = 115200
ESP32_BRIDGE_PORT = "/dev/ttyUSB0"

# ----------------------------
# Motion tuning
# ----------------------------
SPEED_MPH = 5.0
MPH_TO_MPS = 0.44704
SPEED_MPS = SPEED_MPH * MPH_TO_MPS

# Small turn step per UAV correction command
TURN_STEP_DEG = 12.0
TURN_RATE_DEG_S = 90.0
TURN_TOLERANCE_DEG = 5.0

# Forward move defaults
MOVE_FORWARD_10FT_M = 3.048
MOVE_FORWARD_2FT_M = 0.61

# Body GOTO parsing
GOTO_BODY_Y_EPS = 1e-3

# Movement detection
MOVEMENT_EPS_MPS = 0.05

# Telemetry / control timing
TELEM_SEND_HZ = 5
HEADING_CHECK_INTERVAL_S = 0.20
STOP_EARLY_DEG = 8.0
STABLE_COUNT_REQUIRED = 2

print("==========================================")
print("   UGV GROUND STATION - ARUCO READY")
print("==========================================")
print(f"[Ground] Connecting to UGV at {UGV_CONTROL_PORT}...")

try:
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)
    print("[Ground] Connected! Ready to sync.")
except Exception as e:
    print(f"!!! Error connecting to UGV: {e} !!!")
    raise SystemExit(1)


def wait_for_mode(vehicle, mode_name, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while vehicle.mode.name != mode_name and time.time() < deadline:
        time.sleep(0.1)
    return vehicle.mode.name == mode_name


def wait_for_armed(vehicle, armed_state, timeout_s=3.0):
    deadline = time.time() + timeout_s
    while vehicle.armed != armed_state and time.time() < deadline:
        time.sleep(0.1)
    return vehicle.armed == armed_state


def broadcast_status(vehicle, bridge, seq):
    armed_val = 1 if vehicle.armed else 0
    mode_name = vehicle.mode.name
    mode_idx = v2v_bridge.MODE_INITIAL

    if mode_name == "GUIDED":
        mode_idx = v2v_bridge.MODE_GUIDED
    elif mode_name == "AUTO":
        mode_idx = v2v_bridge.MODE_AUTO
    elif mode_name == "LAND":
        mode_idx = v2v_bridge.MODE_LAND

    armable_bit = 0x10 if vehicle.is_armable else 0x00
    gps_fix = 0
    try:
        gps_fix = vehicle.gps_0.fix_type
    except Exception:
        gps_fix = 0
    gps_bit = 0x20 if gps_fix > 0 else 0x00

    safety_byte = (mode_idx & 0x0F) | armable_bit | gps_bit
    t_ms = int(time.time() * 1000) & 0xFFFFFFFF
    v_mps = vehicle.groundspeed if vehicle.groundspeed is not None else 0.0

    bridge.send_telemetry(seq, t_ms, v_mps, 0.0, armed_val, safety_byte)


def arm_ugv(vehicle):
    print("\n[Ground] >>> ARMING UGV")

    # This matches the working behavior from your Challenge 2 file.
    for label, state in [
        ("FIRST ARM", True),
        ("RESET DISARM", False),
        ("FINAL ARM", True),
    ]:
        print(f"[Ground] {label}...")
        vehicle.armed = state
        if not wait_for_armed(vehicle, state):
            raise RuntimeError(f"Failed to set armed={state}")
        time.sleep(1.0)

    print("[Ground] Armed confirmed.")

    vehicle.mode = VehicleMode("GUIDED")
    if not wait_for_mode(vehicle, "GUIDED"):
        raise RuntimeError(f"Failed to enter GUIDED mode, current mode: {vehicle.mode.name}")

    print("[Ground] GUIDED mode confirmed.")
    print("[Ground] UGV FULLY ARMED AND IN GUIDED MODE.\n")


def ensure_ready(vehicle):
    if not vehicle.armed:
        print("[Ground] Vehicle not armed. Re-arming...")
        arm_ugv(vehicle)
    elif vehicle.mode.name != "GUIDED":
        print(f"[Ground] Vehicle mode is {vehicle.mode.name}. Switching to GUIDED...")
        vehicle.mode = VehicleMode("GUIDED")
        if not wait_for_mode(vehicle, "GUIDED"):
            raise RuntimeError(f"Failed to re-enter GUIDED mode, current mode: {vehicle.mode.name}")


def build_velocity_msg(vehicle, speed_mps):
    return vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0x0DE7,
        0,
        0,
        0,
        speed_mps,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def build_stop_msg(vehicle):
    return build_velocity_msg(vehicle, 0.0)


def build_attitude_msg(vehicle, throttle_fraction, yaw_rate_deg_s=0.0):
    return vehicle.message_factory.set_attitude_target_encode(
        0,
        0,
        0,
        0xA3,
        [1.0, 0.0, 0.0, 0.0],
        0.0,
        0.0,
        math.radians(yaw_rate_deg_s),
        throttle_fraction,
    )


def send_stop(vehicle):
    vehicle.send_mavlink(build_stop_msg(vehicle))
    time.sleep(0.5)


def get_groundspeed(vehicle):
    return vehicle.groundspeed if vehicle.groundspeed is not None else 0.0


def get_heading(vehicle):
    h = vehicle.heading
    if h is None:
        raise RuntimeError("vehicle.heading is unavailable")
    return float(h)


def angle_diff_deg(current_deg, start_deg):
    return ((current_deg - start_deg + 540) % 360) - 180


def drive_distance_velocity(vehicle, bridge, distance_m, speed_mps, detection_window_s=1.5):
    if distance_m <= 0:
        send_stop(vehicle)
        return True

    duration_s = distance_m / speed_mps
    drive_msg = build_velocity_msg(vehicle, speed_mps)
    stop_msg = build_stop_msg(vehicle)

    start_t = time.time()
    movement_detected = False
    seq = 0

    while (time.time() - start_t) < duration_s:
        vehicle.send_mavlink(drive_msg)
        broadcast_status(vehicle, bridge, seq)
        seq += 1

        elapsed = time.time() - start_t
        groundspeed = get_groundspeed(vehicle)

        if groundspeed >= MOVEMENT_EPS_MPS:
            movement_detected = True

        if elapsed >= detection_window_s and not movement_detected:
            break

        time.sleep(0.1)

    vehicle.send_mavlink(stop_msg)
    time.sleep(0.5)
    return movement_detected


def drive_distance_attitude(vehicle, bridge, distance_m, speed_mps):
    if distance_m <= 0:
        send_stop(vehicle)
        return

    duration_s = distance_m / speed_mps

    if "WP_SPEED" not in vehicle.parameters:
        raise RuntimeError("WP_SPEED parameter not available on this vehicle.")

    original_wp_speed = float(vehicle.parameters["WP_SPEED"])
    vehicle.parameters["WP_SPEED"] = float(speed_mps)
    time.sleep(0.5)

    drive_msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        0,
        0,
        0xA3,
        [1.0, 0.0, 0.0, 0.0],
        0.0,
        0.0,
        0.0,
        1.0,
    )

    stop_msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        0,
        0,
        0xA3,
        [1.0, 0.0, 0.0, 0.0],
        0.0,
        0.0,
        0.0,
        0.0,
    )

    try:
        start_t = time.time()
        seq = 0

        while (time.time() - start_t) < duration_s:
            vehicle.send_mavlink(drive_msg)
            broadcast_status(vehicle, bridge, seq)
            seq += 1
            time.sleep(0.1)
    finally:
        vehicle.send_mavlink(stop_msg)
        time.sleep(0.5)
        vehicle.parameters["WP_SPEED"] = original_wp_speed
        time.sleep(0.5)


def drive_distance(vehicle, bridge, distance_m, speed_mps):
    ensure_ready(vehicle)
    moved = drive_distance_velocity(vehicle, bridge, distance_m, speed_mps)
    if not moved:
        print("[Ground] Velocity move not detected. Falling back to attitude drive.")
        drive_distance_attitude(vehicle, bridge, distance_m, speed_mps)


def turn_left(vehicle, bridge, angle_deg, yaw_rate_deg_s, tolerance_deg=5.0):
    ensure_ready(vehicle)

    if angle_deg <= 0:
        return

    start_heading = get_heading(vehicle)
    stop_target = abs(angle_deg) - STOP_EARLY_DEG

    turn_msg = build_attitude_msg(
        vehicle,
        throttle_fraction=0.0,
        yaw_rate_deg_s=-abs(yaw_rate_deg_s)
    )

    stable_count = 0
    seq = 0

    while True:
        vehicle.send_mavlink(turn_msg)
        broadcast_status(vehicle, bridge, seq)
        seq += 1

        time.sleep(HEADING_CHECK_INTERVAL_S)

        current_heading = get_heading(vehicle)
        delta = angle_diff_deg(current_heading, start_heading)

        if delta <= -(stop_target - tolerance_deg):
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= STABLE_COUNT_REQUIRED:
            break

    send_stop(vehicle)
    time.sleep(0.6)


def turn_right(vehicle, bridge, angle_deg, yaw_rate_deg_s, tolerance_deg=5.0):
    ensure_ready(vehicle)

    if angle_deg <= 0:
        return

    start_heading = get_heading(vehicle)
    stop_target = abs(angle_deg) - STOP_EARLY_DEG

    turn_msg = build_attitude_msg(
        vehicle,
        throttle_fraction=0.0,
        yaw_rate_deg_s=abs(yaw_rate_deg_s)
    )

    stable_count = 0
    seq = 0

    while True:
        vehicle.send_mavlink(turn_msg)
        broadcast_status(vehicle, bridge, seq)
        seq += 1

        time.sleep(HEADING_CHECK_INTERVAL_S)

        current_heading = get_heading(vehicle)
        delta = angle_diff_deg(current_heading, start_heading)

        if delta >= (stop_target - tolerance_deg):
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= STABLE_COUNT_REQUIRED:
            break

    send_stop(vehicle)
    time.sleep(0.6)


def parse_goto_message(msg_str):
    if not msg_str.startswith("GOTO:"):
        return None

    payload = msg_str.split(":", 1)[1].strip()
    x_str, y_str = payload.split(",")
    x_val = float(x_str.strip())
    y_val = float(y_str.strip())
    return x_val, y_val


def execute_body_goto(vehicle, bridge, x_m, y_m):
    ensure_ready(vehicle)

    # For ArUco navigation, the UAV sends GOTO:distance,0 after alignment.
    if abs(y_m) > GOTO_BODY_Y_EPS:
        print(f"[Ground] Ignoring non-body GOTO with y={y_m:.3f}")
        return

    distance_m = abs(x_m)
    if distance_m <= 0.0:
        send_stop(vehicle)
        return

    print(f"[Ground] >>> BODY GOTO FORWARD {distance_m:.3f} m")
    drive_distance(vehicle, bridge, distance_m, SPEED_MPS)
    send_stop(vehicle)
    time.sleep(0.5)


def main():
    bridge = None

    try:
        arm_ugv(vehicle)

        bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="UGV-Bridge")
        bridge.connect()

        ready_msg = "aruco ground station armed and awaiting commands"
        bridge.send_message(ready_msg)

        seq = 0

        while True:
            broadcast_status(vehicle, bridge, seq)
            seq += 1

            msg_str = bridge.get_message()
            if msg_str:
                print(f"[Ground] Incoming Shout: {msg_str}")

                coords = parse_goto_message(msg_str)
                if coords is not None:
                    x_m, y_m = coords
                    execute_body_goto(vehicle, bridge, x_m, y_m)

            cmd = bridge.get_command()
            if cmd:
                cmdSeq, cmdVal, eStop = cmd
                print(f"[Ground] Got Choice Order: {cmdVal}")

                if eStop == 1:
                    print("!!! [ABORT] EMERGENCY DISARM !!!")
                    send_stop(vehicle)
                    vehicle.armed = False
                    continue

                if cmdVal == v2v_bridge.CMD_ARM:
                    arm_ugv(vehicle)

                elif cmdVal == v2v_bridge.CMD_DISARM:
                    send_stop(vehicle)
                    vehicle.armed = False

                elif cmdVal == v2v_bridge.CMD_TURN_RIGHT:
                    print(f"[Ground] >>> TURN RIGHT STEP {TURN_STEP_DEG:.1f} deg")
                    turn_right(vehicle, bridge, TURN_STEP_DEG, TURN_RATE_DEG_S, TURN_TOLERANCE_DEG)

                elif cmdVal == v2v_bridge.CMD_TURN_LEFT:
                    print(f"[Ground] >>> TURN LEFT STEP {TURN_STEP_DEG:.1f} deg")
                    turn_left(vehicle, bridge, TURN_STEP_DEG, TURN_RATE_DEG_S, TURN_TOLERANCE_DEG)

                elif cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                    print(f"[Ground] >>> MOVE FORWARD {MOVE_FORWARD_10FT_M:.3f} m")
                    drive_distance(vehicle, bridge, MOVE_FORWARD_10FT_M, SPEED_MPS)
                    send_stop(vehicle)

                elif cmdVal == v2v_bridge.CMD_MOVE_2FT:
                    print(f"[Ground] >>> MOVE FORWARD {MOVE_FORWARD_2FT_M:.3f} m")
                    drive_distance(vehicle, bridge, MOVE_FORWARD_2FT_M, SPEED_MPS)
                    send_stop(vehicle)

                elif cmdVal == v2v_bridge.CMD_STOP:
                    print("[Ground] >>> STOPPED")
                    send_stop(vehicle)

            time.sleep(1.0 / TELEM_SEND_HZ)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            send_stop(vehicle)
        except Exception:
            pass

        try:
            if vehicle is not None and vehicle.armed:
                vehicle.armed = False
                wait_for_armed(vehicle, False)
        except Exception:
            pass

        if bridge is not None:
            try:
                bridge.stop()
            except Exception:
                pass

        if vehicle is not None:
            vehicle.close()


if __name__ == "__main__":
    main()
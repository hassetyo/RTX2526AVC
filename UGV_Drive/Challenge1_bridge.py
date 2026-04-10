from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math
import v2v_bridge

# ==========================================
# CHALLENGE 2 GROUND STATION
# Wait for UAV coordinates, then move there
# using Challenge 2 style motion:
#   1) drive straight
#   2) turn using compass/heading feedback
#   3) drive straight
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
SPEED_MPH = 0.8
TURN_ANGLE_DEG = 70.0
TURN_RATE_DEG_S = 10.0
TURN_TOLERANCE_DEG = 5.0
MOVEMENT_EPS_MPS = 0.05
TELEM_SEND_HZ = 5

# Slow-compass turn tuning
HEADING_CHECK_INTERVAL_S = 0.20
STOP_EARLY_DEG = 8.0
STABLE_COUNT_REQUIRED = 2

# ----------------------------
# Unit conversions
# ----------------------------
FT_TO_M = 0.3048
MPH_TO_MPS = 0.44704
SPEED_MPS = SPEED_MPH * MPH_TO_MPS


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
    if not vehicle.is_armable:
        print("Warning: vehicle reports not armable; attempting hybrid arm sequence anyway.")

    for label, state in (
        ("FIRST ARM", True),
        ("RESET DISARM", False),
        ("FINAL ARM", True),
    ):
        print(f"{label} in mode {vehicle.mode.name}...")
        vehicle.armed = state
        if not wait_for_armed(vehicle, state):
            raise RuntimeError(f"Failed to set armed={state}")
        time.sleep(1.0)

    print(f"Switching {vehicle.mode.name} -> GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    if not wait_for_mode(vehicle, "GUIDED"):
        raise RuntimeError(f"Failed to enter GUIDED mode, current mode: {vehicle.mode.name}")


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
    stop_msg = build_velocity_msg(vehicle, 0.0)
    vehicle.send_mavlink(stop_msg)
    time.sleep(0.5)


def get_groundspeed(vehicle):
    return vehicle.groundspeed if vehicle.groundspeed is not None else 0.0


def get_heading(vehicle):
    h = vehicle.heading
    if h is None:
        raise RuntimeError("vehicle.heading is unavailable")
    return float(h)


def angle_diff_deg(current_deg, start_deg):
    """
    Smallest signed angle from start_deg to current_deg, in degrees.
    Result is in [-180, 180].
    Positive = clockwise/right
    Negative = counterclockwise/left
    """
    return ((current_deg - start_deg + 540) % 360) - 180


def drive_distance_velocity(vehicle, bridge, distance_m, speed_mps, detection_window_s=1.5):
    if distance_m <= 0:
        return True

    duration_s = distance_m / speed_mps
    drive_msg = build_velocity_msg(vehicle, speed_mps)
    stop_msg = build_velocity_msg(vehicle, 0.0)

    print(f"Drive start (velocity target): distance={distance_m:.3f} m speed={speed_mps:.3f} m/s")
    start_t = time.time()
    last_print = 0.0
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

        if elapsed - last_print >= 1.0:
            print(
                f"  t={elapsed:4.1f}s armed={vehicle.armed} "
                f"mode={vehicle.mode.name} groundspeed={groundspeed:.3f} m/s"
            )
            last_print = elapsed

        if elapsed >= detection_window_s and not movement_detected:
            print("No meaningful movement detected from velocity target.")
            break

        time.sleep(0.1)

    vehicle.send_mavlink(stop_msg)
    time.sleep(0.5)
    return movement_detected


def drive_distance_attitude(vehicle, bridge, distance_m, speed_mps):
    if distance_m <= 0:
        return

    duration_s = distance_m / speed_mps
    original_wp_speed = None

    if "WP_SPEED" in vehicle.parameters:
        original_wp_speed = float(vehicle.parameters["WP_SPEED"])
        vehicle.parameters["WP_SPEED"] = float(speed_mps)
        time.sleep(0.5)
        print(f"WP_SPEED set to {speed_mps:.3f} m/s for attitude/throttle fallback.")
    else:
        raise RuntimeError("WP_SPEED parameter not available on this vehicle.")

    drive_msg = build_attitude_msg(vehicle, 1.0, 0.0)
    stop_msg = build_attitude_msg(vehicle, 0.0, 0.0)

    try:
        print(f"Drive start (attitude/throttle fallback): distance={distance_m:.3f} m")
        start_t = time.time()
        last_print = 0.0
        seq = 0

        while (time.time() - start_t) < duration_s:
            vehicle.send_mavlink(drive_msg)

            broadcast_status(vehicle, bridge, seq)
            seq += 1

            elapsed = time.time() - start_t

            if elapsed - last_print >= 1.0:
                groundspeed = get_groundspeed(vehicle)
                print(
                    f"  t={elapsed:4.1f}s armed={vehicle.armed} "
                    f"mode={vehicle.mode.name} groundspeed={groundspeed:.3f} m/s"
                )
                last_print = elapsed

            time.sleep(0.1)
    finally:
        vehicle.send_mavlink(stop_msg)
        time.sleep(0.5)
        if original_wp_speed is not None:
            vehicle.parameters["WP_SPEED"] = original_wp_speed
            time.sleep(0.5)
            print(f"WP_SPEED restored to {original_wp_speed:.3f} m/s.")


def drive_distance(vehicle, bridge, distance_m, speed_mps):
    moved = drive_distance_velocity(vehicle, bridge, distance_m, speed_mps)
    if moved:
        return

    print("Falling back to SET_ATTITUDE_TARGET for non-GPS forward motion.")
    drive_distance_attitude(vehicle, bridge, distance_m, speed_mps)


def turn_left(vehicle, bridge, angle_deg, yaw_rate_deg_s, tolerance_deg=5.0):
    if angle_deg <= 0:
        return

    start_heading = get_heading(vehicle)
    target_change = abs(angle_deg)
    stop_target = target_change - STOP_EARLY_DEG

    turn_msg = build_attitude_msg(
        vehicle,
        throttle_fraction=0.0,
        yaw_rate_deg_s=-abs(yaw_rate_deg_s)
    )

    last_print = 0.0
    stable_count = 0
    seq = 0

    print(
        f"TURN LEFT using slow heading updates: "
        f"start={start_heading:.1f} target=-{target_change:.1f} "
        f"stop_target=-{stop_target:.1f}"
    )

    while True:
        vehicle.send_mavlink(turn_msg)
        broadcast_status(vehicle, bridge, seq)
        seq += 1

        time.sleep(HEADING_CHECK_INTERVAL_S)

        current_heading = get_heading(vehicle)
        delta = angle_diff_deg(current_heading, start_heading)

        now = time.time()
        if now - last_print >= 0.2:
            print(f"  heading={current_heading:.1f} delta={delta:.1f}")
            last_print = now

        if delta <= -(stop_target - tolerance_deg):
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= STABLE_COUNT_REQUIRED:
            break

    send_stop(vehicle)
    time.sleep(0.6)

    final_heading = get_heading(vehicle)
    final_delta = angle_diff_deg(final_heading, start_heading)
    print(f"TURN LEFT done: final={final_heading:.1f} delta={final_delta:.1f}")


def turn_right(vehicle, bridge, angle_deg, yaw_rate_deg_s, tolerance_deg=5.0):
    if angle_deg <= 0:
        return

    start_heading = get_heading(vehicle)
    target_change = abs(angle_deg)
    stop_target = target_change - STOP_EARLY_DEG

    turn_msg = build_attitude_msg(
        vehicle,
        throttle_fraction=0.0,
        yaw_rate_deg_s=abs(yaw_rate_deg_s)
    )

    last_print = 0.0
    stable_count = 0
    seq = 0

    print(
        f"TURN RIGHT using slow heading updates: "
        f"start={start_heading:.1f} target=+{target_change:.1f} "
        f"stop_target=+{stop_target:.1f}"
    )

    while True:
        vehicle.send_mavlink(turn_msg)
        broadcast_status(vehicle, bridge, seq)
        seq += 1

        time.sleep(HEADING_CHECK_INTERVAL_S)

        current_heading = get_heading(vehicle)
        delta = angle_diff_deg(current_heading, start_heading)

        now = time.time()
        if now - last_print >= 0.2:
            print(f"  heading={current_heading:.1f} delta={delta:.1f}")
            last_print = now

        if delta >= (stop_target - tolerance_deg):
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= STABLE_COUNT_REQUIRED:
            break

    send_stop(vehicle)
    time.sleep(0.6)

    final_heading = get_heading(vehicle)
    final_delta = angle_diff_deg(final_heading, start_heading)
    print(f"TURN RIGHT done: final={final_heading:.1f} delta={final_delta:.1f}")


def execute_challenge2_move(vehicle, bridge, x_m, y_m):
    print("==========================================")
    print("Executing Challenge 1: Move to y")
    print(f"Received target from UAV: x={x_m:.3f} m, y={y_m:.3f} m")
    print("Convention: x = forward/back, y = right/left")
    print("Plan: drive |x|, turn toward y, drive |y|")
    print("==========================================")

    first_leg = abs(x_m)
    second_leg = abs(y_m)

    


    print(f"Leg 1: driving forward {first_leg:.3f} m")
    drive_distance(vehicle, bridge, first_leg, SPEED_MPS)
    time.sleep(4.0)

    
    

    print("Challenge 2 move complete.")


def parse_goto_message(msg_str):
    # Expected format: GOTO:x,y
    if not msg_str.startswith("GOTO:"):
        return None

    payload = msg_str.split(":", 1)[1].strip()
    x_str, y_str = payload.split(",")
    x_val = float(x_str.strip())
    y_val = float(y_str.strip())
    return x_val, y_val


def main():
    bridge = None

    print("==========================================")
    print("UGV Challenge 2 Ground Station")
    print("==========================================")
    print(f"Connecting to UGV at {UGV_CONTROL_PORT}...")
    print(f"Target speed: {SPEED_MPH:.1f} mph ({SPEED_MPS:.4f} m/s)")

    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)

    try:
        print(f"Initial state: armed={vehicle.armed} mode={vehicle.mode.name} armable={vehicle.is_armable}")

        arm_ugv(vehicle)
        print(f"Post-arm state: armed={vehicle.armed} mode={vehicle.mode.name}")

        print(f"Connecting to bridge at {ESP32_BRIDGE_PORT}...")
        bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="UGV-Bridge")
        bridge.connect()

        bridge.send_message("challenge 2 ground station armed and awaiting coordinates")

        seq = 0

        while True:
            broadcast_status(vehicle, bridge, seq)
            seq += 1

            msg_str = bridge.get_message()
            if msg_str:
                print(f"[Ground] Incoming message: {msg_str}")

                try:
                    coords = parse_goto_message(msg_str)
                    if coords is not None:
                        x_m, y_m = coords
                        execute_challenge2_move(vehicle, bridge, x_m, y_m)
                        bridge.send_message(f"challenge2_complete:{x_m:.2f},{y_m:.2f}")
                except Exception as e:
                    print(f"[Ground] Failed to process message: {e}")

            time.sleep(1.0 / TELEM_SEND_HZ)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down.")
    finally:
        try:
            if vehicle.armed:
                print("Disarming vehicle...")
                vehicle.armed = False
                wait_for_armed(vehicle, False)
        except Exception:
            pass

        if bridge is not None:
            try:
                bridge.stop()
            except Exception:
                pass

        vehicle.close()


if __name__ == "__main__":
    main()
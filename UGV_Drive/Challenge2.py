from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import argparse
#v3
UGV_CONTROL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

SPEED_MPH = 0.8

FT_TO_M = 0.3048
MPH_TO_MPS = 0.44704

SPEED_MPS = SPEED_MPH * MPH_TO_MPS
MOVEMENT_EPS_MPS = 0.05

# RC override tuning for turning
STEERING_NEUTRAL = 1500
STEERING_LEFT = 1200
STEERING_RIGHT = 1800

THROTTLE_NEUTRAL = 1500
THROTTLE_FORWARD = 1600

TURN_90_TIME_S = 2.2
TURN_SETTLE_S = 0.5


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


def arm_ugv(vehicle):
    if not vehicle.is_armable:
        print("Warning: vehicle reports not armable; attempting hybrid arm sequence anyway.")

    for label, state in (("FIRST ARM", True), ("RESET DISARM", False), ("FINAL ARM", True)):
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


def build_attitude_msg(vehicle, throttle_fraction):
    return vehicle.message_factory.set_attitude_target_encode(
        0,
        0,
        0,
        0xA3,
        [1.0, 0.0, 0.0, 0.0],
        0.0,
        0.0,
        0.0,
        throttle_fraction,
    )


def get_groundspeed(vehicle):
    return vehicle.groundspeed if vehicle.groundspeed is not None else 0.0


def drive_distance_velocity(vehicle, distance_m, speed_mps, detection_window_s=1.5):
    if distance_m <= 0:
        return True

    duration_s = distance_m / speed_mps
    drive_msg = build_velocity_msg(vehicle, speed_mps)
    stop_msg = build_velocity_msg(vehicle, 0.0)

    print(f"Drive start (velocity target): armed={vehicle.armed} mode={vehicle.mode.name}")
    start_t = time.time()
    last_print = 0.0
    movement_detected = False

    while (time.time() - start_t) < duration_s:
        vehicle.send_mavlink(drive_msg)
        elapsed = time.time() - start_t
        groundspeed = get_groundspeed(vehicle)

        if groundspeed >= MOVEMENT_EPS_MPS:
            movement_detected = True

        if elapsed - last_print >= 1.0:
            print(f"  t={elapsed:4.1f}s armed={vehicle.armed} mode={vehicle.mode.name} groundspeed={groundspeed:.3f} m/s")
            last_print = elapsed

        if elapsed >= detection_window_s and not movement_detected:
            print("No meaningful movement detected from velocity target.")
            break

        time.sleep(0.1)

    vehicle.send_mavlink(stop_msg)
    time.sleep(0.5)
    return movement_detected


def drive_distance_attitude(vehicle, distance_m, speed_mps):
    if distance_m <= 0:
        return

    duration_s = distance_m / speed_mps
    original_wp_speed = None

    if "WP_SPEED" in vehicle.parameters:
        original_wp_speed = float(vehicle.parameters["WP_SPEED"])
        vehicle.parameters["WP_SPEED"] = float(speed_mps)
        time.sleep(0.5)
        print(f"WP_SPEED set to {speed_mps:.3f} m/s for attitude/throttle test.")
    else:
        raise RuntimeError("WP_SPEED parameter not available on this vehicle.")

    drive_msg = build_attitude_msg(vehicle, 1.0)
    stop_msg = build_attitude_msg(vehicle, 0.0)

    try:
        print(f"Drive start (attitude/throttle fallback): armed={vehicle.armed} mode={vehicle.mode.name}")
        start_t = time.time()
        last_print = 0.0

        while (time.time() - start_t) < duration_s:
            vehicle.send_mavlink(drive_msg)
            elapsed = time.time() - start_t

            if elapsed - last_print >= 1.0:
                groundspeed = get_groundspeed(vehicle)
                print(f"  t={elapsed:4.1f}s armed={vehicle.armed} mode={vehicle.mode.name} groundspeed={groundspeed:.3f} m/s")
                last_print = elapsed

            time.sleep(0.1)

    finally:
        vehicle.send_mavlink(stop_msg)
        time.sleep(0.5)

        if original_wp_speed is not None:
            vehicle.parameters["WP_SPEED"] = original_wp_speed
            time.sleep(0.5)
            print(f"WP_SPEED restored to {original_wp_speed:.3f} m/s.")


def drive_distance(vehicle, distance_m, speed_mps):
    moved = drive_distance_velocity(vehicle, distance_m, speed_mps)
    if moved:
        return

    print("Falling back to SET_ATTITUDE_TARGET for non-GPS forward motion.")
    drive_distance_attitude(vehicle, distance_m, speed_mps)


def clear_rc_override(vehicle):
    vehicle.channels.overrides = {}
    time.sleep(0.2)


def turn_relative(vehicle, angle_deg, direction):
    """
    direction: 1 = right, -1 = left
    RC override turning, no lidar or obstacle detection.
    """
    if angle_deg <= 0:
        return

    dir_label = "right" if direction == 1 else "left"
    print(f"Turning {dir_label} {angle_deg:.0f} degrees...")

    turn_time = TURN_90_TIME_S * (angle_deg / 90.0)
    steering_value = STEERING_RIGHT if direction == 1 else STEERING_LEFT

    vehicle.channels.overrides = {
        "1": steering_value,
        "3": THROTTLE_FORWARD,
    }

    start_t = time.time()
    while (time.time() - start_t) < turn_time:
        time.sleep(0.1)

    clear_rc_override(vehicle)
    time.sleep(TURN_SETTLE_S)


def turn_to_heading(vehicle, current_heading, target_heading):
    order = ["N", "E", "S", "W"]
    cur_idx = order.index(current_heading)
    tgt_idx = order.index(target_heading)

    diff = (tgt_idx - cur_idx) % 4

    if diff == 0:
        return current_heading
    elif diff == 1:
        turn_relative(vehicle, 90, 1)
    elif diff == 2:
        turn_relative(vehicle, 180, 1)
    elif diff == 3:
        turn_relative(vehicle, 90, -1)

    return target_heading


def execute_path(vehicle, target_x_ft, target_y_ft):
    """
    Start at (0,0), facing +Y.
    Move along Y first, then along X.
    No lidar. No obstacle detection.
    """
    current_heading = "N"

    y_m = abs(target_y_ft) * FT_TO_M
    x_m = abs(target_x_ft) * FT_TO_M

    print("==========================================")
    print("   CHALLENGE 2: GRID MOVE")
    print("==========================================")
    print(f"Target coordinate: ({target_x_ft:.2f}, {target_y_ft:.2f}) ft")
    print(f"Speed:             {SPEED_MPH:.1f} mph ({SPEED_MPS:.4f} m/s)")
    print(f"Move Y first:      {target_y_ft:.2f} ft")
    print(f"Then move X:       {target_x_ft:.2f} ft")
    print("==========================================")

    if target_y_ft > 0:
        desired_heading = "N"
    elif target_y_ft < 0:
        desired_heading = "S"
    else:
        desired_heading = current_heading

    current_heading = turn_to_heading(vehicle, current_heading, desired_heading)

    if y_m > 0:
        print(f"Driving along Y for {abs(target_y_ft):.2f} ft...")
        drive_distance(vehicle, y_m, SPEED_MPS)
        print("Y movement complete.")
    else:
        print("No Y movement needed.")

    if target_x_ft > 0:
        desired_heading = "E"
    elif target_x_ft < 0:
        desired_heading = "W"
    else:
        desired_heading = current_heading

    current_heading = turn_to_heading(vehicle, current_heading, desired_heading)

    if x_m > 0:
        print(f"Driving along X for {abs(target_x_ft):.2f} ft...")
        drive_distance(vehicle, x_m, SPEED_MPS)
        print("X movement complete.")
    else:
        print("No X movement needed.")

    print("Path complete.")


def main():
    parser = argparse.ArgumentParser(description="Challenge2: move to (x,y) by going Y first, then X.")
    parser.add_argument("x", type=float, help="Target X in feet")
    parser.add_argument("y", type=float, help="Target Y in feet")
    args = parser.parse_args()

    print(f"Connecting to UGV at {UGV_CONTROL_PORT}...")
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=BAUD_RATE)

    try:
        print(f"Initial state: armed={vehicle.armed} mode={vehicle.mode.name} armable={vehicle.is_armable}")
        arm_ugv(vehicle)
        print("UGV armed in GUIDED mode. Starting Challenge 2 path...")
        execute_path(vehicle, args.x, args.y)
        print("Disarming...")
        vehicle.armed = False
        wait_for_armed(vehicle, False)
    finally:
        clear_rc_override(vehicle)
        vehicle.close()


if __name__ == "__main__":
    main()
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math

# Ports
UGV_CONTROL_PORT = "/dev/ttyACM0"
UGV_BAUD_RATE = 115200

# Mission parameters
SPEED_MPH = 5.0
INITIAL_DISTANCE_FT = 60.0
SECOND_DISTANCE_FT = 0.0
TURN_ANGLE_DEG = 75.0
TURN_RATE_DEG_S = 10.0
TURN_TOLERANCE_DEG = 5.0

# Slow-compass turn tuning
HEADING_CHECK_INTERVAL_S = 0.20
STOP_EARLY_DEG = 8.0
STABLE_COUNT_REQUIRED = 2

# Forward motion tuning
MOVEMENT_DETECTION_WINDOW_S = 5.0     # give velocity control more time before declaring failure
MOVEMENT_EPS_MPS = 0.05               # minimum reported groundspeed considered as "moving"
COMMAND_SEND_INTERVAL_S = 0.1         # how often to resend movement command
ATTITUDE_THROTTLE = 0.35              # tunable fallback throttle instead of fixed 1.0
POST_STOP_DELAY_S = 0.5

# Unit conversions
FT_TO_M = 0.3048
MPH_TO_MPS = 0.44704

# Derived constants
SPEED_MPS = SPEED_MPH * MPH_TO_MPS
INITIAL_DISTANCE_M = INITIAL_DISTANCE_FT * FT_TO_M
SECOND_DISTANCE_M = SECOND_DISTANCE_FT * FT_TO_M


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
    """
    Stop command for both velocity-based and attitude-based driving.
    Sends both a zero velocity command and a zero throttle command so
    whichever mode the rover was obeying gets a proper stop request.
    """
    stop_vel_msg = build_velocity_msg(vehicle, 0.0)
    stop_att_msg = build_attitude_msg(vehicle, 0.0, 0.0)

    vehicle.send_mavlink(stop_vel_msg)
    vehicle.send_mavlink(stop_att_msg)
    time.sleep(POST_STOP_DELAY_S)


def flash_red_tick(last_flash_time):
    # Placeholder so flashing=True does not crash.
    # Replace with actual GPIO LED code if needed.
    return time.time()


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


def turn_left(vehicle, angle_deg, yaw_rate_deg_s, flashing=False, tolerance_deg=5.0):
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
    last_flash_time = time.time()
    stable_count = 0

    print(
        f"TURN LEFT using slow heading updates: "
        f"start={start_heading:.1f} target=-{target_change:.1f} "
        f"stop_target=-{stop_target:.1f}"
    )

    while True:
        vehicle.send_mavlink(turn_msg)

        if flashing:
            last_flash_time = flash_red_tick(last_flash_time)

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


def turn_right(vehicle, angle_deg, yaw_rate_deg_s, flashing=False, tolerance_deg=5.0):
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
    last_flash_time = time.time()
    stable_count = 0

    print(
        f"TURN RIGHT using slow heading updates: "
        f"start={start_heading:.1f} target=+{target_change:.1f} "
        f"stop_target=+{stop_target:.1f}"
    )

    while True:
        vehicle.send_mavlink(turn_msg)

        if flashing:
            last_flash_time = flash_red_tick(last_flash_time)

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


def get_groundspeed(vehicle):
    return vehicle.groundspeed if vehicle.groundspeed is not None else 0.0


def drive_distance_velocity(vehicle, distance_m, speed_mps, detection_window_s=MOVEMENT_DETECTION_WINDOW_S):
    """
    Primary forward-drive method using SET_POSITION_TARGET_LOCAL_NED with
    body-frame forward velocity.
    Returns True if movement was detected, False otherwise.
    """
    if speed_mps <= 0:
        raise ValueError("speed_mps must be > 0")

    duration_s = distance_m / speed_mps
    drive_msg = build_velocity_msg(vehicle, speed_mps)

    print("--------------------------------------------------")
    print("Velocity-control drive")
    print(f"Requested distance : {distance_m:.3f} m")
    print(f"Requested speed    : {speed_mps:.3f} m/s")
    print(f"Planned duration   : {duration_s:.3f} s")
    print(f"Detection window   : {detection_window_s:.3f} s")
    print(f"Start state        : armed={vehicle.armed} mode={vehicle.mode.name}")
    print("--------------------------------------------------")

    start_t = time.time()
    last_print = -1.0
    movement_detected = False

    while (time.time() - start_t) < duration_s:
        vehicle.send_mavlink(drive_msg)

        elapsed = time.time() - start_t
        groundspeed = get_groundspeed(vehicle)

        if groundspeed >= MOVEMENT_EPS_MPS:
            movement_detected = True

        if elapsed - last_print >= 1.0:
            print(
                f"[velocity] t={elapsed:5.1f}s "
                f"groundspeed={groundspeed:.3f} m/s "
                f"moved={movement_detected}"
            )
            last_print = elapsed

        if elapsed >= detection_window_s and not movement_detected:
            print("[velocity] No meaningful movement detected during detection window.")
            print("[velocity] Exiting velocity mode and switching to fallback.")
            break

        time.sleep(COMMAND_SEND_INTERVAL_S)

    send_stop(vehicle)

    if movement_detected:
        print("[velocity] Movement detected successfully.")
    else:
        print("[velocity] Movement NOT detected.")

    return movement_detected


def drive_distance_attitude(vehicle, distance_m, speed_mps, throttle_fraction=ATTITUDE_THROTTLE):
    """
    Fallback forward-drive method using SET_ATTITUDE_TARGET.
    This path now uses a tunable throttle instead of fixed 1.0.
    """
    if speed_mps <= 0:
        raise ValueError("speed_mps must be > 0")
    if not (0.0 <= throttle_fraction <= 1.0):
        raise ValueError("throttle_fraction must be between 0.0 and 1.0")

    duration_s = distance_m / speed_mps
    original_wp_speed = None

    if "WP_SPEED" in vehicle.parameters:
        original_wp_speed = float(vehicle.parameters["WP_SPEED"])
        vehicle.parameters["WP_SPEED"] = float(speed_mps)
        time.sleep(0.5)
        print(f"[attitude] WP_SPEED set to {speed_mps:.3f} m/s")
    else:
        print("[attitude] WP_SPEED parameter not available; continuing without changing it.")

    drive_msg = build_attitude_msg(vehicle, throttle_fraction, 0.0)

    try:
        print("--------------------------------------------------")
        print("Attitude/throttle fallback drive")
        print(f"Requested distance : {distance_m:.3f} m")
        print(f"Reference speed    : {speed_mps:.3f} m/s")
        print(f"Planned duration   : {duration_s:.3f} s")
        print(f"Throttle fraction  : {throttle_fraction:.3f}")
        print(f"Start state        : armed={vehicle.armed} mode={vehicle.mode.name}")
        print("--------------------------------------------------")

        start_t = time.time()
        last_print = -1.0

        while (time.time() - start_t) < duration_s:
            vehicle.send_mavlink(drive_msg)
            elapsed = time.time() - start_t

            if elapsed - last_print >= 1.0:
                groundspeed = get_groundspeed(vehicle)
                print(
                    f"[attitude] t={elapsed:5.1f}s "
                    f"groundspeed={groundspeed:.3f} m/s "
                    f"throttle={throttle_fraction:.3f}"
                )
                last_print = elapsed

            time.sleep(COMMAND_SEND_INTERVAL_S)
    finally:
        send_stop(vehicle)

        if original_wp_speed is not None:
            vehicle.parameters["WP_SPEED"] = original_wp_speed
            time.sleep(0.5)
            print(f"[attitude] WP_SPEED restored to {original_wp_speed:.3f} m/s")


def drive_distance(vehicle, distance_m, speed_mps):
    """
    Try velocity control first. If no movement is detected, fall back to
    throttle-based driving.
    """
    moved = drive_distance_velocity(vehicle, distance_m, speed_mps)

    if moved:
        print("Drive completed using velocity control.")
        return

    print("Falling back to SET_ATTITUDE_TARGET for non-GPS forward motion.")
    drive_distance_attitude(vehicle, distance_m, speed_mps, ATTITUDE_THROTTLE)


def main():
    print("=============================================")
    print("UGV Challenge 1: Drive Forward a Set Distance")
    print("=============================================")
    print(f"Connecting to UGV at {UGV_CONTROL_PORT}...")
    print(f"Target speed: {SPEED_MPH:.1f} mph ({SPEED_MPS:.4f} m/s)")
    print(f"Target distance: {INITIAL_DISTANCE_FT:.1f} ft ({INITIAL_DISTANCE_M:.3f} m)")
    print(f"Fallback throttle: {ATTITUDE_THROTTLE:.3f}")
    print(f"Velocity detection window: {MOVEMENT_DETECTION_WINDOW_S:.1f} s")

    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)

    try:
        print(
            f"Initial state: armed={vehicle.armed} "
            f"mode={vehicle.mode.name} armable={vehicle.is_armable}"
        )

        arm_ugv(vehicle)

        print("UGV armed in GUIDED mode. Starting move...")
        drive_distance(vehicle, INITIAL_DISTANCE_M, SPEED_MPS)
        time.sleep(1.0)

        print("Move complete. Disarming...")
        vehicle.armed = False
        wait_for_armed(vehicle, False)
    finally:
        vehicle.close()
        print("Vehicle connection closed.")


if __name__ == "__main__":
    main()
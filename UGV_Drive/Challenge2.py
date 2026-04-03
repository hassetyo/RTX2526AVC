from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math

# Ports
UGV_CONTROL_PORT = "/dev/ttyACM0"
UGV_BAUD_RATE = 115200

# Mission parameters
SPEED_MPH = 0.8
INITIAL_DISTANCE_FT = 7
SECOND_DISTANCE_FT = 9.0
TURN_ANGLE_DEG = 65.0
TURN_RATE_DEG_S = 45.0

# Unit conversions
FT_TO_M = 0.3048
MPH_TO_MPS = 0.44704

# Derived constants
SPEED_MPS = SPEED_MPH * MPH_TO_MPS
MOVEMENT_EPS_MPS = 0.05
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
    stop_msg = build_velocity_msg(vehicle, 0.0)
    vehicle.send_mavlink(stop_msg)
    time.sleep(0.5)


def flash_red_tick(last_flash_time):
    # Placeholder so flashing=True does not crash.
    # Replace with actual GPIO LED code if needed.
    return time.time()


def turn_left(vehicle, angle_deg, yaw_rate_deg_s, flashing=False):
    duration_s = abs(angle_deg) / yaw_rate_deg_s
    turn_msg = build_attitude_msg(
        vehicle,
        throttle_fraction=0.0,
        yaw_rate_deg_s=-abs(yaw_rate_deg_s)
    )
    start_t = time.time()
    last_print = 0.0
    last_flash_time = time.time()

    print(f"TURN LEFT: angle={angle_deg:.1f} deg  rate={yaw_rate_deg_s:.1f} deg/s")
    while (time.time() - start_t) < duration_s:
        vehicle.send_mavlink(turn_msg)
        elapsed = time.time() - start_t
        if flashing:
            last_flash_time = flash_red_tick(last_flash_time)
        if elapsed - last_print >= 0.5:
            print(f"  turning... t={elapsed:3.1f}s")
            last_print = elapsed
        time.sleep(0.05)

    send_stop(vehicle)


def turn_right(vehicle, angle_deg, yaw_rate_deg_s, flashing=False):
    duration_s = abs(angle_deg) / yaw_rate_deg_s
    turn_msg = build_attitude_msg(
        vehicle,
        throttle_fraction=0.0,
        yaw_rate_deg_s=abs(yaw_rate_deg_s)
    )
    start_t = time.time()
    last_print = 0.0
    last_flash_time = time.time()

    print(f"TURN RIGHT: angle={angle_deg:.1f} deg  rate={yaw_rate_deg_s:.1f} deg/s")
    while (time.time() - start_t) < duration_s:
        vehicle.send_mavlink(turn_msg)
        elapsed = time.time() - start_t
        if flashing:
            last_flash_time = flash_red_tick(last_flash_time)
        if elapsed - last_print >= 0.5:
            print(f"  turning... t={elapsed:3.1f}s")
            last_print = elapsed
        time.sleep(0.05)

    send_stop(vehicle)


def get_groundspeed(vehicle):
    return vehicle.groundspeed if vehicle.groundspeed is not None else 0.0


def drive_distance_velocity(vehicle, distance_m, speed_mps, detection_window_s=1.5):
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


def drive_distance_attitude(vehicle, distance_m, speed_mps):
    duration_s = distance_m / speed_mps
    original_wp_speed = None

    if "WP_SPEED" in vehicle.parameters:
        original_wp_speed = float(vehicle.parameters["WP_SPEED"])
        vehicle.parameters["WP_SPEED"] = float(speed_mps)
        time.sleep(0.5)
        print(f"WP_SPEED set to {speed_mps:.3f} m/s for attitude/throttle test.")
    else:
        raise RuntimeError("WP_SPEED parameter not available on this vehicle.")

    drive_msg = build_attitude_msg(vehicle, 1.0, 0.0)
    stop_msg = build_attitude_msg(vehicle, 0.0, 0.0)

    try:
        print(f"Drive start (attitude/throttle fallback): armed={vehicle.armed} mode={vehicle.mode.name}")
        start_t = time.time()
        last_print = 0.0

        while (time.time() - start_t) < duration_s:
            vehicle.send_mavlink(drive_msg)
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


def drive_distance(vehicle, distance_m, speed_mps):
    moved = drive_distance_velocity(vehicle, distance_m, speed_mps)
    if moved:
        return

    print("Falling back to SET_ATTITUDE_TARGET for non-GPS forward motion.")
    drive_distance_attitude(vehicle, distance_m, speed_mps)


def main():
    print("==========================================")
    print("   UGV DIRECT DRIVE TEST")
    print("==========================================")
    print(f"Connecting to UGV at {UGV_CONTROL_PORT}...")
    print(f"Target speed: {SPEED_MPH:.1f} mph ({SPEED_MPS:.4f} m/s)")

    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)

    try:
        print(f"Initial state: armed={vehicle.armed} mode={vehicle.mode.name} armable={vehicle.is_armable}")
        arm_ugv(vehicle)

        print("UGV armed in GUIDED mode. Starting move...")
        drive_distance(vehicle, INITIAL_DISTANCE_M, SPEED_MPS)
        time.sleep(5.0)
        turn_left(vehicle, TURN_ANGLE_DEG, TURN_RATE_DEG_S, flashing=False)
        time.sleep(5.0)
        drive_distance(vehicle, SECOND_DISTANCE_M, SPEED_MPS)

        print("Move complete. Disarming...")
        vehicle.armed = False
        wait_for_armed(vehicle, False)
    finally:
        vehicle.close()


if __name__ == "__main__":
    main()
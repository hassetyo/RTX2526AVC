from dronekit import connect, VehicleMode
import time

# Ports
UGV_CONTROL_PORT = "/dev/ttyACM0"
UGV_BAUD_RATE = 115200

# Mission parameters
SPEED_MPH = 5.0
INITIAL_DISTANCE_FT = 60.0

# RC channel settings for rover
# Common ArduRover mapping:
#   Channel 1 = steering
#   Channel 3 = throttle
STEERING_CHANNEL = "1"
THROTTLE_CHANNEL = "3"

STEERING_NEUTRAL = 1500
THROTTLE_NEUTRAL = 1500

# Tune this upward/downward for your rover.
# 1600 = mild forward, 1700 = stronger, 1800 = aggressive
THROTTLE_PWM_FORWARD = 1700

# Unit conversions
FT_TO_M = 0.3048
MPH_TO_MPS = 0.44704

# Derived constants
SPEED_MPS = SPEED_MPH * MPH_TO_MPS
INITIAL_DISTANCE_M = INITIAL_DISTANCE_FT * FT_TO_M


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


def get_groundspeed(vehicle):
    return vehicle.groundspeed if vehicle.groundspeed is not None else 0.0


def print_speed_params(vehicle):
    print("Checking common rover speed parameters...")
    for p in ["WP_SPEED", "CRUISE_SPEED", "ATC_SPEED_MAX", "CRUISE_THROTTLE"]:
        try:
            if p in vehicle.parameters:
                print(f"  {p} = {vehicle.parameters[p]}")
        except Exception as e:
            print(f"  Could not read {p}: {e}")


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


def apply_rc_override(vehicle, steering_pwm=None, throttle_pwm=None):
    overrides = {}

    if steering_pwm is not None:
        overrides[STEERING_CHANNEL] = int(steering_pwm)

    if throttle_pwm is not None:
        overrides[THROTTLE_CHANNEL] = int(throttle_pwm)

    vehicle.channels.overrides = overrides


def stop_rc(vehicle):
    vehicle.channels.overrides = {
        STEERING_CHANNEL: STEERING_NEUTRAL,
        THROTTLE_CHANNEL: THROTTLE_NEUTRAL,
    }
    time.sleep(0.5)


def clear_rc_override(vehicle):
    vehicle.channels.overrides = {}
    time.sleep(0.2)


def drive_distance_rc(vehicle, distance_m, speed_mps, throttle_pwm):
    duration_s = distance_m / speed_mps

    print(f"Drive start (RC throttle): armed={vehicle.armed} mode={vehicle.mode.name}")
    print(f"Target duration: {duration_s:.2f} s | requested speed basis: {speed_mps:.3f} m/s")
    print(f"Using throttle PWM: {throttle_pwm}")

    start_t = time.time()
    last_print_sec = -1

    while (time.time() - start_t) < duration_s:
        apply_rc_override(
            vehicle,
            steering_pwm=STEERING_NEUTRAL,
            throttle_pwm=throttle_pwm
        )

        elapsed = time.time() - start_t
        groundspeed = get_groundspeed(vehicle)

        if int(elapsed) != last_print_sec:
            print(
                f"  t={elapsed:4.1f}s armed={vehicle.armed} "
                f"mode={vehicle.mode.name} groundspeed={groundspeed:.3f} m/s "
                f"throttle_pwm={throttle_pwm}"
            )
            last_print_sec = int(elapsed)

        time.sleep(0.1)

    print("Sending RC stop command...")
    stop_rc(vehicle)
    clear_rc_override(vehicle)


def drive_distance(vehicle, distance_m, speed_mps):
    drive_distance_rc(vehicle, distance_m, speed_mps, THROTTLE_PWM_FORWARD)


def main():
    print("=============================================")
    print("UGV Challenge 1: Drive Forward a Set Distance")
    print("=============================================")
    print(f"Connecting to UGV at {UGV_CONTROL_PORT}...")
    print(f"Target speed: {SPEED_MPH:.1f} mph ({SPEED_MPS:.4f} m/s)")
    print(f"Target distance: {INITIAL_DISTANCE_FT:.1f} ft ({INITIAL_DISTANCE_M:.3f} m)")
    print(f"Throttle PWM forward: {THROTTLE_PWM_FORWARD}")

    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)

    try:
        print(f"Initial state: armed={vehicle.armed} mode={vehicle.mode.name} armable={vehicle.is_armable}")
        print_speed_params(vehicle)

        arm_ugv(vehicle)

        print("UGV armed in GUIDED mode. Starting move...")
        drive_distance(vehicle, INITIAL_DISTANCE_M, SPEED_MPS)
        time.sleep(1.0)

        print("Move complete. Disarming...")
        vehicle.armed = False
        wait_for_armed(vehicle, False)
        print("UGV disarmed.")
    finally:
        try:
            stop_rc(vehicle)
            clear_rc_override(vehicle)
        except Exception:
            pass

        vehicle.close()
        print("Connection closed.")


if __name__ == "__main__":
    main()
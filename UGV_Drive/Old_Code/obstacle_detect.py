from dronekit import connect, VehicleMode
from gpiozero import DistanceSensor
from pymavlink import mavutil
import math
import time

try:
    from gpiozero.pins.lgpio import LGPIOFactory
except ImportError:
    LGPIOFactory = None

# This script moves the ugv forwards 5ft at 0.2 mph
# If an obstacle is detected within 1 ft, it executes a simple avoidance path: right 90 deg, forward 3 ft, left 90 deg, forward 3 ft, left 90 deg, forward 3 ft, right 90 deg, forward 3 ft
UGV_CONTROL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

ECHO_PIN = 10
TRIGGER_PIN = 9
SENSOR_MAX_DISTANCE_M = 5.0

INITIAL_DISTANCE_FT = 5.0
AVOIDANCE_DISTANCE_FT = 3.0
OBSTACLE_THRESHOLD_FT = 1.0
SPEED_MPH = 0.2
TURN_ANGLE_DEG = 90.0
TURN_RATE_DEG_S = 45.0

FT_TO_M = 0.3048
MPH_TO_MPS = 0.44704

INITIAL_DISTANCE_M = INITIAL_DISTANCE_FT * FT_TO_M
AVOIDANCE_DISTANCE_M = AVOIDANCE_DISTANCE_FT * FT_TO_M
OBSTACLE_THRESHOLD_M = OBSTACLE_THRESHOLD_FT * FT_TO_M
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


def get_groundspeed(vehicle):
    return vehicle.groundspeed if vehicle.groundspeed is not None else 0.0


def get_distance_m(sensor):
    return sensor.distance


def send_stop(vehicle, repeats=5):
    stop_msg = build_attitude_msg(vehicle, throttle_fraction=0.0, yaw_rate_deg_s=0.0)
    for _ in range(repeats):
        vehicle.send_mavlink(stop_msg)
        time.sleep(0.1)


def drive_forward(vehicle, sensor, distance_m, speed_mps, obstacle_threshold_m=None, label="DRIVE"):
    duration_s = distance_m / speed_mps
    original_wp_speed = None

    if "WP_SPEED" not in vehicle.parameters:
        raise RuntimeError("WP_SPEED parameter not available on this vehicle.")

    original_wp_speed = float(vehicle.parameters["WP_SPEED"])
    vehicle.parameters["WP_SPEED"] = float(speed_mps)
    time.sleep(0.5)

    drive_msg = build_attitude_msg(vehicle, throttle_fraction=1.0, yaw_rate_deg_s=0.0)
    start_t = time.time()
    last_print = 0.0

    try:
        print(f"{label}: target={distance_m:.3f} m speed={speed_mps:.4f} m/s")
        while (time.time() - start_t) < duration_s:
            vehicle.send_mavlink(drive_msg)
            elapsed = time.time() - start_t
            distance_sensor_m = get_distance_m(sensor)

            if elapsed - last_print >= 1.0:
                print(
                    f"  t={elapsed:4.1f}s groundspeed={get_groundspeed(vehicle):.3f} m/s "
                    f"sensor={distance_sensor_m:.3f} m"
                )
                last_print = elapsed

            if obstacle_threshold_m is not None and distance_sensor_m <= obstacle_threshold_m:
                print(f"Obstacle detected at {distance_sensor_m:.3f} m. Stopping forward motion.")
                send_stop(vehicle)
                return True

            time.sleep(0.1)

        send_stop(vehicle)
        return False
    finally:
        vehicle.parameters["WP_SPEED"] = original_wp_speed
        time.sleep(0.5)


def turn_right(vehicle, angle_deg, yaw_rate_deg_s):
    duration_s = abs(angle_deg) / yaw_rate_deg_s
    turn_msg = build_attitude_msg(vehicle, throttle_fraction=0.0, yaw_rate_deg_s=abs(yaw_rate_deg_s))
    start_t = time.time()
    last_print = 0.0

    print(f"TURN RIGHT: angle={angle_deg:.1f} deg rate={yaw_rate_deg_s:.1f} deg/s")
    while (time.time() - start_t) < duration_s:
        vehicle.send_mavlink(turn_msg)
        elapsed = time.time() - start_t
        if elapsed - last_print >= 0.5:
            print(f"  turning... t={elapsed:3.1f}s")
            last_print = elapsed
        time.sleep(0.1)

    send_stop(vehicle)


def turn_left(vehicle, angle_deg, yaw_rate_deg_s):
    duration_s = abs(angle_deg) / yaw_rate_deg_s
    turn_msg = build_attitude_msg(vehicle, throttle_fraction=0.0, yaw_rate_deg_s=-abs(yaw_rate_deg_s))
    start_t = time.time()
    last_print = 0.0

    print(f"TURN LEFT: angle={angle_deg:.1f} deg rate={yaw_rate_deg_s:.1f} deg/s")
    while (time.time() - start_t) < duration_s:
        vehicle.send_mavlink(turn_msg)
        elapsed = time.time() - start_t
        if elapsed - last_print >= 0.5:
            print(f"  turning... t={elapsed:3.1f}s")
            last_print = elapsed
        time.sleep(0.1)

    send_stop(vehicle)


def avoid_obstacle(vehicle, sensor):
    print("Obstacle detected. Executing avoidance path around the object.")

    turn_right(vehicle, TURN_ANGLE_DEG, TURN_RATE_DEG_S)
    drive_forward(vehicle, sensor, AVOIDANCE_DISTANCE_M, SPEED_MPS, obstacle_threshold_m=None, label="AVOID RIGHT")

    turn_left(vehicle, TURN_ANGLE_DEG, TURN_RATE_DEG_S)
    drive_forward(vehicle, sensor, AVOIDANCE_DISTANCE_M, SPEED_MPS, obstacle_threshold_m=None, label="BYPASS FORWARD")

    turn_left(vehicle, TURN_ANGLE_DEG, TURN_RATE_DEG_S)
    drive_forward(vehicle, sensor, AVOIDANCE_DISTANCE_M, SPEED_MPS, obstacle_threshold_m=None, label="RETURN LEFT")

    turn_right(vehicle, TURN_ANGLE_DEG, TURN_RATE_DEG_S)
    drive_forward(vehicle, sensor, AVOIDANCE_DISTANCE_M, SPEED_MPS, obstacle_threshold_m=None, label="REALIGN FORWARD")


def main():
    print("==========================================")
    print("   UGV OBSTACLE DETECT TEST")
    print("==========================================")
    print(f"Connecting to UGV at {UGV_CONTROL_PORT}...")
    print(f"Initial forward path: {INITIAL_DISTANCE_FT:.1f} ft at {SPEED_MPH:.1f} mph")
    print(f"Obstacle trigger: {OBSTACLE_THRESHOLD_FT:.1f} ft")
    print(
        "Avoidance path: right 90 + forward 3 ft, left 90 + forward 3 ft, "
        "left 90 + forward 3 ft, right 90 + forward 3 ft"
    )

    if LGPIOFactory is None:
        raise RuntimeError(
            "lgpio is not installed. Install it in this Python environment "
            "and rerun: sudo apt install python3-lgpio or pip install lgpio"
        )

    sensor = DistanceSensor(
        echo=ECHO_PIN,
        trigger=TRIGGER_PIN,
        max_distance=SENSOR_MAX_DISTANCE_M,
        pin_factory=LGPIOFactory(),
    )
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=BAUD_RATE)

    try:
        print(f"Initial state: armed={vehicle.armed} mode={vehicle.mode.name} armable={vehicle.is_armable}")
        print(f"Initial ultrasonic distance: {get_distance_m(sensor):.3f} m")
        arm_ugv(vehicle)

        obstacle_detected = drive_forward(
            vehicle,
            sensor,
            INITIAL_DISTANCE_M,
            SPEED_MPS,
            obstacle_threshold_m=OBSTACLE_THRESHOLD_M,
            label="INITIAL DRIVE",
        )

        if obstacle_detected:
            avoid_obstacle(vehicle, sensor)

        print("Mission complete. Disarming...")
        vehicle.armed = False
        wait_for_armed(vehicle, False)
    finally:
        send_stop(vehicle)
        sensor.close()
        vehicle.close()


if __name__ == "__main__":
    main()

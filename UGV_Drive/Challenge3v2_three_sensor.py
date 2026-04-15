from dronekit import connect, VehicleMode
from pymavlink import mavutil
from gpiozero import LED, DistanceSensor
import serial
import math
import time

try:
    from gpiozero.pins.lgpio import LGPIOFactory
    _pin_factory = LGPIOFactory()
except ImportError:
    _pin_factory = None

# Ports
UGV_CONTROL_PORT = "/dev/ttyACM0"
UGV_BAUD_RATE    = 115200

LIDAR_PORT      = "/dev/ttyAMA0"
LIDAR_BAUD_RATE = 115200

# GPIO pins
GREEN_LED_PIN = 16
RED_LED_PIN   = 19

RIGHT_TRIG = 5
RIGHT_ECHO = 6
LEFT_TRIG  = 23
LEFT_ECHO  = 22

# Mission Parameters
INITIAL_DISTANCE_FT        = 10.0
SECOND_DISTANCE_FT         = 8.0
AVOIDANCE_DISTANCE_FT      = 2.0    # preserved from your file
BYPASS_FORWARD_DISTANCE_FT = 3.0
SPEED_MPH                  = 0.8

# Obstacle thresholds / offsets from your 3-sensor test
ULTRASONIC_THRESHOLD_FT = 4.0
LIDAR_THRESHOLD_FT      = 4.0

LEFT_US_PRE_SHIFT_FT    = 1.0
CENTER_LIDAR_PRE_SHIFT_FT = 2.0
RIGHT_US_PRE_SHIFT_FT   = 2.5

# Compass-based turn tuning
TURN_ANGLE_DEG      = 90.0
TURN_RATE_DEG_S     = 10.0
TURN_TOLERANCE_DEG  = 5.0

# Slow-compass turn tuning
HEADING_CHECK_INTERVAL_S = 0.20
STOP_EARLY_DEG           = 8.0
STABLE_COUNT_REQUIRED    = 2

# Estimated forward progress gained from ONE completed avoidance maneuver.
# This is used to reduce the remaining mission distance after avoidance.
ESTIMATED_AVOIDANCE_FORWARD_PROGRESS_FT = BYPASS_FORWARD_DISTANCE_FT

# Safety / behavior limits
MAX_AVOIDANCE_DEPTH = 8

# Unit conversions
FT_TO_M    = 0.3048
MPH_TO_MPS = 0.44704

INITIAL_DISTANCE_M        = INITIAL_DISTANCE_FT * FT_TO_M
SECOND_DISTANCE_M         = SECOND_DISTANCE_FT * FT_TO_M
AVOIDANCE_DISTANCE_M      = AVOIDANCE_DISTANCE_FT * FT_TO_M
BYPASS_FORWARD_DISTANCE_M = BYPASS_FORWARD_DISTANCE_FT * FT_TO_M
ULTRASONIC_THRESHOLD_M    = ULTRASONIC_THRESHOLD_FT * FT_TO_M
LIDAR_THRESHOLD_M         = LIDAR_THRESHOLD_FT * FT_TO_M
LEFT_US_PRE_SHIFT_M       = LEFT_US_PRE_SHIFT_FT * FT_TO_M
CENTER_LIDAR_PRE_SHIFT_M  = CENTER_LIDAR_PRE_SHIFT_FT * FT_TO_M
RIGHT_US_PRE_SHIFT_M      = RIGHT_US_PRE_SHIFT_FT * FT_TO_M
SPEED_MPS                 = SPEED_MPH * MPH_TO_MPS
ESTIMATED_AVOIDANCE_FORWARD_PROGRESS_M = (
    ESTIMATED_AVOIDANCE_FORWARD_PROGRESS_FT * FT_TO_M
)

ULTRASONIC_MAX_DISTANCE_M = 4.0

# TF-Nova constants
FRAME_HEADER         = 0x59
LIDAR_MIN_CONFIDENCE = 10
LIDAR_NO_TARGET_M    = 9999.0

# LED flash timing
LED_FLASH_INTERVAL_S = 0.3


# --------------------------------------------------
# LED helpers
# --------------------------------------------------
def open_leds():
    kwargs = {"pin_factory": _pin_factory} if _pin_factory else {}
    green = LED(GREEN_LED_PIN, **kwargs)
    red   = LED(RED_LED_PIN, **kwargs)
    green.off()
    red.off()
    return green, red


def led_clear(green, red):
    red.off()
    green.on()


def led_obstacle(green, red):
    green.off()
    red.on()


def flash_red_tick(red, last_flash_time):
    now = time.time()
    if now - last_flash_time >= LED_FLASH_INTERVAL_S:
        red.toggle()
        return now
    return last_flash_time


# --------------------------------------------------
# Sensor helpers
# --------------------------------------------------
def open_ultrasonic(trig, echo, label="US"):
    kwargs = {"pin_factory": _pin_factory} if _pin_factory else {}
    sensor = DistanceSensor(
        echo=echo,
        trigger=trig,
        max_distance=ULTRASONIC_MAX_DISTANCE_M,
        **kwargs,
    )
    print(f"{label} ultrasonic opened  trig={trig}  echo={echo}")
    return sensor


def open_lidar(port=LIDAR_PORT, baud=LIDAR_BAUD_RATE):
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )
    ser.reset_input_buffer()
    print(f"TF-Nova lidar opened on {port}")
    return ser


def read_lidar_m(ser):
    ser.reset_input_buffer()

    for _ in range(18):
        b1 = ser.read(1)
        if not b1:
            return LIDAR_NO_TARGET_M
        if b1[0] != FRAME_HEADER:
            continue

        b2 = ser.read(1)
        if not b2:
            return LIDAR_NO_TARGET_M
        if b2[0] != FRAME_HEADER:
            continue

        payload = ser.read(7)
        if len(payload) < 7:
            return LIDAR_NO_TARGET_M

        dist_l, dist_h, peak_l, peak_h, temp, confidence, checksum = payload

        raw = [
            FRAME_HEADER, FRAME_HEADER,
            dist_l, dist_h, peak_l, peak_h, temp, confidence
        ]
        if (sum(raw) & 0xFF) != checksum:
            continue

        distance_cm = (dist_h << 8) | dist_l
        if confidence < LIDAR_MIN_CONFIDENCE or distance_cm == 0:
            return LIDAR_NO_TARGET_M

        return distance_cm / 100.0

    return LIDAR_NO_TARGET_M


def get_filtered_lidar_distance(lidar_ser, recent_readings, filter_window=3):
    raw_distance = read_lidar_m(lidar_ser)

    if raw_distance < LIDAR_NO_TARGET_M:
        recent_readings.append(raw_distance)
        if len(recent_readings) > filter_window:
            recent_readings.pop(0)

    return raw_distance, (min(recent_readings) if recent_readings else LIDAR_NO_TARGET_M)


def read_obstacle_state(us_left, us_right, lidar_ser, lidar_recent_readings, filter_window=3):
    left_m = us_left.distance
    right_m = us_right.distance
    raw_center_m, center_m = get_filtered_lidar_distance(
        lidar_ser, lidar_recent_readings, filter_window=filter_window
    )

    left_detected = left_m <= ULTRASONIC_THRESHOLD_M
    right_detected = right_m <= ULTRASONIC_THRESHOLD_M
    center_detected = center_m < LIDAR_NO_TARGET_M and center_m <= LIDAR_THRESHOLD_M

    detected_sources = []
    if left_detected:
        detected_sources.append(("left_ultrasonic", LEFT_US_PRE_SHIFT_M, left_m))
    if center_detected:
        detected_sources.append(("center_lidar", CENTER_LIDAR_PRE_SHIFT_M, center_m))
    if right_detected:
        detected_sources.append(("right_ultrasonic", RIGHT_US_PRE_SHIFT_M, right_m))

    trigger_source = None
    pre_shift_m = 0.0
    trigger_distance_m = None

    if detected_sources:
        # Use the largest requested left shift when more than one sensor sees the obstacle.
        trigger_source, pre_shift_m, trigger_distance_m = max(detected_sources, key=lambda x: x[1])

    return {
        "left_m": left_m,
        "right_m": right_m,
        "center_m": center_m,
        "raw_center_m": raw_center_m,
        "left_detected": left_detected,
        "right_detected": right_detected,
        "center_detected": center_detected,
        "obstacle_detected": bool(detected_sources),
        "trigger_source": trigger_source,
        "pre_shift_m": pre_shift_m,
        "trigger_distance_m": trigger_distance_m,
    }


def format_sensor_state(state):
    center_str = (
        f"{state['center_m'] / FT_TO_M:.2f} ft"
        if state["center_m"] < LIDAR_NO_TARGET_M else "no target"
    )
    return (
        f"left={state['left_m'] / FT_TO_M:.2f} ft ({'hit' if state['left_detected'] else 'clear'}), "
        f"center={center_str} ({'hit' if state['center_detected'] else 'clear'}), "
        f"right={state['right_m'] / FT_TO_M:.2f} ft ({'hit' if state['right_detected'] else 'clear'})"
    )


# --------------------------------------------------
# Vehicle helpers
# --------------------------------------------------
def wait_for_mode(vehicle, mode_name, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while vehicle.mode.name != mode_name and time.time() < deadline:
        time.sleep(0.1)
    return vehicle.mode.name == mode_name


def wait_for_armed(vehicle, armed_state, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while vehicle.armed != armed_state and time.time() < deadline:
        time.sleep(0.1)
    return vehicle.armed == armed_state


def arm_ugv(vehicle):
    print(
        f"Pre-arm state: mode={vehicle.mode.name}  "
        f"armed={vehicle.armed}  armable={vehicle.is_armable}"
    )

    print("Setting ARMING_CHECK=0 to bypass GPS pre-arm requirement...")
    vehicle.parameters["ARMING_CHECK"] = 0
    time.sleep(0.5)

    if vehicle.mode.name == "HOLD":
        print("Mode is HOLD: switching to MANUAL first...")
        vehicle.mode = VehicleMode("MANUAL")
        if not wait_for_mode(vehicle, "MANUAL"):
            raise RuntimeError("Could not leave HOLD mode.")
        time.sleep(0.5)

    print("Arming...")
    vehicle.armed = True
    if not wait_for_armed(vehicle, True, timeout_s=5.0):
        raise RuntimeError("Vehicle failed to arm.")
    print("Armed successfully.")
    time.sleep(0.5)

    print("Switching to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    if not wait_for_mode(vehicle, "GUIDED"):
        raise RuntimeError(
            f"Failed to enter GUIDED mode, current: {vehicle.mode.name}"
        )
    print(f"Ready: armed={vehicle.armed}  mode={vehicle.mode.name}")


def build_attitude_msg(vehicle, throttle_fraction, yaw_rate_deg_s=0.0):
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0,
        0xA3,
        [1.0, 0.0, 0.0, 0.0],
        0.0,
        0.0,
        math.radians(yaw_rate_deg_s),
        throttle_fraction,
    )


def get_groundspeed(vehicle):
    return vehicle.groundspeed if vehicle.groundspeed is not None else 0.0


def send_stop(vehicle, repeats=5):
    stop_msg = build_attitude_msg(vehicle, throttle_fraction=0.0)
    for _ in range(repeats):
        vehicle.send_mavlink(stop_msg)
        time.sleep(0.1)


def get_heading(vehicle):
    h = vehicle.heading
    if h is None:
        raise RuntimeError("vehicle.heading is unavailable")
    return float(h)


def angle_diff_deg(current_deg, start_deg):
    return ((current_deg - start_deg + 540) % 360) - 180


# --------------------------------------------------
# Motion primitives
# --------------------------------------------------
def drive_forward(vehicle, lidar_ser, us_left, us_right, distance_m, speed_mps,
                  green, red,
                  detect_obstacles=False,
                  flashing=False,
                  label="DRIVE"):
    """
    Drives forward for a target distance.

    Returns a dict:
    {
        "obstacle_hit": bool,
        "distance_completed_m": float,
        "obstacle_state": dict or None
    }
    """
    if distance_m <= 0.0:
        print(f"{label}: no forward motion needed (distance <= 0).")
        return {
            "obstacle_hit": False,
            "distance_completed_m": 0.0,
            "obstacle_state": None,
        }

    duration_s = distance_m / speed_mps

    if "WP_SPEED" not in vehicle.parameters:
        raise RuntimeError("WP_SPEED parameter not available.")

    original_wp_speed = float(vehicle.parameters["WP_SPEED"])
    vehicle.parameters["WP_SPEED"] = float(speed_mps)
    time.sleep(0.5)

    drive_msg       = build_attitude_msg(vehicle, throttle_fraction=1.0)
    start_t         = time.time()
    last_print      = 0.0
    last_flash_time = time.time()
    lidar_recent_readings = []

    try:
        print(f"{label}: target={distance_m:.3f} m  speed={speed_mps:.4f} m/s")
        while (time.time() - start_t) < duration_s:
            vehicle.send_mavlink(drive_msg)
            elapsed = time.time() - start_t

            obstacle_state = None
            if detect_obstacles:
                obstacle_state = read_obstacle_state(
                    us_left, us_right, lidar_ser, lidar_recent_readings, filter_window=3
                )

                if obstacle_state["obstacle_detected"]:
                    distance_completed_m = min(elapsed * speed_mps, distance_m)
                    print(
                        f"  *** Obstacle from {obstacle_state['trigger_source']} at "
                        f"{obstacle_state['trigger_distance_m']:.3f} m: STOPPING *** "
                        f"(completed {distance_completed_m:.3f} m)"
                    )
                    print(f"  Sensor state: {format_sensor_state(obstacle_state)}")
                    send_stop(vehicle)
                    return {
                        "obstacle_hit": True,
                        "distance_completed_m": distance_completed_m,
                        "obstacle_state": obstacle_state,
                    }
            else:
                raw_center_m, center_m = get_filtered_lidar_distance(
                    lidar_ser, lidar_recent_readings, filter_window=3
                )
                obstacle_state = {
                    "left_m": us_left.distance,
                    "right_m": us_right.distance,
                    "center_m": center_m,
                    "raw_center_m": raw_center_m,
                    "left_detected": False,
                    "right_detected": False,
                    "center_detected": False,
                    "obstacle_detected": False,
                    "trigger_source": None,
                    "pre_shift_m": 0.0,
                    "trigger_distance_m": None,
                }

            if flashing:
                last_flash_time = flash_red_tick(red, last_flash_time)

            if elapsed - last_print >= 0.5:
                print(
                    f"  t={elapsed:4.1f}s  groundspeed={get_groundspeed(vehicle):.3f} m/s  "
                    f"{format_sensor_state(obstacle_state)}"
                )
                last_print = elapsed

            time.sleep(0.05)

        send_stop(vehicle)
        return {
            "obstacle_hit": False,
            "distance_completed_m": distance_m,
            "obstacle_state": None,
        }

    finally:
        vehicle.parameters["WP_SPEED"] = original_wp_speed
        time.sleep(0.5)


def turn_right(vehicle, angle_deg, yaw_rate_deg_s, green, red, flashing=False, tolerance_deg=5.0):
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

    last_print      = 0.0
    last_flash_time = time.time()
    stable_count    = 0

    print(
        f"TURN RIGHT using slow heading updates: "
        f"start={start_heading:.1f} target=+{target_change:.1f} "
        f"stop_target=+{stop_target:.1f}"
    )

    while True:
        vehicle.send_mavlink(turn_msg)

        if flashing:
            last_flash_time = flash_red_tick(red, last_flash_time)

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


def turn_left(vehicle, angle_deg, yaw_rate_deg_s, green, red, flashing=False, tolerance_deg=5.0):
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

    last_print      = 0.0
    last_flash_time = time.time()
    stable_count    = 0

    print(
        f"TURN LEFT using slow heading updates: "
        f"start={start_heading:.1f} target=-{target_change:.1f} "
        f"stop_target=-{stop_target:.1f}"
    )

    while True:
        vehicle.send_mavlink(turn_msg)

        if flashing:
            last_flash_time = flash_red_tick(red, last_flash_time)

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


def strafe_left_by_turns(vehicle, lidar_ser, us_left, us_right, shift_m, green, red, flashing=True, label="SHIFT LEFT"):
    if shift_m <= 0.0:
        return
    print(f"{label}: moving left {shift_m / FT_TO_M:.2f} ft")
    turn_left(vehicle, 90.0, TURN_RATE_DEG_S, green, red, flashing=flashing, tolerance_deg=TURN_TOLERANCE_DEG)
    time.sleep(1.0)
    drive_forward(
        vehicle, lidar_ser, us_left, us_right,
        shift_m, SPEED_MPS,
        green, red,
        detect_obstacles=False,
        flashing=flashing,
        label=label,
    )
    time.sleep(1.0)
    turn_right(vehicle, 90.0, TURN_RATE_DEG_S, green, red, flashing=flashing, tolerance_deg=TURN_TOLERANCE_DEG)
    time.sleep(1.0)


def strafe_right_by_turns(vehicle, lidar_ser, us_left, us_right, shift_m, green, red, flashing=True, label="SHIFT RIGHT"):
    if shift_m <= 0.0:
        return
    print(f"{label}: moving right {shift_m / FT_TO_M:.2f} ft")
    turn_right(vehicle, 90.0, TURN_RATE_DEG_S, green, red, flashing=flashing, tolerance_deg=TURN_TOLERANCE_DEG)
    time.sleep(1.0)
    drive_forward(
        vehicle, lidar_ser, us_left, us_right,
        shift_m, SPEED_MPS,
        green, red,
        detect_obstacles=False,
        flashing=flashing,
        label=label,
    )
    time.sleep(1.0)
    turn_left(vehicle, 90.0, TURN_RATE_DEG_S, green, red, flashing=flashing, tolerance_deg=TURN_TOLERANCE_DEG)
    time.sleep(1.0)


# --------------------------------------------------
# Higher-level obstacle handling
# --------------------------------------------------
def avoid_obstacle(vehicle, lidar_ser, us_left, us_right, green, red,
                   obstacle_state, depth=0):
    """
    Obstacle policy requested:
    - left ultrasonic hit  -> move left 1 ft first
    - center lidar hit     -> move left 2 ft first
    - right ultrasonic hit -> move left 2.5 ft first

    After that, do the rest of the avoidance path.
    """
    if depth > MAX_AVOIDANCE_DEPTH:
        raise RuntimeError(
            "Maximum avoidance recursion depth reached. "
            "Environment may be too cluttered."
        )

    trigger_source = obstacle_state["trigger_source"]
    pre_shift_m = obstacle_state["pre_shift_m"]

    print(
        f"Executing avoidance path: source={trigger_source}, "
        f"pre-shift-left={pre_shift_m / FT_TO_M:.2f} ft, depth={depth}"
    )
    print(f"Avoidance sensor snapshot: {format_sensor_state(obstacle_state)}")

    green.off()
    red.on()

    # Requested custom left move before the rest of avoidance.
    strafe_left_by_turns(
        vehicle, lidar_ser, us_left, us_right,
        pre_shift_m,
        green, red,
        flashing=True,
        label="PRE-AVOID LEFT SHIFT",
    )

    # Continue forward past the obstacle while still watching all three sensors.
    forward_result = drive_forward(
        vehicle, lidar_ser, us_left, us_right,
        BYPASS_FORWARD_DISTANCE_M, SPEED_MPS,
        green, red,
        detect_obstacles=True,
        flashing=True,
        label="BYPASS FORWARD",
    )

    if forward_result["obstacle_hit"]:
        print("Obstacle encountered again during bypass forward. Recursing...")
        nested_progress = avoid_obstacle(
            vehicle, lidar_ser, us_left, us_right,
            green, red,
            forward_result["obstacle_state"],
            depth=depth + 1,
        )
        return forward_result["distance_completed_m"] + nested_progress

    # Return to the original mission line after the bypass.
    strafe_right_by_turns(
        vehicle, lidar_ser, us_left, us_right,
        pre_shift_m,
        green, red,
        flashing=True,
        label="RETURN RIGHT TO PATH",
    )

    print(
        "Avoidance complete. "
        f"Estimated mission-direction progress = {BYPASS_FORWARD_DISTANCE_M:.3f} m"
    )
    return BYPASS_FORWARD_DISTANCE_M


def execute_leg(vehicle, lidar_ser, us_left, us_right, green, red,
                leg_distance_m, label):
    remaining_m = leg_distance_m

    print(f"\n--- {label}: target remaining = {remaining_m:.3f} m ---")

    while remaining_m > 0.0:
        result = drive_forward(
            vehicle, lidar_ser, us_left, us_right,
            remaining_m, SPEED_MPS,
            green, red,
            detect_obstacles=True,
            flashing=False,
            label=label,
        )

        remaining_m -= result["distance_completed_m"]
        remaining_m = max(0.0, remaining_m)

        if not result["obstacle_hit"]:
            print(f"{label}: completed normally.")
            break

        led_obstacle(green, red)
        time.sleep(0.5)

        estimated_progress_m = avoid_obstacle(
            vehicle, lidar_ser, us_left, us_right,
            green, red,
            result["obstacle_state"],
            depth=0,
        )

        remaining_m -= estimated_progress_m
        remaining_m = max(0.0, remaining_m)

        led_clear(green, red)
        print(
            f"{label}: obstacle cleared, estimated avoidance progress "
            f"subtracted. Remaining distance = {remaining_m:.3f} m"
        )

    print(f"{label}: final remaining distance = {remaining_m:.3f} m")


# --------------------------------------------------
# Entry point
# --------------------------------------------------
def main():
    print("=================================================")
    print("UGV Challenge 3: 3-Sensor Obstacle Avoidance")
    print("=================================================")
    print(f"UGV port                         : {UGV_CONTROL_PORT}")
    print(f"Lidar port                       : {LIDAR_PORT}")
    print(f"Left ultrasonic pins             : trig={LEFT_TRIG}, echo={LEFT_ECHO}")
    print(f"Right ultrasonic pins            : trig={RIGHT_TRIG}, echo={RIGHT_ECHO}")
    print(f"Green LED GPIO                   : {GREEN_LED_PIN}")
    print(f"Red LED GPIO                     : {RED_LED_PIN}")
    print(f"Initial drive                    : {INITIAL_DISTANCE_FT:.1f} ft")
    print(f"Second drive                     : {SECOND_DISTANCE_FT:.1f} ft")
    print(f"Left/right ultrasonic threshold  : {ULTRASONIC_THRESHOLD_FT:.1f} ft")
    print(f"Center lidar threshold           : {LIDAR_THRESHOLD_FT:.1f} ft")
    print(f"Left ultrasonic pre-shift        : {LEFT_US_PRE_SHIFT_FT:.1f} ft")
    print(f"Center lidar pre-shift           : {CENTER_LIDAR_PRE_SHIFT_FT:.1f} ft")
    print(f"Right ultrasonic pre-shift       : {RIGHT_US_PRE_SHIFT_FT:.1f} ft")
    print(f"Avoidance bypass forward         : {BYPASS_FORWARD_DISTANCE_FT:.1f} ft")

    green, red = open_leds()
    lidar_ser  = open_lidar()
    us_left    = open_ultrasonic(LEFT_TRIG, LEFT_ECHO, label="LEFT")
    us_right   = open_ultrasonic(RIGHT_TRIG, RIGHT_ECHO, label="RIGHT")

    print("Warming up lidar (1 s)...")
    time.sleep(1.0)

    initial_state = read_obstacle_state(us_left, us_right, lidar_ser, [])
    print(f"Initial sensor state: {format_sensor_state(initial_state)}")

    print(f"Connecting to UGV at {UGV_CONTROL_PORT}...")
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)

    @vehicle.on_message('STATUSTEXT')
    def on_statustext(self, name, message):
        print(f"[FC] {message.text}")

    try:
        led_clear(green, red)
        arm_ugv(vehicle)

        execute_leg(
            vehicle, lidar_ser, us_left, us_right,
            green, red,
            INITIAL_DISTANCE_M,
            label="INITIAL DRIVE"
        )

        turn_left(
            vehicle, 90.0, TURN_RATE_DEG_S,
            green, red, flashing=True, tolerance_deg=TURN_TOLERANCE_DEG
        )
        time.sleep(4.0)

        execute_leg(
            vehicle, lidar_ser, us_left, us_right,
            green, red,
            SECOND_DISTANCE_M,
            label="SECOND DRIVE"
        )

        print("\nMission complete. Disarming...")
        vehicle.armed = False
        wait_for_armed(vehicle, False)

    finally:
        send_stop(vehicle)
        green.off()
        red.off()
        us_left.close()
        us_right.close()
        lidar_ser.close()
        vehicle.close()


if __name__ == "__main__":
    main()

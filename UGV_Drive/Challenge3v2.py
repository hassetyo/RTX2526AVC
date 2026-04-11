from dronekit import connect, VehicleMode
from pymavlink import mavutil
from gpiozero import LED
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

# LED GPIO Pins
GREEN_LED_PIN = 16
RED_LED_PIN   = 19

# Mission Parameters
INITIAL_DISTANCE_FT        = 10.0   # first leg
SECOND_DISTANCE_FT         = 8.0    # second leg
AVOIDANCE_DISTANCE_FT      = 2.0    # side-step leg length for avoidance path
BYPASS_FORWARD_DISTANCE_FT = 3.0    # forward leg while bypassing obstacle
OBSTACLE_THRESHOLD_FT      = 2.0    # lidar trigger threshold
SPEED_MPH                  = 0.8

# Compass-based turn tuning copied from Challenge2 style
TURN_ANGLE_DEG      = 90.0
TURN_RATE_DEG_S     = 10.0
TURN_TOLERANCE_DEG  = 5.0

# Slow-compass turn tuning
HEADING_CHECK_INTERVAL_S = 0.20
STOP_EARLY_DEG           = 8.0
STABLE_COUNT_REQUIRED    = 2

# Estimated forward progress gained along the original mission direction
# from ONE completed avoidance maneuver.
# Tune this after testing.
ESTIMATED_AVOIDANCE_FORWARD_PROGRESS_FT = 4.0

# Safety / behavior limits
MAX_AVOIDANCE_DEPTH = 8   # prevents infinite recursion if area is too cluttered

FT_TO_M    = 0.3048
MPH_TO_MPS = 0.44704

INITIAL_DISTANCE_M        = INITIAL_DISTANCE_FT * FT_TO_M
SECOND_DISTANCE_M         = SECOND_DISTANCE_FT * FT_TO_M
AVOIDANCE_DISTANCE_M      = AVOIDANCE_DISTANCE_FT * FT_TO_M
BYPASS_FORWARD_DISTANCE_M = BYPASS_FORWARD_DISTANCE_FT * FT_TO_M
OBSTACLE_THRESHOLD_M      = OBSTACLE_THRESHOLD_FT * FT_TO_M
SPEED_MPS                 = SPEED_MPH * MPH_TO_MPS
ESTIMATED_AVOIDANCE_FORWARD_PROGRESS_M = (
    ESTIMATED_AVOIDANCE_FORWARD_PROGRESS_FT * FT_TO_M
)

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
# TF-Nova lidar helpers
# --------------------------------------------------
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
    """
    Flush the input buffer so we always parse the newest frame,
    then scan for the two-byte header and decode one complete frame.
    """
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
    """
    Smallest signed angle from start_deg to current_deg, in degrees.
    Result is in [-180, 180].
    Positive = clockwise/right
    Negative = counterclockwise/left
    """
    return ((current_deg - start_deg + 540) % 360) - 180


# --------------------------------------------------
# Lidar filtering helper
# --------------------------------------------------
def get_filtered_lidar_distance(lidar_ser, recent_readings, filter_window=3):
    raw_distance = read_lidar_m(lidar_ser)

    if raw_distance < LIDAR_NO_TARGET_M:
        recent_readings.append(raw_distance)
        if len(recent_readings) > filter_window:
            recent_readings.pop(0)

    distance_now = min(recent_readings) if recent_readings else LIDAR_NO_TARGET_M
    return raw_distance, distance_now


# --------------------------------------------------
# Motion primitives
# --------------------------------------------------
def drive_forward(vehicle, lidar_ser, distance_m, speed_mps,
                  green, red,
                  obstacle_threshold_m=None,
                  flashing=False,
                  label="DRIVE"):
    """
    Drives forward for a target distance.

    Returns a dict:
    {
        "obstacle_hit": bool,
        "distance_completed_m": float
    }
    """
    if distance_m <= 0.0:
        print(f"{label}: no forward motion needed (distance <= 0).")
        return {
            "obstacle_hit": False,
            "distance_completed_m": 0.0
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

    filter_window   = 3
    recent_readings = []

    try:
        print(f"{label}: target={distance_m:.3f} m  speed={speed_mps:.4f} m/s")
        while (time.time() - start_t) < duration_s:
            vehicle.send_mavlink(drive_msg)
            elapsed = time.time() - start_t

            raw_distance, distance_now = get_filtered_lidar_distance(
                lidar_ser, recent_readings, filter_window=filter_window
            )

            if flashing:
                last_flash_time = flash_red_tick(red, last_flash_time)

            if elapsed - last_print >= 0.5:
                lidar_str = (
                    f"{distance_now:.3f} m (raw {raw_distance:.3f} m)"
                    if distance_now < LIDAR_NO_TARGET_M
                    else "no target"
                )
                print(
                    f"  t={elapsed:4.1f}s  "
                    f"groundspeed={get_groundspeed(vehicle):.3f} m/s  "
                    f"lidar={lidar_str}"
                )
                last_print = elapsed

            if (obstacle_threshold_m is not None
                    and distance_now < obstacle_threshold_m):
                distance_completed_m = min(elapsed * speed_mps, distance_m)
                print(
                    f"  *** Obstacle at {distance_now:.3f} m: STOPPING *** "
                    f"(completed {distance_completed_m:.3f} m)"
                )
                send_stop(vehicle)
                return {
                    "obstacle_hit": True,
                    "distance_completed_m": distance_completed_m
                }

            time.sleep(0.05)

        send_stop(vehicle)
        return {
            "obstacle_hit": False,
            "distance_completed_m": distance_m
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


# --------------------------------------------------
# Higher-level obstacle handling
# --------------------------------------------------
def avoid_obstacle(vehicle, lidar_ser, green, red, depth=0):
    """
    Perform one avoidance maneuver.

    Important:
    - Every forward leg during avoidance still watches lidar.
    - If another obstacle is found during a forward avoidance leg,
      avoidance is called again immediately.
    - Returns the estimated forward progress along the original mission path.
    """
    if depth > MAX_AVOIDANCE_DEPTH:
        raise RuntimeError(
            "Maximum avoidance recursion depth reached. "
            "Environment may be too cluttered."
        )

    print(f"Executing avoidance path: red LED flashing. depth={depth}")
    green.off()
    red.on()

    turn_left(vehicle, 90.0, TURN_RATE_DEG_S,
              green, red, flashing=True, tolerance_deg=TURN_TOLERANCE_DEG)
    time.sleep(4.0)

    drive_forward(vehicle, lidar_ser, AVOIDANCE_DISTANCE_M, SPEED_MPS,
                  green, red, obstacle_threshold_m=None,
                  flashing=True, label="AVOID RIGHT")
    time.sleep(4.0)

    turn_right(vehicle, 90.0, TURN_RATE_DEG_S,
               green, red, flashing=True, tolerance_deg=TURN_TOLERANCE_DEG)
    time.sleep(4.0)

    drive_forward(vehicle, lidar_ser, AVOIDANCE_DISTANCE_M + 1, SPEED_MPS,
                  green, red, obstacle_threshold_m=None,
                  flashing=True, label="BYPASS FORWARD")
    time.sleep(4.0)

    turn_right(vehicle, 90.0, TURN_RATE_DEG_S,
               green, red, flashing=True, tolerance_deg=TURN_TOLERANCE_DEG)
    time.sleep(4.0)

    drive_forward(vehicle, lidar_ser, AVOIDANCE_DISTANCE_M, SPEED_MPS,
                  green, red, obstacle_threshold_m=None,
                  flashing=True, label="RETURN LEFT")
    time.sleep(4.0)

    turn_left(vehicle, 90.0, TURN_RATE_DEG_S,
              green, red, flashing=True, tolerance_deg=TURN_TOLERANCE_DEG)
    time.sleep(4.0)

    print(
        "Avoidance complete. "
        f"Estimated mission-direction progress = "
        f"{ESTIMATED_AVOIDANCE_FORWARD_PROGRESS_M:.3f} m"
    )
    return ESTIMATED_AVOIDANCE_FORWARD_PROGRESS_M


def execute_leg(vehicle, lidar_ser, green, red, leg_distance_m, label):
    """
    Executes one straight mission leg.

    If an obstacle is hit:
    - subtract actual forward distance already driven before the stop
    - run avoidance
    - subtract estimated forward progress gained during avoidance
    - continue until leg_distance_m is fully completed
    """
    remaining_m = leg_distance_m

    print(f"\n--- {label}: target remaining = {remaining_m:.3f} m ---")

    while remaining_m > 0.0:
        result = drive_forward(
            vehicle, lidar_ser,
            remaining_m, SPEED_MPS,
            green, red,
            obstacle_threshold_m=OBSTACLE_THRESHOLD_M,
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

        estimated_progress_m = avoid_obstacle(vehicle, lidar_ser, green, red, depth + 1 if False else 0)

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
    print("UGV Challenge 3: Continuous Obstacle Avoidance")
    print("=================================================")
    print(f"UGV port                         : {UGV_CONTROL_PORT}")
    print(f"Lidar port                       : {LIDAR_PORT}")
    print(f"Green LED GPIO                   : {GREEN_LED_PIN}")
    print(f"Red LED GPIO                     : {RED_LED_PIN}")
    print(f"Initial drive                    : {INITIAL_DISTANCE_FT:.1f} ft")
    print(f"Second drive                     : {SECOND_DISTANCE_FT:.1f} ft")
    print(f"Obstacle trigger                 : {OBSTACLE_THRESHOLD_FT:.1f} ft ({OBSTACLE_THRESHOLD_M:.3f} m)")
    print(f"Estimated avoidance progress     : {ESTIMATED_AVOIDANCE_FORWARD_PROGRESS_FT:.1f} ft ({ESTIMATED_AVOIDANCE_FORWARD_PROGRESS_M:.3f} m)")
    print(f"Avoidance side-step              : {AVOIDANCE_DISTANCE_FT:.1f} ft ({AVOIDANCE_DISTANCE_M:.3f} m)")
    print(f"Avoidance bypass forward         : {BYPASS_FORWARD_DISTANCE_FT:.1f} ft ({BYPASS_FORWARD_DISTANCE_M:.3f} m)")

    green, red = open_leds()
    lidar_ser  = open_lidar()

    print("Warming up lidar (1 s)...")
    time.sleep(1.0)
    sample = read_lidar_m(lidar_ser)
    if sample < LIDAR_NO_TARGET_M:
        print(f"Initial lidar reading: {sample:.3f} m")
    else:
        print("Initial lidar reading: no target in range")

    print(f"Connecting to UGV at {UGV_CONTROL_PORT}...")
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)

    @vehicle.on_message('STATUSTEXT')
    def on_statustext(self, name, message):
        print(f"[FC] {message.text}")

    try:
        led_clear(green, red)
        arm_ugv(vehicle)

        execute_leg(
            vehicle, lidar_ser, green, red,
            INITIAL_DISTANCE_M,
            label="INITIAL DRIVE"
        )

        turn_left(
            vehicle, 90.0, TURN_RATE_DEG_S,
            green, red, flashing=True, tolerance_deg=TURN_TOLERANCE_DEG
        )
        time.sleep(4.0)

        execute_leg(
            vehicle, lidar_ser, green, red,
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
        lidar_ser.close()
        vehicle.close()


if __name__ == "__main__":
    main()
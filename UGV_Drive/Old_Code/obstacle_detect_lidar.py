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
INITIAL_DISTANCE_FT   = 10.0    # y coordinate
AVOIDANCE_DISTANCE_FT = 2.0     # how far to drive during each leg of the avoidance path
OBSTACLE_THRESHOLD_FT = 1.5     # if lidar detects an obstacle closer than this, trigger avoidance
SPEED_MPH             = 0.8
TURN_ANGLE_DEG        = 130.0
TURN_RATE_DEG_S       = 45.0

FT_TO_M    = 0.3048
MPH_TO_MPS = 0.44704

INITIAL_DISTANCE_M   = INITIAL_DISTANCE_FT   * FT_TO_M
AVOIDANCE_DISTANCE_M = AVOIDANCE_DISTANCE_FT * FT_TO_M
OBSTACLE_THRESHOLD_M = OBSTACLE_THRESHOLD_FT * FT_TO_M
SPEED_MPS            = SPEED_MPH * MPH_TO_MPS

# TF-Nova constants 
FRAME_HEADER         = 0x59
LIDAR_MIN_CONFIDENCE = 10
LIDAR_NO_TARGET_M    = 9999.0

# LED flash timing 
LED_FLASH_INTERVAL_S = 0.3


#  LED helpers
def open_leds():
    kwargs = {"pin_factory": _pin_factory} if _pin_factory else {}
    green = LED(GREEN_LED_PIN, **kwargs)
    red   = LED(RED_LED_PIN,   **kwargs)
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


#  TF-Nova lidar helpers
def open_lidar(port=LIDAR_PORT, baud=LIDAR_BAUD_RATE):
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,   # enough time for one full 9-byte frame at 115200
    )
    # Discard any stale bytes that accumulated during startup
    ser.reset_input_buffer()
    print(f"TF-Nova lidar opened on {port}")
    return ser


def read_lidar_m(ser):
    """
    Flush the input buffer so we always parse the NEWEST frame,
    then scan for the two-byte header and decode one complete frame.

    At 100 Hz the sensor produces a new frame every 10 ms.
    Flushing before each read means we always act on fresh data
    rather than a frame that was sitting in the OS buffer.
    """
    # Drop everything queued so far â€” we want the latest measurement
    ser.reset_input_buffer()

    # Now wait for one fresh frame (sensor outputs at 100 Hz so â‰¤10 ms)
    # Scan up to 18 bytes (2 full frames worth) looking for the header pair
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

        # Header found â€” read remaining 7 bytes
        payload = ser.read(7)
        if len(payload) < 7:
            return LIDAR_NO_TARGET_M

        dist_l, dist_h, peak_l, peak_h, temp, confidence, checksum = payload

        raw = [FRAME_HEADER, FRAME_HEADER,
               dist_l, dist_h, peak_l, peak_h, temp, confidence]
        if (sum(raw) & 0xFF) != checksum:
            # Checksum fail â€” try again from next byte
            continue

        distance_cm = (dist_h << 8) | dist_l
        if confidence < LIDAR_MIN_CONFIDENCE or distance_cm == 0:
            return LIDAR_NO_TARGET_M

        return distance_cm / 100.0

    return LIDAR_NO_TARGET_M


#  Vehicle helpers
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
    print(f"Pre-arm state: mode={vehicle.mode.name}  armed={vehicle.armed}  armable={vehicle.is_armable}")

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
        raise RuntimeError(f"Failed to enter GUIDED mode, current: {vehicle.mode.name}")
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


#  Motion primitives
def drive_forward(vehicle, lidar_ser, distance_m, speed_mps,
                  green, red,
                  obstacle_threshold_m=None,
                  flashing=False,
                  label="DRIVE"):

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

    # Running minimum filter â€” keeps the lowest recent reading
    # so a single noisy frame doesn't mask a real obstacle
    FILTER_WINDOW   = 3
    recent_readings = []

    try:
        print(f"{label}: target={distance_m:.3f} m  speed={speed_mps:.4f} m/s")
        while (time.time() - start_t) < duration_s:
            vehicle.send_mavlink(drive_msg)
            elapsed      = time.time() - start_t
            raw_distance = read_lidar_m(lidar_ser)

            # Build a rolling minimum over the last N valid readings
            if raw_distance < LIDAR_NO_TARGET_M:
                recent_readings.append(raw_distance)
                if len(recent_readings) > FILTER_WINDOW:
                    recent_readings.pop(0)

            distance_now = min(recent_readings) if recent_readings else LIDAR_NO_TARGET_M

            if flashing:
                last_flash_time = flash_red_tick(red, last_flash_time)

            if elapsed - last_print >= 0.5:   # print every 0.5s for faster feedback
                lidar_str = (
                    f"{distance_now:.3f} m (raw {raw_distance:.3f} m)"
                    if distance_now < LIDAR_NO_TARGET_M
                    else "no target"
                )
                print(
                    f"  t={elapsed:4.1f}s  groundspeed={get_groundspeed(vehicle):.3f} m/s"
                    f"  lidar={lidar_str}"
                )
                last_print = elapsed

            if (obstacle_threshold_m is not None
                    and distance_now < obstacle_threshold_m):
                print(f"  *** Obstacle at {distance_now:.3f} m: STOPPING ***")
                send_stop(vehicle)
                return True

            time.sleep(0.05)   # 20 Hz loop - tighter than before

        send_stop(vehicle)
        return False

    finally:
        vehicle.parameters["WP_SPEED"] = original_wp_speed
        time.sleep(0.5)


def turn_right(vehicle, angle_deg, yaw_rate_deg_s, green, red, flashing=False):
    duration_s      = abs(angle_deg) / yaw_rate_deg_s
    turn_msg        = build_attitude_msg(
        vehicle, throttle_fraction=0.0,
        yaw_rate_deg_s=abs(yaw_rate_deg_s)
    )
    start_t         = time.time()
    last_print      = 0.0
    last_flash_time = time.time()

    print(f"TURN RIGHT: angle={angle_deg:.1f} deg  rate={yaw_rate_deg_s:.1f} deg/s")
    while (time.time() - start_t) < duration_s:
        vehicle.send_mavlink(turn_msg)
        elapsed = time.time() - start_t
        if flashing:
            last_flash_time = flash_red_tick(red, last_flash_time)
        if elapsed - last_print >= 0.5:
            print(f"  turning... t={elapsed:3.1f}s")
            last_print = elapsed
        time.sleep(0.05)

    send_stop(vehicle)


def turn_left(vehicle, angle_deg, yaw_rate_deg_s, green, red, flashing=False):
    duration_s      = abs(angle_deg) / yaw_rate_deg_s
    turn_msg        = build_attitude_msg(
        vehicle, throttle_fraction=0.0,
        yaw_rate_deg_s=-abs(yaw_rate_deg_s)
    )
    start_t         = time.time()
    last_print      = 0.0
    last_flash_time = time.time()

    print(f"TURN LEFT: angle={angle_deg:.1f} deg  rate={yaw_rate_deg_s:.1f} deg/s")
    while (time.time() - start_t) < duration_s:
        vehicle.send_mavlink(turn_msg)
        elapsed = time.time() - start_t
        if flashing:
            last_flash_time = flash_red_tick(red, last_flash_time)
        if elapsed - last_print >= 0.5:
            print(f"  turning... t={elapsed:3.1f}s")
            last_print = elapsed
        time.sleep(0.05)

    send_stop(vehicle)


def avoid_obstacle(vehicle, lidar_ser, green, red):
    print("Executing avoidance path: red LED flashing.")
    green.off()
    red.on()

    turn_left(vehicle, TURN_ANGLE_DEG, TURN_RATE_DEG_S,
               green, red, flashing=True)
    time.sleep(4.0)

    drive_forward(vehicle, lidar_ser, AVOIDANCE_DISTANCE_M, SPEED_MPS,
                  green, red, obstacle_threshold_m=None,
                  flashing=True, label="AVOID RIGHT")
    time.sleep(4.0)

    turn_right(vehicle, TURN_ANGLE_DEG, TURN_RATE_DEG_S,
              green, red, flashing=True)
    time.sleep(4.0)

    drive_forward(vehicle, lidar_ser, AVOIDANCE_DISTANCE_M + 1, SPEED_MPS,
                  green, red, obstacle_threshold_m=None,
                  flashing=True, label="BYPASS FORWARD")
    time.sleep(4.0)

    turn_right(vehicle, TURN_ANGLE_DEG, TURN_RATE_DEG_S,
              green, red, flashing=True)
    time.sleep(4.0)

    drive_forward(vehicle, lidar_ser, AVOIDANCE_DISTANCE_M, SPEED_MPS,
                  green, red, obstacle_threshold_m=None,
                  flashing=True, label="RETURN LEFT")
    time.sleep(4.0)

    turn_left(vehicle, TURN_ANGLE_DEG, TURN_RATE_DEG_S,
               green, red, flashing=True)
    time.sleep(4.0)

    drive_forward(vehicle, lidar_ser, AVOIDANCE_DISTANCE_M, SPEED_MPS,
                  green, red, obstacle_threshold_m=None,
                  flashing=True, label="REALIGN FORWARD")
    time.sleep(4.0)


#  Entry point
def main():
    print("==========================================")
    print("   UGV OBSTACLE DETECT TEST  (TF-Nova)")
    print("==========================================")
    print(f"UGV port        : {UGV_CONTROL_PORT}")
    print(f"Lidar port      : {LIDAR_PORT}")
    print(f"Green LED GPIO  : {GREEN_LED_PIN}")
    print(f"Red LED GPIO    : {RED_LED_PIN}")
    print(f"Initial drive   : {INITIAL_DISTANCE_FT:.1f} ft at {SPEED_MPH:.1f} mph")
    print(f"Obstacle trigger: {OBSTACLE_THRESHOLD_FT:.1f} ft ({OBSTACLE_THRESHOLD_M:.3f} m)")

    green, red = open_leds()
    lidar_ser  = open_lidar()

    # Warm up the lidar â€” let the buffer fill with fresh frames
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

        obstacle_hit = drive_forward(
            vehicle, lidar_ser,
            INITIAL_DISTANCE_M, SPEED_MPS,
            green, red,
            obstacle_threshold_m=OBSTACLE_THRESHOLD_M,
            flashing=False,
            label="INITIAL DRIVE",
        )

        if obstacle_hit:
            led_obstacle(green, red)
            time.sleep(0.5)
            avoid_obstacle(vehicle, lidar_ser, green, red)
            led_clear(green, red)
            print("Obstacle cleared: green LED on.")

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

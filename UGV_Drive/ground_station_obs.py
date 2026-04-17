from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math
import serial
from gpiozero import DistanceSensor
import v2v_bridge

try:
    from gpiozero.pins.lgpio import LGPIOFactory
    _pin_factory = LGPIOFactory()
except ImportError:
    _pin_factory = None


# ==========================================
# UGV GROUND STATION + OBSTACLE AVOIDANCE
# ==========================================
# Behavior:
#   1) arm and enter GUIDED at startup
#   2) receive turn / move commands from UAV bridge
#   3) continuously monitor 3 obstacle sensors
#   4) if obstacle detected:
#         - stop normal command handling
#         - discard all incoming bridge traffic during avoidance
#         - go around obstacle to the LEFT
#         - drain stale bridge traffic
#         - resume listening only to new UAV commands
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
MPH_TO_MPS = 0.44704
SPEED_MPS = SPEED_MPH * MPH_TO_MPS

TURN_STEP_DEG = 12.0
TURN_RATE_DEG_S = 10.0
TURN_TOLERANCE_DEG = 5.0

MOVE_FORWARD_10FT_M = 3.048
MOVE_FORWARD_2FT_M = 0.61

GOTO_BODY_Y_EPS = 1e-3
MOVEMENT_EPS_MPS = 0.05

TELEM_SEND_HZ = 5
HEADING_CHECK_INTERVAL_S = 0.20
STOP_EARLY_DEG = 8.0
STABLE_COUNT_REQUIRED = 2

# Velocity move behavior
VELOCITY_DETECTION_WINDOW_S = 2.0
COMMAND_SEND_INTERVAL_S = 0.1

# ----------------------------
# Obstacle sensor pins / serial
# ----------------------------
RIGHT_TRIG = 5
RIGHT_ECHO = 6
LEFT_TRIG = 23
LEFT_ECHO = 22

LIDAR_PORT = "/dev/ttyAMA0"
LIDAR_BAUD_RATE = 115200

# ----------------------------
# Obstacle thresholds
# ----------------------------
FT_TO_M = 0.3048
ULTRASONIC_THRESHOLD_FT = 1.9
LIDAR_THRESHOLD_FT = 2.1

ULTRASONIC_THRESHOLD_M = ULTRASONIC_THRESHOLD_FT * FT_TO_M
LIDAR_THRESHOLD_M = LIDAR_THRESHOLD_FT * FT_TO_M
ULTRASONIC_MAX_DISTANCE_M = 4.0

# TF-Nova constants
FRAME_HEADER = 0x59
LIDAR_MIN_CONFIDENCE = 10
LIDAR_NO_TARGET_M = 9999.0

# ----------------------------
# Avoidance path tuning
# ----------------------------
# "Go around obstacle to the LEFT"
AVOID_LEFT_TURN_DEG = 90.0
AVOID_RIGHT_TURN_DEG = 90.0

# Step 1: move left around the obstacle
AVOID_LEFT_BYPASS_M = 1.0

# Step 2: move forward enough to get past obstacle
AVOID_FORWARD_CLEAR_M = 1.6

# Step 3: move back right to original path line
AVOID_RETURN_RIGHT_M = 1.0

# After finishing avoidance, keep draining bridge briefly
POST_AVOID_DRAIN_S = 0.75

# How many empty drain cycles before we consider buffers clear
DRAIN_EMPTY_CYCLES_REQUIRED = 5


print("==========================================")
print(" UGV GROUND STATION + OBSTACLE AVOIDANCE ")
print("==========================================")
print(f"[Ground] Connecting to UGV at {UGV_CONTROL_PORT}...")


# ----------------------------
# Vehicle connect
# ----------------------------
vehicle = None
try:
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)
    print("[Ground] Connected to UGV.")
except Exception as e:
    print(f"!!! Error connecting to UGV: {e} !!!")
    raise SystemExit(1)


# ----------------------------
# Sensor helpers
# ----------------------------
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
    print(f"[Sensors] TF-Nova lidar opened on {port}")
    return ser


def read_lidar_m(ser):
    """
    Read one fresh frame from the TF-Nova and return distance in metres.
    Returns LIDAR_NO_TARGET_M if no valid target.
    """
    ser.reset_input_buffer()

    for _ in range(18):
        b1 = ser.read(1)
        if not b1 or b1[0] != FRAME_HEADER:
            continue

        b2 = ser.read(1)
        if not b2 or b2[0] != FRAME_HEADER:
            continue

        payload = ser.read(7)
        if len(payload) < 7:
            return LIDAR_NO_TARGET_M

        dist_l, dist_h, peak_l, peak_h, temp, confidence, checksum = payload

        raw = [FRAME_HEADER, FRAME_HEADER,
               dist_l, dist_h, peak_l, peak_h, temp, confidence]

        if (sum(raw) & 0xFF) != checksum:
            continue

        distance_cm = (dist_h << 8) | dist_l
        if confidence < LIDAR_MIN_CONFIDENCE or distance_cm == 0:
            return LIDAR_NO_TARGET_M

        return distance_cm / 100.0

    return LIDAR_NO_TARGET_M


def open_ultrasonic(trig, echo, label="US"):
    kwargs = {"pin_factory": _pin_factory} if _pin_factory else {}
    sensor = DistanceSensor(
        echo=echo,
        trigger=trig,
        max_distance=ULTRASONIC_MAX_DISTANCE_M,
        **kwargs,
    )
    print(f"[Sensors] {label} ultrasonic opened trig={trig} echo={echo}")
    return sensor


# ----------------------------
# General vehicle helpers
# ----------------------------
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
    print("\n[Ground] >>> ARMING UGV")

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
    vehicle.send_mavlink(build_attitude_msg(vehicle, 0.0, 0.0))
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


# ----------------------------
# Bridge / telemetry helpers
# ----------------------------
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


def discard_bridge_inputs(bridge, duration_s=0.0):
    """
    Read and discard ALL currently incoming bridge traffic.
    This prevents commands received during avoidance from surviving
    and being executed later.
    """
    deadline = time.time() + max(0.0, duration_s)
    emptied_cycles = 0

    while True:
        saw_any = False

        while True:
            msg = bridge.get_message()
            if not msg:
                break
            saw_any = True
            print(f"[Bridge] Discarded message during pause: {msg}")

        while True:
            cmd = bridge.get_command()
            if not cmd:
                break
            saw_any = True
            print(f"[Bridge] Discarded command during pause: {cmd}")

        if saw_any:
            emptied_cycles = 0
        else:
            emptied_cycles += 1

        if time.time() >= deadline and emptied_cycles >= DRAIN_EMPTY_CYCLES_REQUIRED:
            break

        time.sleep(0.02)


# ----------------------------
# Obstacle helpers
# ----------------------------
def read_all_obstacle_sensors(us_left, lidar_ser, us_right):
    left_m = us_left.distance
    right_m = us_right.distance
    center_m = read_lidar_m(lidar_ser)

    left_detected = left_m <= ULTRASONIC_THRESHOLD_M
    right_detected = right_m <= ULTRASONIC_THRESHOLD_M
    center_detected = (center_m < LIDAR_NO_TARGET_M and center_m <= LIDAR_THRESHOLD_M)

    obstacle_detected = left_detected or center_detected or right_detected

    return {
        "left_m": left_m,
        "center_m": center_m,
        "right_m": right_m,
        "left_detected": left_detected,
        "center_detected": center_detected,
        "right_detected": right_detected,
        "obstacle_detected": obstacle_detected,
    }


def obstacle_detected_now(us_left, lidar_ser, us_right):
    return read_all_obstacle_sensors(us_left, lidar_ser, us_right)["obstacle_detected"]


# ----------------------------
# Movement with obstacle monitoring
# ----------------------------
def drive_distance_velocity(vehicle, bridge, us_left, lidar_ser, us_right,
                            distance_m, speed_mps, detection_window_s=VELOCITY_DETECTION_WINDOW_S):
    if distance_m <= 0:
        send_stop(vehicle)
        return True, False  # moved, obstacle_triggered

    duration_s = distance_m / speed_mps
    drive_msg = build_velocity_msg(vehicle, speed_mps)
    stop_msg = build_stop_msg(vehicle)

    start_t = time.time()
    movement_detected = False
    seq = 0

    while (time.time() - start_t) < duration_s:
        # Obstacle check first
        if obstacle_detected_now(us_left, lidar_ser, us_right):
            vehicle.send_mavlink(stop_msg)
            time.sleep(0.2)
            return movement_detected, True

        vehicle.send_mavlink(drive_msg)
        broadcast_status(vehicle, bridge, seq)
        seq += 1

        elapsed = time.time() - start_t
        groundspeed = get_groundspeed(vehicle)

        if groundspeed >= MOVEMENT_EPS_MPS:
            movement_detected = True

        if elapsed >= detection_window_s and not movement_detected:
            break

        time.sleep(COMMAND_SEND_INTERVAL_S)

    vehicle.send_mavlink(stop_msg)
    time.sleep(0.5)
    return movement_detected, False


def drive_distance_attitude(vehicle, bridge, us_left, lidar_ser, us_right,
                            distance_m, speed_mps):
    if distance_m <= 0:
        send_stop(vehicle)
        return False  # obstacle_triggered

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
            if obstacle_detected_now(us_left, lidar_ser, us_right):
                vehicle.send_mavlink(stop_msg)
                time.sleep(0.2)
                return True

            vehicle.send_mavlink(drive_msg)
            broadcast_status(vehicle, bridge, seq)
            seq += 1
            time.sleep(COMMAND_SEND_INTERVAL_S)

    finally:
        vehicle.send_mavlink(stop_msg)
        time.sleep(0.5)
        vehicle.parameters["WP_SPEED"] = original_wp_speed
        time.sleep(0.5)

    return False


def drive_distance(vehicle, bridge, us_left, lidar_ser, us_right, distance_m, speed_mps):
    ensure_ready(vehicle)

    moved, obstacle_triggered = drive_distance_velocity(
        vehicle, bridge, us_left, lidar_ser, us_right, distance_m, speed_mps
    )

    if obstacle_triggered:
        return True

    if not moved:
        print("[Ground] Velocity move not detected. Falling back to attitude drive.")
        obstacle_triggered = drive_distance_attitude(
            vehicle, bridge, us_left, lidar_ser, us_right, distance_m, speed_mps
        )
        if obstacle_triggered:
            return True

    return False


def turn_left(vehicle, bridge, us_left, lidar_ser, us_right,
              angle_deg, yaw_rate_deg_s, tolerance_deg=5.0, check_obstacles=True):
    ensure_ready(vehicle)

    if angle_deg <= 0:
        return False  # obstacle_triggered

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
        if check_obstacles and obstacle_detected_now(us_left, lidar_ser, us_right):
            send_stop(vehicle)
            return True

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
    return False


def turn_right(vehicle, bridge, us_left, lidar_ser, us_right,
               angle_deg, yaw_rate_deg_s, tolerance_deg=5.0, check_obstacles=True):
    ensure_ready(vehicle)

    if angle_deg <= 0:
        return False  # obstacle_triggered

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
        if check_obstacles and obstacle_detected_now(us_left, lidar_ser, us_right):
            send_stop(vehicle)
            return True

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
    return False


# ----------------------------
# Body GOTO support
# ----------------------------
def parse_goto_message(msg_str):
    if not msg_str.startswith("GOTO:"):
        return None

    payload = msg_str.split(":", 1)[1].strip()
    x_str, y_str = payload.split(",")
    x_val = float(x_str.strip())
    y_val = float(y_str.strip())
    return x_val, y_val


def execute_body_goto(vehicle, bridge, us_left, lidar_ser, us_right, x_m, y_m):
    ensure_ready(vehicle)

    if abs(y_m) > GOTO_BODY_Y_EPS:
        print(f"[Ground] Ignoring non-body GOTO with y={y_m:.3f}")
        return False  # obstacle_triggered

    distance_m = abs(x_m)
    if distance_m <= 0.0:
        send_stop(vehicle)
        return False

    print(f"[Ground] >>> BODY GOTO FORWARD {distance_m:.3f} m")
    obstacle_triggered = drive_distance(
        vehicle, bridge, us_left, lidar_ser, us_right, distance_m, SPEED_MPS
    )
    send_stop(vehicle)
    time.sleep(0.5)
    return obstacle_triggered


# ----------------------------
# Obstacle avoidance routine
# ----------------------------
def avoid_obstacle_left_and_resume(vehicle, bridge, us_left, lidar_ser, us_right):
    """
    Pause normal UAV command execution, discard all incoming bridge traffic,
    go around obstacle to the LEFT, then drain stale inputs again and resume.
    """
    print("\n[Avoid] ==================================================")
    print("[Avoid] OBSTACLE DETECTED - pausing UAV command handling")
    print("[Avoid] Incoming bridge commands/messages will be discarded")
    print("[Avoid] Executing LEFT-side bypass")
    print("[Avoid] ==================================================\n")

    ensure_ready(vehicle)
    send_stop(vehicle)

    # Immediately discard anything already arriving
    discard_bridge_inputs(bridge, duration_s=0.20)

    # Step A: turn left 90
    print(f"[Avoid] Step A: turn left {AVOID_LEFT_TURN_DEG:.1f} deg")
    turn_left(
        vehicle, bridge, us_left, lidar_ser, us_right,
        AVOID_LEFT_TURN_DEG, TURN_RATE_DEG_S, TURN_TOLERANCE_DEG,
        check_obstacles=False
    )
    discard_bridge_inputs(bridge, duration_s=0.10)

    # Step B: move left around obstacle
    print(f"[Avoid] Step B: bypass left {AVOID_LEFT_BYPASS_M:.2f} m")
    drive_distance(
        vehicle, bridge, us_left, lidar_ser, us_right,
        AVOID_LEFT_BYPASS_M, SPEED_MPS
    )
    discard_bridge_inputs(bridge, duration_s=0.10)

    # Step C: turn right to face original forward direction
    print(f"[Avoid] Step C: turn right {AVOID_RIGHT_TURN_DEG:.1f} deg")
    turn_right(
        vehicle, bridge, us_left, lidar_ser, us_right,
        AVOID_RIGHT_TURN_DEG, TURN_RATE_DEG_S, TURN_TOLERANCE_DEG,
        check_obstacles=False
    )
    discard_bridge_inputs(bridge, duration_s=0.10)

    # Step D: move forward past the obstacle
    print(f"[Avoid] Step D: clear forward {AVOID_FORWARD_CLEAR_M:.2f} m")
    drive_distance(
        vehicle, bridge, us_left, lidar_ser, us_right,
        AVOID_FORWARD_CLEAR_M, SPEED_MPS
    )
    discard_bridge_inputs(bridge, duration_s=0.10)

    # Step E: turn right to point back toward original path line
    print(f"[Avoid] Step E: turn right {AVOID_RIGHT_TURN_DEG:.1f} deg")
    turn_right(
        vehicle, bridge, us_left, lidar_ser, us_right,
        AVOID_RIGHT_TURN_DEG, TURN_RATE_DEG_S, TURN_TOLERANCE_DEG,
        check_obstacles=False
    )
    discard_bridge_inputs(bridge, duration_s=0.10)

    # Step F: move right to rejoin original path line
    print(f"[Avoid] Step F: return right {AVOID_RETURN_RIGHT_M:.2f} m")
    drive_distance(
        vehicle, bridge, us_left, lidar_ser, us_right,
        AVOID_RETURN_RIGHT_M, SPEED_MPS
    )
    discard_bridge_inputs(bridge, duration_s=0.10)

    # Step G: turn left to restore original heading
    print(f"[Avoid] Step G: turn left {AVOID_LEFT_TURN_DEG:.1f} deg")
    turn_left(
        vehicle, bridge, us_left, lidar_ser, us_right,
        AVOID_LEFT_TURN_DEG, TURN_RATE_DEG_S, TURN_TOLERANCE_DEG,
        check_obstacles=False
    )

    send_stop(vehicle)

    # Final aggressive drain so nothing that arrived during avoidance survives
    discard_bridge_inputs(bridge, duration_s=POST_AVOID_DRAIN_S)

    print("\n[Avoid] Avoidance complete. Resuming fresh UAV commands only.\n")


# ----------------------------
# Main
# ----------------------------
def main():
    bridge = None
    us_right = None
    us_left = None
    lidar_ser = None

    try:
        arm_ugv(vehicle)

        # Open sensors
        us_right = open_ultrasonic(RIGHT_TRIG, RIGHT_ECHO, label="RIGHT")
        us_left = open_ultrasonic(LEFT_TRIG, LEFT_ECHO, label="LEFT")
        lidar_ser = open_lidar()

        print("[Sensors] Warming up lidar (1 s)...")
        time.sleep(1.0)

        bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="UGV-Bridge")
        bridge.connect()

        ready_msg = "aruco ground station armed and awaiting commands"
        bridge.send_message(ready_msg)

        seq = 0

        while True:
            broadcast_status(vehicle, bridge, seq)
            seq += 1

            # Always monitor obstacle sensors first
            sensor_state = read_all_obstacle_sensors(us_left, lidar_ser, us_right)
            if sensor_state["obstacle_detected"]:
                print(
                    f"[Sensors] Obstacle detected: "
                    f"L={sensor_state['left_detected']} "
                    f"C={sensor_state['center_detected']} "
                    f"R={sensor_state['right_detected']}"
                )
                avoid_obstacle_left_and_resume(vehicle, bridge, us_left, lidar_ser, us_right)
                time.sleep(1.0 / TELEM_SEND_HZ)
                continue

            # Normal UAV message handling
            msg_str = bridge.get_message()
            if msg_str:
                print(f"[Ground] Incoming Shout: {msg_str}")
                coords = parse_goto_message(msg_str)
                if coords is not None:
                    x_m, y_m = coords
                    obstacle_triggered = execute_body_goto(
                        vehicle, bridge, us_left, lidar_ser, us_right, x_m, y_m
                    )
                    if obstacle_triggered:
                        avoid_obstacle_left_and_resume(vehicle, bridge, us_left, lidar_ser, us_right)

            cmd = bridge.get_command()
            if cmd:
                cmdSeq, cmdVal, eStop = cmd
                print(f"[Ground] Got Choice Order: {cmdVal}")

                if eStop == 1:
                    print("!!! [ABORT] EMERGENCY DISARM !!!")
                    send_stop(vehicle)
                    vehicle.armed = False
                    time.sleep(1.0 / TELEM_SEND_HZ)
                    continue

                if cmdVal == v2v_bridge.CMD_ARM:
                    arm_ugv(vehicle)

                elif cmdVal == v2v_bridge.CMD_DISARM:
                    send_stop(vehicle)
                    vehicle.armed = False

                elif cmdVal == v2v_bridge.CMD_TURN_RIGHT:
                    print(f"[Ground] >>> TURN RIGHT STEP {TURN_STEP_DEG:.1f} deg")
                    obstacle_triggered = turn_right(
                        vehicle, bridge, us_left, lidar_ser, us_right,
                        TURN_STEP_DEG, TURN_RATE_DEG_S, TURN_TOLERANCE_DEG
                    )
                    if obstacle_triggered:
                        avoid_obstacle_left_and_resume(vehicle, bridge, us_left, lidar_ser, us_right)

                elif cmdVal == v2v_bridge.CMD_TURN_LEFT:
                    print(f"[Ground] >>> TURN LEFT STEP {TURN_STEP_DEG:.1f} deg")
                    obstacle_triggered = turn_left(
                        vehicle, bridge, us_left, lidar_ser, us_right,
                        TURN_STEP_DEG, TURN_RATE_DEG_S, TURN_TOLERANCE_DEG
                    )
                    if obstacle_triggered:
                        avoid_obstacle_left_and_resume(vehicle, bridge, us_left, lidar_ser, us_right)

                elif cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                    print(f"[Ground] >>> MOVE FORWARD {MOVE_FORWARD_10FT_M:.3f} m")
                    obstacle_triggered = drive_distance(
                        vehicle, bridge, us_left, lidar_ser, us_right,
                        MOVE_FORWARD_10FT_M, SPEED_MPS
                    )
                    send_stop(vehicle)
                    if obstacle_triggered:
                        avoid_obstacle_left_and_resume(vehicle, bridge, us_left, lidar_ser, us_right)

                elif cmdVal == v2v_bridge.CMD_MOVE_2FT:
                    print(f"[Ground] >>> MOVE FORWARD {MOVE_FORWARD_2FT_M:.3f} m")
                    obstacle_triggered = drive_distance(
                        vehicle, bridge, us_left, lidar_ser, us_right,
                        MOVE_FORWARD_2FT_M, SPEED_MPS
                    )
                    send_stop(vehicle)
                    if obstacle_triggered:
                        avoid_obstacle_left_and_resume(vehicle, bridge, us_left, lidar_ser, us_right)

                elif cmdVal == v2v_bridge.CMD_STOP:
                    print("[Ground] >>> STOPPED")
                    send_stop(vehicle)

            time.sleep(1.0 / TELEM_SEND_HZ)

    except KeyboardInterrupt:
        print("\n[Ground] Stopped by user.")

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

        if us_right is not None:
            try:
                us_right.close()
            except Exception:
                pass

        if us_left is not None:
            try:
                us_left.close()
            except Exception:
                pass

        if lidar_ser is not None:
            try:
                lidar_ser.close()
            except Exception:
                pass

        if vehicle is not None:
            vehicle.close()

        print("[Ground] Shutdown complete.")


if __name__ == "__main__":
    main()
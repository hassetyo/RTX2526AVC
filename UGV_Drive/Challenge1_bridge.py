from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import logging
import v2v_bridge

# ==========================================
# CHALLENGE 1 GROUND STATION
# Behavior:
#   1) arm
#   2) receive destination
#   3) go straight to y only
#   4) stop
# No turning. No x movement.
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
MOVEMENT_EPS_MPS = 0.05
TELEM_SEND_HZ = 5

# ----------------------------
# Unit conversions
# ----------------------------
MPH_TO_MPS = 0.44704
SPEED_MPS = SPEED_MPH * MPH_TO_MPS

# ----------------------------
# Logging
# ----------------------------
LOG_FILE = "UGVChallenge1.log"

logger = logging.getLogger("UGVChallenge1")
logger.setLevel(logging.INFO)
logger.handlers.clear()

file_handler = logging.FileHandler(LOG_FILE, mode="w")
file_handler.setFormatter(logging.Formatter("%(message)s"))
logger.addHandler(file_handler)
logger.propagate = False


def log_line(text):
    logger.info(text)


def timestamp():
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


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
    for state in (True, False, True):
        vehicle.armed = state
        if not wait_for_armed(vehicle, state):
            raise RuntimeError(f"Failed to set armed={state}")
        time.sleep(1.0)

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


def build_stop_msg(vehicle):
    return build_velocity_msg(vehicle, 0.0)


def send_stop(vehicle):
    vehicle.send_mavlink(build_stop_msg(vehicle))
    time.sleep(0.5)


def get_groundspeed(vehicle):
    return vehicle.groundspeed if vehicle.groundspeed is not None else 0.0


def drive_distance_velocity(vehicle, bridge, distance_m, speed_mps, detection_window_s=1.5):
    if distance_m <= 0:
        send_stop(vehicle)
        return True

    duration_s = distance_m / speed_mps
    drive_msg = build_velocity_msg(vehicle, speed_mps)
    stop_msg = build_stop_msg(vehicle)

    start_t = time.time()
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

        if elapsed >= detection_window_s and not movement_detected:
            break

        time.sleep(0.1)

    vehicle.send_mavlink(stop_msg)
    time.sleep(0.5)
    return movement_detected


def drive_distance_attitude(vehicle, bridge, distance_m, speed_mps):
    if distance_m <= 0:
        send_stop(vehicle)
        return

    duration_s = distance_m / speed_mps
    original_wp_speed = None

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
            vehicle.send_mavlink(drive_msg)
            broadcast_status(vehicle, bridge, seq)
            seq += 1
            time.sleep(0.1)
    finally:
        vehicle.send_mavlink(stop_msg)
        time.sleep(0.5)
        vehicle.parameters["WP_SPEED"] = original_wp_speed
        time.sleep(0.5)


def drive_distance(vehicle, bridge, distance_m, speed_mps):
    moved = drive_distance_velocity(vehicle, bridge, distance_m, speed_mps)
    if not moved:
        drive_distance_attitude(vehicle, bridge, distance_m, speed_mps)


def parse_goto_message(msg_str):
    if not msg_str.startswith("GOTO:"):
        return None

    payload = msg_str.split(":", 1)[1].strip()
    x_str, y_str = payload.split(",")
    x_val = float(x_str.strip())
    y_val = float(y_str.strip())
    return x_val, y_val


def main():
    bridge = None
    vehicle = None

    ugv_start_time = timestamp()
    uav_start_time = None
    uav_end_time = None

    log_line(f"UAV Start Time: ")
    log_line(f"Destination discovery: ")
    log_line(f"Communication between Autonomous Vehicles: ")
    log_line(f"Location of the destination: ")
    log_line(f"UGV Start Time: {ugv_start_time}")
    log_line(f"UGV receipt of destination location: ")
    log_line(f"UGV generated path: ")
    log_line(f"UGV speed: ")
    log_line(f"UGV End time: ")
    log_line(f"UAV End time: ")

    try:
        vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)
        arm_ugv(vehicle)

        bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="UGV-Bridge")
        bridge.connect()

        ready_msg = "challenge 1 ground station armed and awaiting destination"
        bridge.send_message(ready_msg)

        seq = 0

        while True:
            broadcast_status(vehicle, bridge, seq)
            seq += 1

            msg_str = bridge.get_message()
            if not msg_str:
                time.sleep(1.0 / TELEM_SEND_HZ)
                continue

            if uav_start_time is None:
                uav_start_time = timestamp()

            coords = parse_goto_message(msg_str)
            if coords is None:
                time.sleep(1.0 / TELEM_SEND_HZ)
                continue

            x_m, y_m = coords

            # Challenge 1 uses only y as the straight destination distance.
            destination_distance_m = abs(y_m)

            # Rewrite the log cleanly with only the required items populated.
            logger.handlers[0].stream.seek(0)
            logger.handlers[0].stream.truncate()

            log_line(f"UAV Start Time: {uav_start_time}")
            log_line("Destination discovery: Destination coordinates received from UAV")
            log_line(f"Communication between Autonomous Vehicles: UAV -> UGV: {msg_str} | UGV -> UAV: {ready_msg}")
            log_line(f"Location of the destination: y={y_m:.3f} m")
            log_line(f"UGV Start Time: {ugv_start_time}")
            log_line(f"UGV receipt of destination location: y={y_m:.3f} m")
            log_line(f"UGV generated path: Straight to y only, distance={destination_distance_m:.3f} m")
            log_line(f"UGV speed: {SPEED_MPH:.1f} mph ({SPEED_MPS:.4f} m/s)")

            drive_distance(vehicle, bridge, destination_distance_m, SPEED_MPS)
            send_stop(vehicle)

            complete_msg = f"challenge1_complete:y={y_m:.2f}"
            bridge.send_message(complete_msg)
            uav_end_time = timestamp()
            ugv_end_time = timestamp()

            log_line(f"UGV End time: {ugv_end_time}")
            log_line(f"UAV End time: {uav_end_time}")
            break

    except KeyboardInterrupt:
        pass
    finally:
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

        if vehicle is not None:
            vehicle.close()


if __name__ == "__main__":
    main()
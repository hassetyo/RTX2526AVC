from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import v2v_bridge

# ==========================================
# GPS GROUND STATION
#
# Repeat-safe command behavior:
#   UAV can repeatedly send:
#       GPS_CMD:12,32.985123,-96.750456
#       FORWARD_CMD:13,5.0
#
#   UGV responses:
#       CMD_STARTED:12
#       CMD_ACTIVE:12
#       CMD_DONE:12
#       CMD_FAILED:12
#       CMD_BUSY:12
#
# GPS status broadcast:
#       GPS_STATUS:lat,lon,alt,heading,fix,sats
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
SPEED_MPH = 5.0
MPH_TO_MPS = 0.44704
SPEED_MPS = SPEED_MPH * MPH_TO_MPS

MOVE_FORWARD_10FT_M = 3.048
MOVE_FORWARD_2FT_M = 0.61

GPS_ARRIVAL_RADIUS_M = 0.75
GPS_TIMEOUT_S = 120
GPS_MIN_FIX_TYPE = 3
GPS_MIN_SATS = 6

TELEM_SEND_HZ = 5
GPS_SEND_HZ = 2

# Prevent completed command list from growing forever
MAX_COMPLETED_COMMANDS = 100


print("==========================================")
print("   UGV GROUND STATION - GPS COMMAND READY")
print("==========================================")
print(f"[Ground] Connecting to UGV at {UGV_CONTROL_PORT}...")

try:
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)
    print("[Ground] Connected! Ready to sync.")
except Exception as e:
    print(f"!!! Error connecting to UGV: {e} !!!")
    raise SystemExit(1)


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


def wait_for_gps(vehicle, timeout_s=60):
    print("[Ground] Waiting for GPS fix...")

    deadline = time.time() + timeout_s

    while time.time() < deadline:
        try:
            fix_type = vehicle.gps_0.fix_type
            sats = vehicle.gps_0.satellites_visible
            loc = vehicle.location.global_relative_frame

            if (
                fix_type >= GPS_MIN_FIX_TYPE
                and sats >= GPS_MIN_SATS
                and loc.lat is not None
                and loc.lon is not None
            ):
                print(f"[Ground] GPS OK: fix={fix_type}, sats={sats}")
                print(f"[Ground] Current GPS: lat={loc.lat}, lon={loc.lon}")
                return True

            print(f"[Ground] Waiting GPS... fix={fix_type}, sats={sats}")

        except Exception as e:
            print(f"[Ground] GPS read error: {e}")

        time.sleep(1.0)

    print("[Ground] GPS timeout. Cannot safely use GPS navigation.")
    return False


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
        raise RuntimeError(
            f"Failed to enter GUIDED mode, current mode: {vehicle.mode.name}"
        )

    print("[Ground] GUIDED mode confirmed.")
    print("[Ground] UGV FULLY ARMED AND IN GUIDED MODE.\n")


def ensure_ready(vehicle):
    if vehicle.mode.name != "GUIDED":
        print(f"[Ground] Vehicle mode is {vehicle.mode.name}. Switching to GUIDED...")
        vehicle.mode = VehicleMode("GUIDED")

        if not wait_for_mode(vehicle, "GUIDED"):
            raise RuntimeError(
                f"Failed to enter GUIDED mode, current mode: {vehicle.mode.name}"
            )

    if not vehicle.armed:
        print("[Ground] Vehicle not armed. Re-arming...")
        arm_ugv(vehicle)


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

    gps_bit = 0x20 if gps_fix >= GPS_MIN_FIX_TYPE else 0x00

    safety_byte = (mode_idx & 0x0F) | armable_bit | gps_bit
    t_ms = int(time.time() * 1000) & 0xFFFFFFFF
    v_mps = vehicle.groundspeed if vehicle.groundspeed is not None else 0.0

    bridge.send_telemetry(seq, t_ms, v_mps, 0.0, armed_val, safety_byte)


def send_gps_coordinates(vehicle, bridge):
    try:
        loc = vehicle.location.global_relative_frame

        if loc.lat is None or loc.lon is None:
            bridge.send_message("GPS_STATUS:UNAVAILABLE")
            return

        lat = loc.lat
        lon = loc.lon
        alt = loc.alt if loc.alt is not None else 0.0
        heading = vehicle.heading if vehicle.heading is not None else 0

        try:
            fix = vehicle.gps_0.fix_type
            sats = vehicle.gps_0.satellites_visible
        except Exception:
            fix = 0
            sats = 0

        msg = f"GPS_STATUS:{lat:.7f},{lon:.7f},{alt:.2f},{heading},{fix},{sats}"
        bridge.send_message(msg)

    except Exception as e:
        print(f"[Ground] Failed to send GPS coordinates: {e}")


def build_stop_msg(vehicle):
    return vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0x0DE7,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def send_stop(vehicle):
    vehicle.send_mavlink(build_stop_msg(vehicle))
    vehicle.flush()
    time.sleep(0.5)


def get_current_location(vehicle):
    loc = vehicle.location.global_relative_frame

    if loc.lat is None or loc.lon is None:
        raise RuntimeError("Current GPS location unavailable.")

    return loc


def get_distance_meters(loc1, loc2):
    radius_earth_m = 6371000.0

    lat1 = math.radians(loc1.lat)
    lon1 = math.radians(loc1.lon)
    lat2 = math.radians(loc2.lat)
    lon2 = math.radians(loc2.lon)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )

    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return radius_earth_m * c


def location_from_distance_and_bearing(start_loc, distance_m, bearing_deg):
    radius_earth_m = 6378137.0

    bearing = math.radians(bearing_deg)
    lat1 = math.radians(start_loc.lat)
    lon1 = math.radians(start_loc.lon)

    angular_distance = distance_m / radius_earth_m

    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular_distance)
        + math.cos(lat1) * math.sin(angular_distance) * math.cos(bearing)
    )

    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(angular_distance) * math.cos(lat1),
        math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2),
    )

    return LocationGlobalRelative(
        math.degrees(lat2),
        math.degrees(lon2),
        start_loc.alt if start_loc.alt is not None else 0,
    )


def get_heading(vehicle):
    if vehicle.heading is None:
        raise RuntimeError("Vehicle heading unavailable.")

    return float(vehicle.heading)


def goto_gps(vehicle, bridge, target_lat, target_lon, speed_mps=SPEED_MPS):
    ensure_ready(vehicle)

    if not wait_for_gps(vehicle, timeout_s=20):
        print("[Ground] GPS not ready. Aborting GPS goto.")
        return False

    current = get_current_location(vehicle)

    target = LocationGlobalRelative(
        target_lat,
        target_lon,
        current.alt if current.alt else 0,
    )

    print("[Ground] >>> GPS GOTO")
    print(f"[Ground] Current: lat={current.lat:.7f}, lon={current.lon:.7f}")
    print(f"[Ground] Target:  lat={target.lat:.7f}, lon={target.lon:.7f}")

    vehicle.groundspeed = speed_mps
    vehicle.simple_goto(target, groundspeed=speed_mps)

    start_t = time.time()
    seq = 0
    last_gps_send_t = 0.0

    while time.time() - start_t < GPS_TIMEOUT_S:
        broadcast_status(vehicle, bridge, seq)
        seq += 1

        now = time.time()

        if now - last_gps_send_t >= 1.0 / GPS_SEND_HZ:
            send_gps_coordinates(vehicle, bridge)
            last_gps_send_t = now

        current = get_current_location(vehicle)
        remaining = get_distance_meters(current, target)

        print(f"[Ground] Distance to target: {remaining:.2f} m")

        if remaining <= GPS_ARRIVAL_RADIUS_M:
            print("[Ground] GPS target reached.")
            send_stop(vehicle)
            return True

        time.sleep(1.0)

    print("[Ground] GPS goto timeout.")
    send_stop(vehicle)
    return False


def drive_forward_gps(vehicle, bridge, distance_m, speed_mps=SPEED_MPS):
    ensure_ready(vehicle)

    if distance_m <= 0:
        print("[Ground] Distance must be greater than 0.")
        send_stop(vehicle)
        return False

    if not wait_for_gps(vehicle, timeout_s=20):
        print("[Ground] GPS not ready. Aborting forward GPS drive.")
        return False

    start_loc = get_current_location(vehicle)
    heading_deg = get_heading(vehicle)

    target = location_from_distance_and_bearing(start_loc, distance_m, heading_deg)

    print(f"[Ground] >>> GPS FORWARD {distance_m:.2f} m")
    print(f"[Ground] Start:   lat={start_loc.lat:.7f}, lon={start_loc.lon:.7f}")
    print(f"[Ground] Heading: {heading_deg:.1f} deg")
    print(f"[Ground] Target:  lat={target.lat:.7f}, lon={target.lon:.7f}")

    return goto_gps(vehicle, bridge, target.lat, target.lon, speed_mps)


def remember_completed_command(command_state, command_id):
    command_state["completed"].add(command_id)
    command_state["completed_order"].append(command_id)

    while len(command_state["completed_order"]) > MAX_COMPLETED_COMMANDS:
        old_id = command_state["completed_order"].pop(0)
        command_state["completed"].discard(old_id)


def parse_bridge_message(msg_str):
    """
    Supported repeat-safe formats:

    GPS_CMD:12,32.985123,-96.750456
        command_id = 12
        target latitude = 32.985123
        target longitude = -96.750456

    FORWARD_CMD:13,5.0
        command_id = 13
        drive forward 5.0 meters using current GPS heading

    Also supports old one-shot formats:
        GPS:32.985123,-96.750456
        GOTO_GPS:32.985123,-96.750456
        GOTO:32.985123,-96.750456
        FORWARD:5.0
    """

    if not msg_str:
        return None

    msg = msg_str.strip()

    try:
        if msg.upper().startswith("GPS_CMD:"):
            payload = msg.split(":", 1)[1].strip()
            cmd_id_str, lat_str, lon_str = payload.split(",")
            return ("GPS", int(cmd_id_str), float(lat_str), float(lon_str), True)

        if msg.upper().startswith("FORWARD_CMD:"):
            payload = msg.split(":", 1)[1].strip()
            cmd_id_str, distance_str = payload.split(",")
            return ("FORWARD", int(cmd_id_str), float(distance_str), True)

        if msg.upper().startswith("GPS:"):
            payload = msg.split(":", 1)[1].strip()
            lat_str, lon_str = payload.split(",")
            return ("GPS", None, float(lat_str), float(lon_str), False)

        if msg.upper().startswith("GOTO_GPS:"):
            payload = msg.split(":", 1)[1].strip()
            lat_str, lon_str = payload.split(",")
            return ("GPS", None, float(lat_str), float(lon_str), False)

        if msg.upper().startswith("GOTO:"):
            payload = msg.split(":", 1)[1].strip()
            lat_str, lon_str = payload.split(",")
            return ("GPS", None, float(lat_str), float(lon_str), False)

        if msg.upper().startswith("FORWARD:"):
            distance_str = msg.split(":", 1)[1].strip()
            return ("FORWARD", None, float(distance_str), False)

    except Exception as e:
        print(f"[Ground] Failed to parse message '{msg_str}': {e}")
        return None

    return None


def execute_parsed_message(vehicle, bridge, parsed, command_state):
    if parsed is None:
        return

    command_type = parsed[0]
    command_id = parsed[1]
    is_repeat_safe = parsed[-1]

    # Old one-shot commands still execute immediately.
    if not is_repeat_safe:
        if command_type == "FORWARD":
            distance_m = parsed[2]
            drive_forward_gps(vehicle, bridge, distance_m, SPEED_MPS)

        elif command_type == "GPS":
            lat = parsed[2]
            lon = parsed[3]
            goto_gps(vehicle, bridge, lat, lon, SPEED_MPS)

        return

    # Repeat-safe command handling.
    if command_id in command_state["completed"]:
        print(f"[Ground] Command {command_id} already completed. Re-sending CMD_DONE.")
        bridge.send_message(f"CMD_DONE:{command_id}")
        return

    if command_state["active"] == command_id:
        print(f"[Ground] Command {command_id} already active.")
        bridge.send_message(f"CMD_ACTIVE:{command_id}")
        return

    if command_state["active"] is not None:
        active_id = command_state["active"]
        print(f"[Ground] Busy with command {active_id}. Ignoring command {command_id}.")
        bridge.send_message(f"CMD_BUSY:{active_id}")
        return

    command_state["active"] = command_id
    bridge.send_message(f"CMD_STARTED:{command_id}")

    success = False

    try:
        if command_type == "FORWARD":
            distance_m = parsed[2]
            print(f"[Ground] Starting repeat-safe FORWARD command {command_id}: {distance_m:.2f} m")
            success = drive_forward_gps(vehicle, bridge, distance_m, SPEED_MPS)

        elif command_type == "GPS":
            lat = parsed[2]
            lon = parsed[3]
            print(f"[Ground] Starting repeat-safe GPS command {command_id}: {lat}, {lon}")
            success = goto_gps(vehicle, bridge, lat, lon, SPEED_MPS)

    except Exception as e:
        print(f"[Ground] Command {command_id} failed with error: {e}")
        success = False

    if success:
        remember_completed_command(command_state, command_id)
        bridge.send_message(f"CMD_DONE:{command_id}")
        print(f"[Ground] CMD_DONE:{command_id}")
    else:
        bridge.send_message(f"CMD_FAILED:{command_id}")
        print(f"[Ground] CMD_FAILED:{command_id}")

    command_state["active"] = None


def main():
    bridge = None

    command_state = {
        "active": None,
        "completed": set(),
        "completed_order": [],
    }

    try:
        if not wait_for_gps(vehicle, timeout_s=60):
            return

        arm_ugv(vehicle)

        bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="UGV-Bridge")
        bridge.connect()

        bridge.send_message("gps ground station armed and awaiting commands")

        seq = 0
        last_gps_send_t = 0.0

        while True:
            broadcast_status(vehicle, bridge, seq)
            seq += 1

            now = time.time()

            if now - last_gps_send_t >= 1.0 / GPS_SEND_HZ:
                send_gps_coordinates(vehicle, bridge)
                last_gps_send_t = now

            msg_str = bridge.get_message()

            if msg_str:
                print(f"[Ground] Incoming Message: {msg_str}")

                parsed = parse_bridge_message(msg_str)

                if parsed is not None:
                    execute_parsed_message(vehicle, bridge, parsed, command_state)
                else:
                    print("[Ground] Message ignored. Unknown format.")

            cmd = bridge.get_command()

            if cmd:
                cmdSeq, cmdVal, eStop = cmd
                print(f"[Ground] Got Choice Order: {cmdVal}")

                if eStop == 1:
                    print("!!! [ABORT] EMERGENCY DISARM !!!")
                    send_stop(vehicle)
                    vehicle.armed = False
                    command_state["active"] = None
                    continue

                if cmdVal == v2v_bridge.CMD_ARM:
                    arm_ugv(vehicle)

                elif cmdVal == v2v_bridge.CMD_DISARM:
                    send_stop(vehicle)
                    vehicle.armed = False
                    command_state["active"] = None

                elif cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                    print(f"[Ground] >>> GPS MOVE FORWARD {MOVE_FORWARD_10FT_M:.3f} m")
                    drive_forward_gps(vehicle, bridge, MOVE_FORWARD_10FT_M, SPEED_MPS)

                elif cmdVal == v2v_bridge.CMD_MOVE_2FT:
                    print(f"[Ground] >>> GPS MOVE FORWARD {MOVE_FORWARD_2FT_M:.3f} m")
                    drive_forward_gps(vehicle, bridge, MOVE_FORWARD_2FT_M, SPEED_MPS)

                elif cmdVal == v2v_bridge.CMD_STOP:
                    print("[Ground] >>> STOPPED")
                    send_stop(vehicle)
                    command_state["active"] = None

                else:
                    print("[Ground] Command ignored in GPS mode.")

            time.sleep(1.0 / TELEM_SEND_HZ)

    except KeyboardInterrupt:
        print("[Ground] Keyboard interrupt.")

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

        if vehicle is not None:
            vehicle.close()


if __name__ == "__main__":
    main()
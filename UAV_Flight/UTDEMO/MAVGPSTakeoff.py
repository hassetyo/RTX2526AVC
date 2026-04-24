from __future__ import print_function
import time
import argparse
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DEFAULT_ALTITUDE = 10          # metres
LAND_ALTITUDE_THRESHOLD = 0.5  # metres — considered landed below this
CONNECTION_STRING_LIVE = '/dev/ttyACM0'
CONNECTION_BAUD_LIVE = 115200
SITL_LAT = 32.92019271850586
SITL_LON = -96.94831085205078

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Simple GPS takeoff and land script.")
parser.add_argument(
    "--livedrone",
    action="store_true",
    help="Connect to a real drone instead of SITL simulation.",
)
parser.add_argument(
    "--altitude",
    type=float,
    default=DEFAULT_ALTITUDE,
    help=f"Target altitude in metres (default: {DEFAULT_ALTITUDE}).",
)
args = parser.parse_args()

SIMULATE_DRONE = not args.livedrone
TARGET_ALTITUDE = args.altitude


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def connect_vehicle():
    """Connect to the vehicle via SITL or a live serial connection."""
    if SIMULATE_DRONE:
        import dronekit_sitl
        print(f"Starting SITL at ({SITL_LAT}, {SITL_LON})...")
        sitl = dronekit_sitl.start_default(SITL_LAT, SITL_LON)
        connection_string = sitl.connection_string()
        print(f"Connecting via pymavlink to {connection_string}")
    else:
        connection_string = CONNECTION_STRING_LIVE
        print(f"Connecting to live drone on {connection_string}")

    master = mavutil.mavlink_connection(connection_string, baud=CONNECTION_BAUD_LIVE)
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat received (system {master.target_system}, "
          f"component {master.target_component})")
    return master


def request_message_streams(master):
    """Request all data streams and ensure position messages are flowing."""
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,  # 10 Hz
        1    # start
    )

    def set_interval(msg_id, hz):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(1e6 / hz),  # interval in microseconds
            0, 0, 0, 0, 0
        )

    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2)


def set_param(master, param_id, value):
    """Set a MAVLink parameter and wait for confirmation."""
    master.mav.param_set_send(
        master.target_system, master.target_component,
        param_id.encode('utf-8'),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    # Wait for PARAM_VALUE echo confirming the write
    deadline = time.time() + 5
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and msg.param_id.rstrip('\x00') == param_id:
            print(f"Param {param_id} = {msg.param_value}")
            return
    print(f"Warning: no confirmation for param {param_id}")


def get_altitude(master):
    """Return relative altitude in metres from GLOBAL_POSITION_INT."""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    if msg:
        return msg.relative_alt / 1000.0  # mm -> metres
    return 0.0


def get_gps_fix(master):
    """Return the current GPS fix type (0=no fix, 2=2D, 3=3D)."""
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
    return msg.fix_type if msg else 0


def get_battery_voltage(master):
    """Return battery voltage in volts from SYS_STATUS."""
    msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
    return msg.voltage_battery / 1000.0 if msg else 0.0


def wait_for_armable(master):
    """Block until the vehicle has a GPS fix and reports STANDBY system status."""
    print("Waiting for vehicle to become armable...")
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb is None:
            print("  No heartbeat — retrying...")
            continue

        gps_fix = get_gps_fix(master)
        standby = (hb.system_status == mavutil.mavlink.MAV_STATE_STANDBY)

        if standby and gps_fix >= 3:
            break

        battery_v = get_battery_voltage(master)
        print(f"  Not armable yet — GPS fix: {gps_fix}, "
              f"system status: {hb.system_status}, battery: {battery_v:.1f} V")
        time.sleep(1)

    print("Vehicle is armable.")


def set_mode(master, mode_name):
    """Set the flight mode and block until confirmed via HEARTBEAT."""
    mapping = master.mode_mapping()
    if mode_name not in mapping:
        raise RuntimeError(
            f"Unknown mode '{mode_name}'. Available: {list(mapping.keys())}"
        )
    mode_id = mapping[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    deadline = time.time() + 10
    while time.time() < deadline:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and hb.custom_mode == mode_id:
            print(f"{mode_name} mode active.")
            return
        print(f"  Waiting for {mode_name} mode...")

    raise TimeoutError(f"Timed out waiting for {mode_name} mode.")


def arm_vehicle(master):
    """Send arm command and block until the vehicle reports armed."""
    # Disable pre-arm checks in SITL so GPS/EKF warm-up doesn't block arming
    if SIMULATE_DRONE:
        set_param(master, 'ARMING_CHECK', 0)
        time.sleep(2)  # give the FC time to apply the param before arming

    # Retry loop — ArduCopter can reject the first attempt while transitioning
    for attempt in range(1, 6):
        print(f"Arming (attempt {attempt})...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,   # confirmation
            1,   # param1: 1 = arm
            0, 0, 0, 0, 0, 0
        )

        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                break
            print(f"  Arm rejected (MAV_RESULT={ack.result}), retrying...")
        else:
            print("  No ACK received, retrying...")
        time.sleep(2)
    else:
        raise RuntimeError("Arm command repeatedly rejected by FC. Check pre-arm status.")

    # Poll HEARTBEAT until armed flag is set
    deadline = time.time() + 15
    while time.time() < deadline:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("Vehicle armed.")
            return
        print("  Waiting for armed status...")

    raise TimeoutError("Timed out waiting for the vehicle to arm.")


def arm_and_set_guided(master):
    """Set GUIDED mode then arm."""
    set_mode(master, 'GUIDED')
    arm_vehicle(master)
    time.sleep(1)  # allow FC to fully transition to armed state before takeoff


def takeoff(master, target_altitude):
    """Issue a takeoff command and block until the target altitude is reached."""
    print(f"Taking off to {target_altitude} m...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,               # confirmation
        0, 0, 0, 0,      # params 1-4 (unused for copter)
        0, 0,            # lat, lon (use current position)
        target_altitude  # param7: altitude
    )

    time.sleep(2)  # give the FC time to begin climbing before polling
    while True:
        current_alt = get_altitude(master)
        print(f"  Altitude: {current_alt:.1f} m")
        if current_alt >= target_altitude * 0.95:
            print("Target altitude reached.")
            break
        time.sleep(1)


def land(master):
    """Switch to LAND mode and block until the vehicle has touched down."""
    print("Initiating landing...")
    set_mode(master, 'LAND')

    while True:
        current_alt = get_altitude(master)
        if current_alt <= LAND_ALTITUDE_THRESHOLD:
            break
        print(f"  Altitude: {current_alt:.1f} m — descending...")
        time.sleep(1)

    print("Vehicle has landed.")


def print_vehicle_state(master):
    """Print a one-line summary of GPS position, altitude, mode, and arm state."""
    pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    gps_fix = get_gps_fix(master)
    hb = master.recv_match(type='HEARTBEAT', blocking=False)

    lat = pos.lat / 1e7 if pos else 0.0
    lon = pos.lon / 1e7 if pos else 0.0
    alt = pos.relative_alt / 1000.0 if pos else 0.0

    mode = "UNKNOWN"
    armed = False
    if hb:
        inv_mapping = {v: k for k, v in master.mode_mapping().items()}
        mode = inv_mapping.get(hb.custom_mode, str(hb.custom_mode))
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    print(f"  GPS fix: {gps_fix} | Lat: {lat:.7f} | Lon: {lon:.7f} | "
          f"Alt: {alt:.1f} m | Mode: {mode} | Armed: {armed}")


# ---------------------------------------------------------------------------
# Main mission
# ---------------------------------------------------------------------------

def main():
    master = connect_vehicle()
    request_message_streams(master)

    try:
        print("\n--- Vehicle state at connection ---")
        print_vehicle_state(master)

        wait_for_armable(master)
        arm_and_set_guided(master)
        takeoff(master, TARGET_ALTITUDE)

        print("\n--- Hovering for 10 seconds ---")
        for _ in range(10):
            print_vehicle_state(master)
            time.sleep(1)

        land(master)

        print("\n--- Final vehicle state ---")
        print_vehicle_state(master)

    finally:
        print("Closing vehicle connection.")
        master.close()


if __name__ == "__main__":
    main()

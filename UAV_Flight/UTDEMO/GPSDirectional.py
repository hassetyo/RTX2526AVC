from pymavlink import mavutil
import time
import math
import GPSv2v_bridge as v2v_bridge

# CONNECTION 
CONNECTION_STRING = "/dev/ttyACM0" #"udp:127.0.0.1:14551" 
BAUD_RATE = 57600

# FLIGHT PARAMS 
TARGET_ALT_M    = 5.0
HOVER_TIMEOUT_S = 10
LAND_TIMEOUT_S  = 90

# BRIDGE PARAMS
ESP32_BRIDGE_PORT = "/dev/ttyUSB0"
BRIDGE_BAUD_RATE  = 115200
BRIDGE_GPS_TIMEOUT_S = 30
UGV_COMMAND_TIMEOUT_S = 120
UGV_MOVE_FORWARD_M = 5.0
UGV_MOVE_RIGHT_M = 5.0

# Waypoint acceptance radius (metres) and per-leg timeout (seconds)
WP_TOLERANCE_M  = 0.5
WP_TIMEOUT_S    = 30

# LOGGING 
def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")

# MAVLINK SETUP 
def connect():
    log(f"Connecting to {CONNECTION_STRING}")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    log("Heartbeat received")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1
    )
    return master

# GPS CHECK 
def wait_for_gps(master, timeout=30):
    log("Waiting for strong GPS fix...")
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if msg and msg.fix_type >= 3 and msg.satellites_visible >= 8:
            log(f"GPS OK (fix={msg.fix_type}, sats={msg.satellites_visible})")
            return True
    log("GPS timeout!")
    return False

# POSITION CHECK 
def wait_for_position(master, timeout=30):
    log("Waiting for position estimate (EKF)...")
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            log("Position estimate acquired")
            return True
    log("Position timeout!")
    return False

# MODE CONTROL 
def set_mode(master, mode):
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise Exception(f"Mode {mode} not supported. Available: {list(mapping.keys())}")
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    log(f"Mode -> {mode}")
    deadline = time.time() + 5
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb:
            current_mode = mavutil.mode_string_v10(hb)
            if current_mode == mode:
                log(f"Confirmed mode: {mode}")
                return True
    log(f"Warning: Mode change to {mode} not confirmed")
    return False

# ARMING 
def arm(master):
    log("Arming...")
    master.arducopter_arm()
    deadline = time.time() + 15
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log("Armed successfully")
            return True
    log("Failed to arm (check pre-arm errors)")
    return False

# TAKEOFF 
def takeoff(master, alt):
    log(f"Taking off to {alt} m")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt
    )
    start = time.time()
    while time.time() - start < 25:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg:
            rel_alt = msg.relative_alt / 1000.0
            print(f"Altitude: {rel_alt:.2f} m", end="\r")
            if rel_alt >= alt * 0.95:
                print()
                log("Reached target altitude")
                return True
    print()
    log("Takeoff timeout!")
    return False

# LANDING 
def land(master):
    log("Landing...")
    set_mode(master, "LAND")
    deadline = time.time() + LAND_TIMEOUT_S
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb:
            armed = hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:
                log("Disarmed — landed")
                return
    log("Landing timeout — forcing disarm")
    master.arducopter_disarm()

# POSITION HELPERS 

def get_current_position(master):
    """
    Return the current (lat_deg, lon_deg, rel_alt_m) from GLOBAL_POSITION_INT.
    Blocks for up to 3 seconds.
    """
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    if msg is None:
        raise RuntimeError("Could not get current position — no GLOBAL_POSITION_INT message.")
    return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0


def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    """
    Compute a new (lat, lon) that is north_m metres north and east_m metres east
    of the given coordinate.  Uses a flat-earth approximation valid up to ~1 km.

    Args:
        lat_deg: starting latitude in decimal degrees
        lon_deg: starting longitude in decimal degrees
        north_m: offset in metres (+north / -south)
        east_m:  offset in metres (+east  / -west)

    Returns:
        (new_lat_deg, new_lon_deg)
    """
    d_lat = north_m / 111_111.0
    d_lon = east_m  / (111_111.0 * math.cos(math.radians(lat_deg)))
    return lat_deg + d_lat, lon_deg + d_lon


def offset_by_bearing(lat_deg, lon_deg, bearing_deg, distance_m):
    """Compute a new GPS coordinate distance_m metres along bearing_deg."""
    rad = math.radians(bearing_deg)
    north_m = distance_m * math.cos(rad)
    east_m = distance_m * math.sin(rad)
    return offset_latlon(lat_deg, lon_deg, north_m, east_m)


def haversine_distance(lat1, lon1, lat2, lon2):
    """Return the great-circle distance in metres between two lat/lon points."""
    R = 6_371_000  # Earth radius in metres
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def send_waypoint(master, lat_deg, lon_deg, alt_m):
    """
    Command the drone to fly to an absolute GPS waypoint at the given altitude
    using SET_POSITION_TARGET_GLOBAL_INT (works in GUIDED mode).
    """
    master.mav.set_position_target_global_int_send(
        0,                                              # time_boot_ms (ignored)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000_1111_1111_1000,                          # type_mask: position only
        int(lat_deg * 1e7),
        int(lon_deg * 1e7),
        alt_m,
        0, 0, 0,                                        # vx, vy, vz (ignored)
        0, 0, 0,                                        # ax, ay, az (ignored)
        0, 0                                            # yaw, yaw_rate (ignored)
    )


def wait_until_reached(master, lat_deg, lon_deg,
                       tolerance_m=WP_TOLERANCE_M, timeout_s=WP_TIMEOUT_S):
    """
    Block until the drone is within tolerance_m metres of (lat_deg, lon_deg),
    or until timeout_s seconds elapse.

    Returns True if the waypoint was reached, False on timeout.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            cur_lat = msg.lat / 1e7
            cur_lon = msg.lon / 1e7
            dist = haversine_distance(cur_lat, cur_lon, lat_deg, lon_deg)
            print(f"  Distance to waypoint: {dist:.2f} m", end="\r")
            if dist <= tolerance_m:
                print()
                return True
    print()
    log(f"Timeout reaching waypoint ({lat_deg:.6f}, {lon_deg:.6f})")
    return False


def goto_offset(master, north_m, east_m, alt_m=None,
                tolerance_m=WP_TOLERANCE_M, timeout_s=WP_TIMEOUT_S):
    """
    Fly north_m metres north and east_m metres east of the current position.
    Altitude is maintained unless alt_m is explicitly provided.

    Returns True if the waypoint was reached.
    """
    cur_lat, cur_lon, cur_alt = get_current_position(master)
    target_lat, target_lon = offset_latlon(cur_lat, cur_lon, north_m, east_m)
    target_alt = alt_m if alt_m is not None else cur_alt

    log(f"Flying N={north_m:+.1f} m  E={east_m:+.1f} m  "
        f"-> ({target_lat:.6f}, {target_lon:.6f})  alt={target_alt:.1f} m")

    send_waypoint(master, target_lat, target_lon, target_alt)
    reached = wait_until_reached(master, target_lat, target_lon, tolerance_m, timeout_s)
    if reached:
        log("Waypoint reached.")
    return reached


# CARDINAL DIRECTION HELPERS 

def fly_north(master, metres, **kw):
    """Fly metres north of the current position."""
    return goto_offset(master, north_m=metres, east_m=0, **kw)

def fly_south(master, metres, **kw):
    """Fly metres south of the current position."""
    return goto_offset(master, north_m=-metres, east_m=0, **kw)

def fly_east(master, metres, **kw):
    """Fly metres east of the current position."""
    return goto_offset(master, north_m=0, east_m=metres, **kw)

def fly_west(master, metres, **kw):
    """Fly metres west of the current position."""
    return goto_offset(master, north_m=0, east_m=-metres, **kw)

def fly_bearing(master, bearing_deg, distance_m, **kw):
    """
    Fly distance_m metres in the direction of bearing_deg (0=N, 90=E, 180=S, 270=W).
    """
    rad = math.radians(bearing_deg)
    north_m = distance_m * math.cos(rad)
    east_m  = distance_m * math.sin(rad)
    log(f"Flying bearing={bearing_deg}°  dist={distance_m} m")
    return goto_offset(master, north_m=north_m, east_m=east_m, **kw)

#BRIDGE SETUP

def connect_bridge():
    """Open the V2V bridge to the ESP32 radio."""
    bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, baud=BRIDGE_BAUD_RATE, name="UAV-Bridge")
    bridge.connect()
    return bridge


def wait_for_ugv_status(bridge, timeout_s=BRIDGE_GPS_TIMEOUT_S):
    """Wait for an available GPS_STATUS update from the UGV."""
    log(f"Waiting up to {timeout_s}s for UGV GPS_STATUS from bridge...")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        status = bridge.wait_for_gps_status(timeout_s=0.5)
        if status and status.get("available") and status.get("lat") is not None and status.get("lon") is not None:
            log(
                f"UGV GPS received: lat={status['lat']:.7f}, lon={status['lon']:.7f}, "
                f"heading={status['heading']:.1f}"
            )
            return status
    log("Timeout waiting for UGV GPS_STATUS")
    return None


def move_ugv_forward_and_right(bridge):
    """Send the UGV forward 5 m, then 5 m to its right, and return final GPS status."""
    status = wait_for_ugv_status(bridge)
    if status is None:
        return None

    log(f"Commanding UGV forward {UGV_MOVE_FORWARD_M:.1f} m")
    if not bridge.send_forward_command_until_done(
        UGV_MOVE_FORWARD_M,
        timeout_s=UGV_COMMAND_TIMEOUT_S,
    ):
        log("UGV forward command failed or timed out")
        return None

    status = wait_for_ugv_status(bridge)
    if status is None:
        return None

    right_bearing_deg = (status["heading"] + 90.0) % 360.0
    target_lat, target_lon = offset_by_bearing(
        status["lat"],
        status["lon"],
        right_bearing_deg,
        UGV_MOVE_RIGHT_M,
    )

    log(
        f"Commanding UGV right {UGV_MOVE_RIGHT_M:.1f} m "
        f"-> ({target_lat:.7f}, {target_lon:.7f})"
    )
    if not bridge.send_gps_command_until_done(
        target_lat,
        target_lon,
        timeout_s=UGV_COMMAND_TIMEOUT_S,
    ):
        log("UGV right-move GPS command failed or timed out")
        return None

    return wait_for_ugv_status(bridge)


# MAIN 
def main():
    master = connect()
    bridge = None

    try:
        if not wait_for_gps(master):
            return

        if not wait_for_position(master):
            return

        if not set_mode(master, "GUIDED"):
            log("Mode change failed — aborting")
            return

        if not arm(master):
            return

        time.sleep(2)  # stabilization delay

        if not takeoff(master, TARGET_ALT_M):
            log("Takeoff failed — landing")
            land(master)
            return

        # Mission
        # fly_north(master, 5)       
        # time.sleep(4)
        # fly_west(master,  5)       
        # time.sleep(4)
        # fly_south(master, 5)        
        # time.sleep(4)
        # fly_east(master,  5)       # back to start

        # fly_bearing(master, 45, 7)  # 7 m north-east
        # goto_offset(master, north_m=3, east_m=-2, alt_m=5)  # custom offset + alt
        bridge = connect_bridge()
        bridge.clear_buffers()

        ugv_status = move_ugv_forward_and_right(bridge)
        if ugv_status is not None:
            target_lat = ugv_status["lat"]
            target_lon = ugv_status["lon"]
            log(f"Flying to UGV location: ({target_lat:.7f}, {target_lon:.7f})")
            time.sleep(2)  
            send_waypoint(master, target_lat, target_lon, TARGET_ALT_M)
            wait_until_reached(master, target_lat, target_lon)
        else:
            log("UGV move sequence failed — hovering in place")

        log(f"Hovering for {HOVER_TIMEOUT_S} seconds")
        time.sleep(HOVER_TIMEOUT_S)

        land(master)

    except KeyboardInterrupt:
        log("Emergency interrupt — landing")
        land(master)
    finally:
        if bridge is not None:
            try:
                bridge.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()

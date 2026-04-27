from pymavlink import mavutil
import time

# ─── CONNECTION ─────────────────────────────────────────────
CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE = 57600

# ─── LOGGING ────────────────────────────────────────────────
def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")

# ─── MAVLINK SETUP ──────────────────────────────────────────
def connect():
    log(f"Connecting to {CONNECTION_STRING}")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    log("Heartbeat received")
    
    # Request data streams so we receive mission and position updates
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1
    )
    return master

# ─── PRE-FLIGHT CHECKS ──────────────────────────────────────
def wait_for_gps(master, timeout=60):
    log("Waiting for strong GPS fix (required for AUTO and Geofence)...")
    start = time.time()
    
    while time.time() - start < timeout:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if msg and msg.fix_type >= 3 and msg.satellites_visible >= 8:
            log(f"GPS OK (fix={msg.fix_type}, sats={msg.satellites_visible})")
            return True
            
    log("GPS timeout! Cannot safely arm.")
    return False

def wait_for_position(master, timeout=60):
    log("Waiting for EKF position estimate...")
    start = time.time()
    
    while time.time() - start < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            log("Position estimate acquired")
            return True
            
    log("Position timeout!")
    return False

# ─── FLIGHT CONTROL ─────────────────────────────────────────
def set_mode(master, mode):
    mapping = master.mode_mapping()
    if mode not in mapping:
        log(f"Mode {mode} not available.")
        return False
        
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    log(f"Requested Mode -> {mode}")
    
    deadline = time.time() + 5
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and mavutil.mode_string_v10(hb) == mode:
            log(f"Confirmed mode: {mode}")
            return True
            
    log(f"Warning: Mode change to {mode} not confirmed")
    return False

def arm(master):
    log("Arming...")
    master.arducopter_arm()
    
    deadline = time.time() + 15
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log("Armed successfully")
            return True
            
    log("Failed to arm. Check Mission Planner for Pre-Arm errors.")
    return False

# ─── MISSION MONITORING ─────────────────────────────────────
def get_mission_count(master):
    master.mav.mission_request_list_send(
        master.target_system,
        master.target_component
    )
    msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if msg:
        log(f"Mission has {msg.count} waypoints (including Home at 0)")
        return msg.count
    log("Could not get mission count")
    return None

def monitor_mission(master):
    log("Monitoring mission progress...")
    last_seq = -1

    # Get total waypoint count so we know when the last one is reached
    mission_count = get_mission_count(master)
    last_wp_index = (mission_count - 1) if mission_count else None

    while True:
        # Track current waypoint
        msg = master.recv_match(type='MISSION_CURRENT', blocking=False)
        if msg and msg.seq != last_seq:
            log(f"Navigating to waypoint: {msg.seq}")
            last_seq = msg.seq

        # Check if a waypoint was reached
        reached = master.recv_match(type='MISSION_ITEM_REACHED', blocking=False)
        if reached:
            log(f"Reached waypoint: {reached.seq}")
            if last_wp_index and reached.seq >= last_wp_index:
                log("Final waypoint reached — mission complete")
                break

        # Fallback: disarm detection for RTL/auto-land endings
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb:
            armed = hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:
                log("Mission complete (drone disarmed automatically)")
                break

# ─── MAIN ───────────────────────────────────────────────────
def main():
    master = connect()

    try:
        # 1. Wait for safety checks to pass
        if not wait_for_gps(master):
            return
        if not wait_for_position(master):
            return

        # 2. Switch to AUTO mode (Requires Waypoint #1 to be TAKEOFF)
        if not set_mode(master, "AUTO"):
            return

        # 3. Attempt to arm
        if not arm(master):
            return
        
        time.sleep(2)

        log("Mission started")

        # 4. Track progress until mission end or disarm
        monitor_mission(master)

    except KeyboardInterrupt:
        # Ctrl+C forces the drone into LAND mode
        log("Emergency interrupt — forcing LAND mode")
        set_mode(master, "LAND")

if __name__ == "__main__":
    main()

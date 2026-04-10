from pymavlink import mavutil  # confirmed mavlink pattern
import time                    # for timing and sleeps
import math                    # for distance calcs
import cv2                     # for aruco detection
import numpy as np             # for frame math
import v2v_bridge              # our radio bridge to the ugv

# challenge 2 - uav scouts field, finds destination aruco, sends coords to ugv,
# then tracks and lands on ugv while it drives to the destination
# camera is FRONT mounted - image setpoint is bottom-center not image center
# field is 15x15 yards, uav starts in the BOTTOM RIGHT corner on top of ugv

################################# config constants

CONNECTION_STRING  = "/dev/ttyACM0"   # flight controller usb wire
BAUD_RATE          = 57600            # confirmed baud
ESP32_PORT         = "/dev/ttyUSB0"   # radio bridge wire

LOG_FILE           = "mission2_log.txt"

# altitude
TARGET_ALT         = 1.5     # meters to hover and search at
LAND_DESCENT_ALT   = 0.6     # meters: below this switch to pure descent logic

# aruco config
ARUCO_DICT         = cv2.aruco.DICT_6X6_1000
DEST_MARKER_ID     = 0        # the destination marker judges place on the field
UGV_MARKER_ID      = 2        # the aruco marker on top of the ugv

# front-camera image setpoint - aim for bottom center of frame not image center
FRAME_W            = 640      # camera resolution width
FRAME_H            = 480      # camera resolution height
TARGET_BOTTOM_RATIO = 0.90    # target is 90% down the frame
TARGET_PX_X        = FRAME_W / 2.0          # horizontal: true center is fine
TARGET_PX_Y        = FRAME_H * TARGET_BOTTOM_RATIO  # vertical: near the bottom

# centering control gains (pixels to m/s, tuned for slow approach)
GAIN_HORIZ         = 0.0012   # gain for horizontal pixel error -> lateral velocity
GAIN_VERT          = 0.0010   # gain for vertical pixel error -> forward velocity
DEADBAND_HORIZ_PX  = 18       # pixels: horizontal error smaller than this = centered
DEADBAND_VERT_PX   = 18       # pixels: vertical error smaller than this = centered
CENTER_STABLE_TIME = 2.0      # seconds marker must be stable before saving position

# snake search config (field is 15x15 yards = ~13.7m x 13.7m)
SEARCH_SPEED       = 0.4      # m/s forward sweep speed
LANE_SPACING       = 2.5      # meters between snake rows
FORWARD_STEP       = 1.5      # meters: initial step into the field before sweeping
SEARCH_ROW_WIDTH   = 12.0     # meters: total sweep width per row
SEARCH_ROWS        = 5        # number of rows to sweep before giving up

# return to home / position control
HOME_SPEED         = 0.6      # m/s toward home position
HOME_ARRIVE_DIST   = 0.5      # meters: close enough to consider home reached
POSITION_KP        = 0.5      # p gain for local position error -> velocity

# moving ugv landing bias
FORWARD_LEAD_M     = 0.25     # meters: stay this far ahead of ugv center while landing
LAND_GAIN_HORIZ    = 0.0009   # gentler gain during descent
LAND_GAIN_VERT     = 0.0008   # gentler forward gain during descent
LAND_MAX_V         = 0.25     # m/s max velocity during descent
DESCENT_RATE       = 0.15     # m/s descent speed (set via velocity z)

# timing and safety
MISSION_TIMEOUT    = 1000    # 6 min hard limit (challenge 2 is 10 min but leave margin)
MARKER_LOST_TIMEOUT = 3.0     # seconds ugv marker can be gone before fallback
CMD_SEQ_START      = 200      # starting sequence number for bridge commands

# field entry: uav starts bottom-right corner, field is NORTH of start
# NED convention: north = +x, east = +y
# so "into the field" means negative x (south to north) and slightly left (west, -y)
FIELD_ENTRY_X      = -2.0     # meters north to step into the field before searching
FIELD_ENTRY_Y      =  0.0     # no lateral shift on entry


############################ logging

def log_event(text):
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}\n"
    print(line.strip())
    with open(LOG_FILE, "a") as f:
        f.write(line)


############################ mavlink helpers (same style as mission 4)

def change_mode(master, mode: str):
    # switches the flight controller flight mode
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"Unknown mode '{mode}'")
        return
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set: {mode}")
    time.sleep(1)


def arm_drone(master):
    # arms motors via mavlink
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Arming motors...")
    time.sleep(2)


def disarm_drone(master):
    # kills the motors
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Disarmed.")


def request_message_streams(master):
    # asks the flight controller to start streaming position and sensor data
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,  # 10 hz
        1    # start
    )
    print("Message streams requested.")
    time.sleep(0.5)


def get_altitude_m(master):
    # gets current altitude from GLOBAL_POSITION_INT (relative alt in mm)
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
    if msg:
        return msg.relative_alt / 1000.0  # mm to meters
    return 0.0


def print_altitude(master, label="Alt"):
    alt = get_altitude_m(master)
    print(f"  [{label}] Altitude: {alt:.2f}m")
    return alt


def get_local_xy(master):
    # returns current local NED x,y position from LOCAL_POSITION_NED
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1.0)
    if msg:
        return msg.x, msg.y
    return None, None


def move_by_velocity(master, vx, vy, vz, duration_s):
    # sends SET_POSITION_TARGET_LOCAL_NED in velocity mode (requires GUIDED mode)
    # vx = north m/s, vy = east m/s, vz = down m/s (NED, so positive vz = descend)
    # type_mask: ignore position and acceleration, use velocity only = 0b0000111111000111 = 0x0FC7
    type_mask = 0b0000111111000111
    deadline  = time.time() + duration_s
    while time.time() < deadline:
        master.mav.set_position_target_local_ned_send(
            0,                                              # time_boot_ms
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,                                       # x,y,z position (ignored)
            vx, vy, vz,                                    # velocity
            0, 0, 0,                                       # acceleration (ignored)
            0, 0                                           # yaw, yaw_rate (ignored)
        )
        time.sleep(0.05)  # 20hz command loop


def move_to_local_xy(master, target_x, target_y, target_z_ned, speed=HOME_SPEED, arrive_dist=HOME_ARRIVE_DIST):
    # flies to a local NED x,y coordinate using proportional velocity control
    # target_z_ned is altitude in NED (negative = up because NED convention)
    print(f"  [Nav] Flying to local x:{target_x:.2f} y:{target_y:.2f}")
    while True:
        cx, cy = get_local_xy(master)
        if cx is None:
            time.sleep(0.1)
            continue

        err_x = target_x - cx
        err_y = target_y - cy
        dist  = math.sqrt(err_x**2 + err_y**2)

        print(f"  [Nav] Pos: x={cx:.2f} y={cy:.2f}  dist_to_target={dist:.2f}m")

        if dist < arrive_dist:
            print(f"  [Nav] Arrived at target. dist={dist:.2f}m")
            # stop cleanly
            move_by_velocity(master, 0, 0, 0, 0.3)
            break

        # proportional velocity capped at speed
        scale = min(speed / dist, speed) if dist > 0 else 0
        vx    = err_x * POSITION_KP * scale
        vy    = err_y * POSITION_KP * scale
        # clamp to max speed
        mag   = math.sqrt(vx**2 + vy**2)
        if mag > speed:
            vx = vx / mag * speed
            vy = vy / mag * speed

        move_by_velocity(master, vx, vy, 0, 0.15)


############################ aruco detection helpers

def get_aruco_detector():
    # builds and returns the aruco detector object
    aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    return detector


def detect_aruco_markers(frame, detector):
    # detects all aruco markers in frame and returns dict of {id: center_xy}
    gray               = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _    = detector.detectMarkers(gray)
    found              = {}
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            cx = float(np.mean(corners[i][0][:, 0]))  # pixel x center
            cy = float(np.mean(corners[i][0][:, 1]))  # pixel y center
            found[int(marker_id)] = (cx, cy)
    return found


def pixel_error_to_target(marker_cx, marker_cy):
    # returns pixel error from marker center to the bottom-center setpoint
    # positive err_x = marker is to the right of target (fly right)
    # positive err_y = marker is ABOVE the target row (fly forward/down frame)
    err_x = marker_cx - TARGET_PX_X
    err_y = marker_cy - TARGET_PX_Y
    return err_x, err_y


############################ mission phase functions

def takeoff_to_altitude(master):
    # arms, switches to guided, and climbs to TARGET_ALT
    log_event("=== PHASE: ARMING AND TAKEOFF ===")
    change_mode(master, "GUIDED")
    arm_drone(master)
    log_event(f"Climbing to {TARGET_ALT}m...")

    # send takeoff command via mavlink
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,   # params 1-4 unused
        0, 0,          # lat, lon (use current)
        TARGET_ALT     # altitude in meters
    )

    # wait until we reach altitude
    start = time.time()
    while time.time() - start < 30:
        alt = print_altitude(master, "Takeoff")
        if alt >= TARGET_ALT * 0.92:  # 92% close enough
            log_event(f"Takeoff altitude reached: {alt:.2f}m")
            break
        time.sleep(0.5)

    time.sleep(1.0)  # settle


def save_home_position(master):
    # saves the uav local x,y right after takeoff so we can return later
    log_event("Saving home (start/UGV) position...")
    for _ in range(5):  # try a few times to get a good reading
        hx, hy = get_local_xy(master)
        if hx is not None:
            log_event(f"Home saved: x={hx:.2f} y={hy:.2f}")
            return hx, hy
        time.sleep(0.3)
    log_event("[!] Could not get home position from local NED.")
    return 0.0, 0.0  # fallback to origin, not ideal but safe


def snake_search_for_destination(master, cam, detector):
    # performs a snake/lawnmower pattern looking for DEST_MARKER_ID
    # returns (found, dest_cx_px, dest_cy_px) where found is a bool
    # and dest coords are pixel positions if found
    log_event("=== PHASE: SNAKE SEARCH FOR DESTINATION MARKER ===")
    log_event(f"Searching for ArUco ID:{DEST_MARKER_ID} across field...")

    # first step forward into the field before starting sweeps
    log_event(f"Stepping {FIELD_ENTRY_X:.1f}m into field...")
    move_by_velocity(master, abs(FIELD_ENTRY_X) * SEARCH_SPEED, 0, 0,
                     abs(FIELD_ENTRY_X) / SEARCH_SPEED)

    # snake sweep: alternate left and right across the field
    direction = 1  # 1 = sweep east (+y), -1 = sweep west (-y)
    for row in range(SEARCH_ROWS):
        log_event(f"Search row {row+1}/{SEARCH_ROWS} direction={'EAST' if direction>0 else 'WEST'}")
        sweep_time  = SEARCH_ROW_WIDTH / SEARCH_SPEED
        sweep_start = time.time()

        while time.time() - sweep_start < sweep_time:
            ret, frame = cam.read()
            if not ret or frame is None:
                time.sleep(0.02)
                continue

            markers = detect_aruco_markers(frame, detector)
            if DEST_MARKER_ID in markers:
                cx, cy = markers[DEST_MARKER_ID]
                log_event(f"DESTINATION MARKER FOUND! ID:{DEST_MARKER_ID} at px ({cx:.0f},{cy:.0f})")
                # stop movement immediately
                move_by_velocity(master, 0, 0, 0, 0.3)
                return True, cx, cy

            # print altitude every sweep leg
            alt = get_altitude_m(master)
            print(f"  [Search] Row {row+1} | Alt:{alt:.2f}m | sweeping...", end="\r", flush=True)

            # fly sideways across the row
            move_by_velocity(master, 0, direction * SEARCH_SPEED, 0, 0.1)

        # end of row: advance one lane forward then flip direction
        print()  # newline after the \r status line
        log_event(f"Advancing {LANE_SPACING:.1f}m to next lane...")
        move_by_velocity(master, SEARCH_SPEED, 0, 0, LANE_SPACING / SEARCH_SPEED)
        direction *= -1  # flip sweep direction

    # never found it
    log_event("[!] Destination marker NOT found after full search.")
    move_by_velocity(master, 0, 0, 0, 0.3)  # stop
    return False, 0.0, 0.0


def center_marker_bottom_frame(master, cam, detector, target_id, stable_time=CENTER_STABLE_TIME):
    # hovers and nudges using image error until the marker is stably centered
    # at the bottom-center setpoint for stable_time seconds
    # returns True if stable, False if timed out
    log_event(f"Centering marker ID:{target_id} at bottom-center setpoint...")
    stable_start  = None
    timeout_start = time.time()

    while True:
        if time.time() - timeout_start > 20.0:  # 20 second centering timeout
            log_event("[!] Centering timeout.")
            return False

        ret, frame = cam.read()
        if not ret or frame is None:
            time.sleep(0.05)
            continue

        markers = detect_aruco_markers(frame, detector)
        if target_id not in markers:
            # lost it - hold still and try again
            move_by_velocity(master, 0, 0, 0, 0.1)
            stable_start = None
            continue

        cx, cy      = markers[target_id]
        err_x, err_y = pixel_error_to_target(cx, cy)

        print(f"  [Center ID:{target_id}] px_err_x:{err_x:.1f} px_err_y:{err_y:.1f}")

        # check if we are inside the deadband
        in_x = abs(err_x) < DEADBAND_HORIZ_PX
        in_y = abs(err_y) < DEADBAND_VERT_PX

        if in_x and in_y:
            if stable_start is None:
                stable_start = time.time()
                log_event("Inside deadband. Waiting for stable hold...")
            elif time.time() - stable_start >= stable_time:
                log_event(f"Stable for {stable_time}s. Centered.")
                move_by_velocity(master, 0, 0, 0, 0.2)
                return True
        else:
            stable_start = None

        # proportional correction: err_x -> lateral (vy), err_y -> forward (vx)
        # for front camera: positive err_y means marker is above setpoint row = ugv is further ahead
        # so we need to fly forward (positive vx in NED)
        vy_cmd = clamp(err_x * GAIN_HORIZ, -0.3, 0.3)
        vx_cmd = clamp(err_y * GAIN_VERT,  -0.3, 0.3)

        move_by_velocity(master, vx_cmd, vy_cmd, 0, 0.1)


def save_destination_position(master, samples=5):
    # averages a few local position readings for a better dest x,y estimate
    log_event("Saving destination x,y from local position...")
    xs, ys = [], []
    for _ in range(samples):
        x, y = get_local_xy(master)
        if x is not None:
            xs.append(x)
            ys.append(y)
        time.sleep(0.2)
    if not xs:
        log_event("[!] Could not read local position for destination.")
        return None, None
    dest_x = sum(xs) / len(xs)
    dest_y = sum(ys) / len(ys)
    log_event(f"Destination position saved: x={dest_x:.2f} y={dest_y:.2f}")
    return dest_x, dest_y


def return_to_home(master, home_x, home_y):
    # flies back to the saved home/ugv position
    log_event("=== PHASE: RETURNING TO HOME/UGV POSITION ===")
    log_event(f"Target: x={home_x:.2f} y={home_y:.2f}")
    alt_ned = -TARGET_ALT  # NED: altitude is negative
    move_to_local_xy(master, home_x, home_y, alt_ned, speed=HOME_SPEED)
    log_event("Returned to home position.")
    time.sleep(1.0)  # hover briefly before looking for ugv marker


def reacquire_ugv_marker(master, cam, detector):
    # scans around the home position until the ugv aruco marker is visible
    # returns (cx_px, cy_px) or None if timeout
    log_event("=== PHASE: REACQUIRING UGV MARKER ===")
    timeout = time.time() + 15.0
    while time.time() < timeout:
        ret, frame = cam.read()
        if not ret or frame is None:
            time.sleep(0.05)
            continue

        markers = detect_aruco_markers(frame, detector)
        if UGV_MARKER_ID in markers:
            cx, cy = markers[UGV_MARKER_ID]
            log_event(f"UGV marker reacquired at px ({cx:.0f},{cy:.0f})")
            return cx, cy

        alt = get_altitude_m(master)
        print(f"  [Reacquire] Scanning for UGV ID:{UGV_MARKER_ID}  Alt:{alt:.2f}m", end="\r", flush=True)
        time.sleep(0.1)

    print()
    log_event("[!] Could not reacquire UGV marker.")
    return None


def send_destination_to_ugv(bridge, dest_x, dest_y, cmd_seq):
    # ---- challenge 2 bridge command wrapper ----
    # this calls the existing CMD_MISSION_2 command which is the challenge 2 trigger
    # the ugv ground station interprets this as: start driving toward the destination
    # the actual x,y are packed into the telemetry stream as a separate update
    # (the real protocol sends a dedicated challenge 2 position packet - call that here)
    log_event(f"=== PHASE: SENDING DESTINATION TO UGV ===")
    log_event(f"Destination: x={dest_x:.2f} y={dest_y:.2f}")

    # ---- CHALLENGE 2 DESTINATION BRIDGE CALL ----
    # replace this with the actual challenge 2 position send when protocol is ready
    # for now: fire CMD_MISSION_2 which tells the ugv to enter challenge 2 mode
    # and separately call the position wrapper below
    bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_MISSION_2, estop=0)
    bridge.send_message(f"C2 DEST x={dest_x:.2f} y={dest_y:.2f}")
    # ---- END CHALLENGE 2 BRIDGE CALL ----

    log_event("Destination sent to UGV via bridge. UGV should start moving.")
    time.sleep(0.5)


def track_and_land_on_moving_ugv(master, cam, detector):
    # tracks the moving ugv and lands on it while it drives to destination
    # uses bottom-center setpoint + forward lead so uav stays slightly ahead
    log_event("=== PHASE: MOVING LANDING ON UGV ===")

    last_seen_time  = time.time()
    last_vx         = 0.0
    last_vy         = 0.0
    landing_started = False
    on_ground_count = 0

    mission_start = time.time()

    while True:
        # overall landing timeout fallback
        if time.time() - mission_start > 120.0:
            log_event("[!] Moving landing timeout. Switching to LAND mode.")
            change_mode(master, "LAND")
            break

        alt   = get_altitude_m(master)
        ret, frame = cam.read()

        if not ret or frame is None:
            time.sleep(0.02)
            continue

        markers       = detect_aruco_markers(frame, detector)
        ugv_visible   = UGV_MARKER_ID in markers

        if ugv_visible:
            cx, cy        = markers[UGV_MARKER_ID]
            last_seen_time = time.time()
            err_x, err_y  = pixel_error_to_target(cx, cy)

            print(f"  [Land] Alt:{alt:.2f}m | px_err_x:{err_x:.1f} err_y:{err_y:.1f}")

            # forward lead: shift the target setpoint up slightly in the image
            # so the drone stays a bit ahead of the ugv marker center
            # a higher py target means we want the marker lower in frame = ugv is ahead = ok
            lead_px       = FORWARD_LEAD_M / 0.005  # rough px per meter at low alt (tunable)
            effective_err_y = err_y - lead_px  # pull target forward

            vy_cmd = clamp(err_x * LAND_GAIN_HORIZ, -LAND_MAX_V, LAND_MAX_V)
            vx_cmd = clamp(effective_err_y * LAND_GAIN_VERT, -LAND_MAX_V, LAND_MAX_V)

            last_vx = vx_cmd
            last_vy = vy_cmd

            # descend based on altitude
            if alt > LAND_DESCENT_ALT:
                vz_cmd = DESCENT_RATE  # positive z = down in NED
            else:
                vz_cmd = DESCENT_RATE * 0.5  # slower near the ground
                if not landing_started:
                    log_event(f"Below {LAND_DESCENT_ALT}m. Slow descent phase started.")
                    landing_started = True

            move_by_velocity(master, vx_cmd, vy_cmd, vz_cmd, 0.08)

        else:
            # ugv marker not visible
            time_lost = time.time() - last_seen_time
            print(f"  [Land] Alt:{alt:.2f}m | UGV LOST for {time_lost:.1f}s")

            if time_lost < MARKER_LOST_TIMEOUT:
                # hold last known velocity and keep descending - uav is probably still on track
                vz_cmd = DESCENT_RATE if alt > LAND_DESCENT_ALT else DESCENT_RATE * 0.5
                move_by_velocity(master, last_vx, last_vy, vz_cmd, 0.08)
            elif alt < 0.5:
                # very low altitude and lost - just land, close enough
                log_event("[!] Low altitude + marker lost. Committing to land.")
                change_mode(master, "LAND")
                break
            else:
                # higher altitude and lost - hover and wait for marker to come back
                move_by_velocity(master, 0, 0, 0, 0.1)

        # touchdown detection: altitude is near floor
        if alt < 0.15 and alt > 0.01:
            on_ground_count += 1
            if on_ground_count >= 3:
                log_event("TOUCHDOWN CONFIRMED. UAV on UGV.")
                move_by_velocity(master, 0, 0, 0, 0.2)  # stop all velocity
                break
        else:
            on_ground_count = 0

        time.sleep(0.02)


############################ small utility

def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def open_camera(index=0):
    # opens the front camera
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    print(f"Camera opened: index={index} {FRAME_W}x{FRAME_H}")
    return cap


def safe_land_and_exit(master):
    # emergency fallback - release overrides and switch to land
    try:
        change_mode(master, "LAND")
        # release any rc overrides
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )
    except Exception:
        pass


#################### main mission logic

def main():
    log_event("==========================================")
    log_event("   UAV CHALLENGE 2 - SCOUT + UGV ESCORT  ")
    log_event("==========================================")

    # --- connect to flight controller ---
    log_event(f"Connecting to FC: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    log_event("Drone Heartbeat OK.")

    request_message_streams(master)

    # --- connect to ugv radio bridge ---
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        bridge.send_message("CHALLENGE 2: UAV ONLINE")
        log_event("V2V Bridge connected.")
    except Exception as e:
        log_event(f"[!] Radio bridge fail: {e}. Aborting.")
        return

    # --- open front camera ---
    cam      = open_camera(0)
    detector = get_aruco_detector()

    # pre-flight: make sure ugv is stopped while we take off
    bridge.send_command(cmdSeq=CMD_SEQ_START, cmd=v2v_bridge.CMD_STOP, estop=0)

    cmd_seq      = CMD_SEQ_START + 1
    dest_x       = None
    dest_y       = None
    home_x       = 0.0
    home_y       = 0.0
    mission_ok   = True

    try:
        # ===== PHASE 1: TAKEOFF =====
        takeoff_to_altitude(master)

        # save home position right after reaching altitude
        home_x, home_y = save_home_position(master)
        log_event(f"Home/UGV position: x={home_x:.2f} y={home_y:.2f}")

        # ===== PHASE 2: SNAKE SEARCH =====
        mission_start = time.time()
        found, dest_cx, dest_cy = snake_search_for_destination(master, cam, detector)

        if not found:
            log_event("[!] Destination not found. Returning home and landing.")
            mission_ok = False
        else:
            log_event("Destination marker found. Centering above it...")

            # ===== PHASE 3: CENTER OVER DESTINATION =====
            centered = center_marker_bottom_frame(master, cam, detector, DEST_MARKER_ID)
            if not centered:
                log_event("[!] Could not center over destination. Aborting.")
                mission_ok = False

        if mission_ok:
            # ===== PHASE 3b: SAVE DESTINATION X,Y =====
            dest_x, dest_y = save_destination_position(master)
            if dest_x is None:
                log_event("[!] Position save failed. Aborting.")
                mission_ok = False

        if mission_ok:
            # ===== PHASE 4: RETURN TO HOME/UGV =====
            return_to_home(master, home_x, home_y)
            log_event("Back at home position. Hovering above UGV...")

            # ===== PHASE 5: REACQUIRE UGV =====
            ugv_result = reacquire_ugv_marker(master, cam, detector)
            if ugv_result is None:
                log_event("[!] Could not reacquire UGV marker. Landing at home.")
                mission_ok = False

        if mission_ok:
            # center on ugv briefly before sending the command
            log_event("Centering above UGV before sending destination...")
            center_marker_bottom_frame(master, cam, detector, UGV_MARKER_ID,
                                       stable_time=1.5)

            # ===== PHASE 6: SEND DESTINATION TO UGV =====
            send_destination_to_ugv(bridge, dest_x, dest_y, cmd_seq)
            cmd_seq += 1

            log_event("UGV now moving toward destination. Starting moving landing...")
            time.sleep(1.0)  # brief pause to let ugv start rolling

            # ===== PHASE 7: TRACK AND LAND ON MOVING UGV =====
            track_and_land_on_moving_ugv(master, cam, detector)

        # if anything failed, land where we are
        if not mission_ok:
            log_event("Mission aborted. Switching to LAND mode.")
            change_mode(master, "LAND")

        # wait for motors to stop after landing
        log_event("Waiting for autopilot landing confirmation...")
        while True:
            msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                log_event("Motors stopped. Landing confirmed.")
                break
            alt = get_altitude_m(master)
            print(f"  [Land] Alt: {alt:.2f}m", end="\r", flush=True)
            time.sleep(0.5)

    except KeyboardInterrupt:
        log_event("\n[!] Emergency: Ctrl+C. Switching to LAND.")
        safe_land_and_exit(master)
        time.sleep(1)

    except Exception as e:
        log_event(f"\n[!] Unexpected error: {e}. Switching to LAND.")
        safe_land_and_exit(master)
        time.sleep(1)

    finally:
        try:
            bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_STOP, estop=0)
            bridge.stop()
        except Exception:
            pass
        try:
            cam.release()
        except Exception:
            pass
        log_event("Challenge 2 mission finalized.")


if __name__ == "__main__":
    main()
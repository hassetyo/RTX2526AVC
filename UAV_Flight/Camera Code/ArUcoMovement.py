from pymavlink import mavutil  # using the confirmed mavlink pattern instead of dronekit
import time                    # for timing and sleeps
import sys                     # for clean exits
import math                    # for angle math
import numpy as np             # for vector ops
import cv2                     # for aruco detection
import v2v_bridge              # our custom radio bridge talker

# import the camera and estimator classes from the shared aruco utility
from ArUcoDistance import CameraInterface, ArucoDistanceEstimator

# -----------------------------------------------------------------------
# UGV VISUAL NAVIGATOR - UAV SIDE
# -----------------------------------------------------------------------
# The drone hovers and uses its ZED 2 camera to see both:
#   - Marker ID 5  : the ArUco tag mounted flat on top of the UGV,
#                    with its local +Y axis aligned with the UGV's
#                    physical forward direction.
#   - Marker ID 0  : the stationary destination marker on the ground.
#
# Navigation logic (closed-loop visual servo):
#   PHASE 1 - ALIGN:
#       Compute the heading error between the UGV's current forward
#       direction and the bearing toward the destination marker.
#       Send CMD_TURN_ANGLE_RIGHT or CMD_TURN_ANGLE_LEFT to correct.
#       Re-check with the camera; repeat until within ANGLE_DEADBAND.
#
#   PHASE 2 - APPROACH:
#       Compute the 3-D distance between the two markers' tvecs.
#       Send CMD_MOVE_DIST to drive that distance.
#       Re-check; repeat until within ARRIVAL_DIST_M.
#
# Coordinate system notes (important for the angle maths):
#   The ZED 2 is assumed to be mounted on the drone looking DOWNWARD
#   (nadir / near-nadir).  In standard OpenCV camera convention:
#       Camera +X  →  right   (when looking down: drone right side)
#       Camera +Y  →  down    (into the ground; depth axis is Z)
#       Camera +Z  →  forward (optical axis pointing down toward ground)
#   So the GROUND PLANE is spanned by Camera X and Camera Z.
#   We use the (x, z) components of tvec as the 2-D ground position,
#   and the (x, z) projection of R @ [0,1,0] as the UGV forward vector.
#   If your camera is mounted differently, adjust the axis selection in
#   ugv_forward_2d() and dest_vector_2d().
# -----------------------------------------------------------------------

################################# config stuff i setup

# connection settings from your working test script
CONNECTION_STRING = "/dev/ttyACM0"  # drone wire (use COM4 if testing on windows)
BAUD_RATE         = 57600           # using the confirmed 57600 speed
ESP32_PORT        = "/dev/ttyUSB0"  # the radio bridge usb wire

# aruco marker ids
UGV_MARKER_ID  = 5  # the tag mounted flat on top of the UGV, +Y = UGV forward
DEST_MARKER_ID = 0  # the stationary target marker on the ground

# aruco detection settings
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50  # marker dictionary (match your printed markers)
MARKER_SIZE_M   = 0.254                  # physical side length of each printed marker, metres

# navigation thresholds
ANGLE_DEADBAND_DEG = 5.0   # stop turning when heading error is within this many degrees
ARRIVAL_DIST_M     = 0.20  # consider UGV "arrived" when markers are this close (metres)
MAX_SCAN_TIMEOUT   = 10.0  # max seconds to wait for both markers to appear in frame

# turn and drive speed assumptions (must match ground_station.py)
UGV_YAW_RATE_DEG_S = 60.0  # degrees per second (from ground_station execute_turn)
UGV_DRIVE_SPEED_MPS = 1.5  # metres per second (from ground_station SPEED_MPS)

# safety buffer added on top of the calculated manoeuvre time before re-checking
MANOEUVRE_BUFFER_S = 0.75  # extra seconds to let the UGV fully settle after each move

# throttle settings used while the drone is in navigator mode (holding altitude)
THROTTLE_HOVER = 1500  # rough middle ground for holding height - from working mission scripts

# log file path
LOG_FILE = "ugv_navigator_log.txt"

############################ logging helper

def log_event(text):  # writes timestamped entries to console and the log file
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}\n"
    print(line.strip())
    with open(LOG_FILE, "a") as f:
        f.write(line)

############################ mavlink helpers (same pattern as confirmed working scripts)

def change_mode(master, mode: str):  # changes the flight controller mode
    mapping = master.mode_mapping()  # ask for the list of modes
    if mode not in mapping:          # if the mode is fake
        log_event(f"Unknown mode '{mode}'")  # log the error
        return                               # bail out
    mode_id = mapping[mode]          # find the secret mode id
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )                                        # blast it
    log_event(f"Mode set: {mode}")           # log the change
    time.sleep(1)                            # wait for the mode to settle

def set_throttle(master, pwm):  # physically pushes the throttle via rc override
    # channel 3 is the throttle in ardupilot
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )

def disarm_drone(master):  # stops the motors securely
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    log_event("Disarmed.")  # log the safety

############################ camera & aruco helpers

def open_camera():  # opens the ZED 2 camera using the shared CameraInterface class
    log_event("[Camera] Opening ZED 2 via CameraInterface...")
    cam = CameraInterface(
        use_zed=True,   # use the ZED 2 SDK on the Jetson Nano
        fps=30          # 30fps is smooth enough for this nav loop
    )
    log_event("[Camera] ZED 2 open OK.")
    return cam

def build_estimator(cam):  # builds the aruco pose estimator using the camera's own intrinsics
    return ArucoDistanceEstimator(
        camera_matrix=cam.camera_matrix,    # pulled straight from the ZED SDK calibration
        dist_coeffs=cam.dist_coeffs,        # ZED SDK distortion coefficients
        marker_size_m=MARKER_SIZE_M,        # physical marker size for pose scaling
        dictionary_name=ARUCO_DICT_TYPE     # must match the dictionary used to print the markers
    )

def scan_for_both_markers(cam, estimator):
    """
    Grabs frames until both UGV_MARKER_ID and DEST_MARKER_ID are visible,
    or MAX_SCAN_TIMEOUT seconds pass.
    Returns (ugv_pose, dest_pose) on success, or (None, None) on timeout.
    """
    scan_start = time.time()  # start the timeout clock
    while True:
        elapsed = time.time() - scan_start  # how long we have been waiting
        if elapsed >= MAX_SCAN_TIMEOUT:     # if we ran out of time
            log_event(f"[Camera] Timeout: both markers not seen in {MAX_SCAN_TIMEOUT}s")
            return None, None              # bail with no poses

        frame = cam.get_frame()            # grab a fresh frame from the ZED
        if frame is None:                  # if the camera burped
            time.sleep(0.05)               # small nap then retry
            continue

        poses = estimator.detect_markers(frame)  # run aruco detection, get all visible poses

        ugv_pose  = poses.get(UGV_MARKER_ID)   # look for the UGV marker
        dest_pose = poses.get(DEST_MARKER_ID)  # look for the destination marker

        if ugv_pose is not None and dest_pose is not None:  # if both are visible
            return ugv_pose, dest_pose                      # return the pose pair

        # log a live scan ticker so we know the loop is alive
        log_event(f"[Camera] Waiting for markers... {elapsed:.1f}s "
                  f"| UGV={ugv_pose is not None} DEST={dest_pose is not None}")
        time.sleep(0.05)  # ~20fps scan rate, gentle on the Nano

############################ 2D navigation maths

def get_ugv_forward_2d(ugv_pose):
    """
    Returns the UGV's current forward direction as a 2D unit vector [x, z]
    in camera space.

    The UGV marker (ID 5) is mounted flat on the vehicle with its local
    +Y axis aligned to the UGV's physical forward direction.
    solvePnP gives us an rvec that encodes the rotation from marker-local
    space into camera space, so R @ [0,1,0] is where the marker's +Y axis
    points in camera coordinates.  We take the (x, z) components of that
    3D vector to get the 2D ground-plane projection.
    """
    R, _ = cv2.Rodrigues(ugv_pose.rvec)       # convert rotation vector → 3x3 matrix
    marker_y_in_cam = R @ np.array([0.0, 1.0, 0.0])  # where marker +Y lands in camera frame

    # camera ground-plane = X/Z (for nadir camera); take those two components
    forward_xz = np.array([marker_y_in_cam[0], marker_y_in_cam[2]])  # [cam_x, cam_z]
    norm = np.linalg.norm(forward_xz)  # magnitude for normalisation
    if norm < 1e-6:                    # guard against a degenerate zero vector
        return np.array([0.0, 1.0])    # fall back to "looking along cam +Z"
    return forward_xz / norm           # normalised unit vector

def get_dest_vector_2d(ugv_pose, dest_pose):
    """
    Returns the direction from the UGV marker to the destination marker
    as a 2D unit vector [x, z] in camera space (ground plane).
    """
    p_ugv  = ugv_pose.tvec   # UGV marker 3D position in camera frame (metres)
    p_dest = dest_pose.tvec  # destination marker 3D position in camera frame

    diff    = p_dest - p_ugv              # 3D vector UGV → destination
    diff_xz = np.array([diff[0], diff[2]])  # project onto X/Z ground plane
    dist_2d = np.linalg.norm(diff_xz)       # 2D horizontal distance
    if dist_2d < 1e-6:                      # guard: already at destination
        return np.array([0.0, 1.0]), 0.0    # dummy direction, zero distance
    return diff_xz / dist_2d, dist_2d       # (unit direction, horizontal distance in metres)

def compute_heading_error_deg(ugv_forward_2d, dest_dir_2d):
    """
    Computes the signed heading error in degrees between the UGV's current
    forward direction and the direction it needs to face to reach the
    destination marker.

    Positive result → UGV must turn RIGHT (clockwise when viewed from above).
    Negative result → UGV must turn LEFT  (counter-clockwise).

    Uses the 2D cross product for sign and dot product for magnitude,
    same maths as atan2(sin, cos) of the angle between the two vectors.
    """
    fx, fz = ugv_forward_2d     # UGV current forward
    dx, dz = dest_dir_2d        # direction to destination

    cross = fx * dz - fz * dx   # 2D cross product (scalar; sign = rotation sense)
    dot   = fx * dx + fz * dz   # 2D dot product

    angle_rad = math.atan2(cross, dot)   # signed angle in radians
    return math.degrees(angle_rad)       # convert to degrees for readability

def get_3d_distance(ugv_pose, dest_pose):
    """
    Returns the straight-line 3D distance between the two markers in metres.
    This is the Euclidean distance of their tvec positions in camera space.
    """
    return float(np.linalg.norm(dest_pose.tvec - ugv_pose.tvec))

############################ navigation state machine

def phase_align(cam, estimator, bridge, cmd_seq_counter):
    """
    PHASE 1: Turn the UGV until it faces the destination marker.
    Returns the updated cmd_seq_counter when alignment is confirmed.
    Raises RuntimeError if alignment cannot be achieved after many attempts.
    """
    log_event("[NAV] === PHASE 1: ALIGN ===")
    attempts = 0  # track how many correction turns we have sent

    while True:  # keep correcting until we are within the deadband
        # --- sample camera for current pose of both markers ---
        ugv_pose, dest_pose = scan_for_both_markers(cam, estimator)
        if ugv_pose is None:  # failed to see both markers
            raise RuntimeError("Cannot see both markers during alignment phase.")

        # --- compute heading error ---
        ugv_fwd  = get_ugv_forward_2d(ugv_pose)           # UGV current forward unit vector
        dest_dir, dist_m = get_dest_vector_2d(ugv_pose, dest_pose)  # direction + distance
        error_deg = compute_heading_error_deg(ugv_fwd, dest_dir)     # signed error

        log_event(f"[ALIGN] Heading error: {error_deg:+.1f} deg | "
                  f"Distance: {dist_m:.3f}m | Attempt: {attempts}")

        # --- check if we are already close enough ---
        if abs(error_deg) <= ANGLE_DEADBAND_DEG:  # within the deadband - we are done
            log_event(f"[ALIGN] Heading locked. Error {error_deg:+.1f} deg is within "
                      f"{ANGLE_DEADBAND_DEG} deg deadband.")
            return cmd_seq_counter  # hand back the updated counter

        # --- choose direction and send turn command ---
        angle_to_send_deg = abs(error_deg)  # always send a positive magnitude
        cmd_seq_counter   += 1              # increment sequence so the UGV can deduplicate

        if error_deg > 0:  # positive error → UGV must rotate clockwise = RIGHT
            log_event(f"[ALIGN] Sending TURN RIGHT {angle_to_send_deg:.1f} deg  (seq={cmd_seq_counter})")
            bridge.send_command(
                cmdSeq=int(round(angle_to_send_deg)),  # cmdSeq carries the degrees for this command type
                cmd=v2v_bridge.CMD_TURN_ANGLE_RIGHT,   # new command: turn right by N degrees
                estop=0
            )
        else:              # negative error → UGV must rotate counter-clockwise = LEFT
            log_event(f"[ALIGN] Sending TURN LEFT {angle_to_send_deg:.1f} deg  (seq={cmd_seq_counter})")
            bridge.send_command(
                cmdSeq=int(round(angle_to_send_deg)),  # cmdSeq carries the degrees for this command type
                cmd=v2v_bridge.CMD_TURN_ANGLE_LEFT,    # new command: turn left by N degrees
                estop=0
            )

        # --- wait for the UGV to complete the turn ---
        turn_time = (angle_to_send_deg / UGV_YAW_RATE_DEG_S) + MANOEUVRE_BUFFER_S  # expected + buffer
        log_event(f"[ALIGN] Waiting {turn_time:.1f}s for turn to complete...")
        time.sleep(turn_time)  # give the UGV time to physically rotate

        attempts += 1  # count each correction attempt

def phase_approach(cam, estimator, bridge, cmd_seq_counter):
    """
    PHASE 2: Drive the UGV forward until the UGV marker is adjacent to the
    destination marker (within ARRIVAL_DIST_M).
    Returns the updated cmd_seq_counter when the UGV has arrived.
    Raises RuntimeError if arrival cannot be confirmed.
    """
    log_event("[NAV] === PHASE 2: APPROACH ===")
    attempts = 0  # track how many drive commands we have sent

    while True:  # keep driving until the UGV is close enough
        # --- sample camera for current pose ---
        ugv_pose, dest_pose = scan_for_both_markers(cam, estimator)
        if ugv_pose is None:  # cannot see both markers
            raise RuntimeError("Cannot see both markers during approach phase.")

        dist_m = get_3d_distance(ugv_pose, dest_pose)  # straight-line distance between markers
        log_event(f"[APPROACH] Distance: {dist_m:.3f}m | Attempt: {attempts}")

        # --- check arrival condition ---
        if dist_m <= ARRIVAL_DIST_M:  # close enough to call it done
            log_event(f"[APPROACH] ARRIVAL CONFIRMED. Distance {dist_m:.3f}m <= {ARRIVAL_DIST_M}m threshold.")
            return cmd_seq_counter  # navigation complete

        # --- send drive command for the remaining distance ---
        drive_dist_cm = int(round(dist_m * 100.0))  # convert metres to centimetres for the command
        cmd_seq_counter += 1                         # increment sequence counter

        log_event(f"[APPROACH] Sending MOVE_DIST {drive_dist_cm}cm  (seq={cmd_seq_counter})")
        bridge.send_command(
            cmdSeq=drive_dist_cm,                  # cmdSeq carries the distance in centimetres
            cmd=v2v_bridge.CMD_MOVE_DIST,          # new command: drive forward N centimetres
            estop=0
        )

        # --- wait for the UGV to drive that distance ---
        drive_time = dist_m / UGV_DRIVE_SPEED_MPS + MANOEUVRE_BUFFER_S  # expected + buffer
        log_event(f"[APPROACH] Waiting {drive_time:.1f}s for drive to complete...")
        time.sleep(drive_time)  # give the UGV time to physically move

        # re-check alignment before the next drive segment (small heading errors accumulate)
        log_event("[APPROACH] Re-checking alignment before next drive segment...")
        cmd_seq_counter = phase_align(cam, estimator, bridge, cmd_seq_counter)  # quick re-align

        attempts += 1  # count each drive attempt

############################ main entry point

def main():  # the main boss function
    log_event("==========================================")
    log_event("   UGV VISUAL NAVIGATOR - UAV SIDE       ")
    log_event(f"   UGV marker: ID {UGV_MARKER_ID}       ")
    log_event(f"   Dest marker: ID {DEST_MARKER_ID}      ")
    log_event("==========================================")

    # ------------------------------------------------------------------ #
    # STEP 1: connect to drone mavlink                                     #
    # ------------------------------------------------------------------ #
    log_event(f"Connecting to Drone: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)  # open link
    master.wait_heartbeat()  # wait for buzz
    log_event("Drone Heartbeat OK.")

    # ------------------------------------------------------------------ #
    # STEP 2: connect to the v2v radio bridge                              #
    # ------------------------------------------------------------------ #
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Navigator")  # radio bridge object
    try:
        bridge.connect()  # open serial wire to the ESP32
        bridge.send_message("UGV NAVIGATOR: RADIO LINK UP")  # hello over air
    except Exception as e:
        log_event(f"Radio Bridge Fail: {e}")
        return  # bail if no radio - cannot talk to UGV

    # ------------------------------------------------------------------ #
    # STEP 3: open ZED 2 camera and build the aruco estimator              #
    # ------------------------------------------------------------------ #
    cam = open_camera()          # opens ZED 2 using the shared CameraInterface
    estimator = build_estimator(cam)  # creates ArucoDistanceEstimator with ZED intrinsics

    cmd_seq_counter = 500  # start at 500 so it does not collide with mission 4 sequence numbers

    try:
        # -------------------------------------------------------------- #
        # STEP 4: put the drone in STABILIZE + throttle hold while we     #
        # run the navigation (drone should already be hovering, but we     #
        # explicitly enter the hover loop to keep the RC override alive).  #
        # -------------------------------------------------------------- #
        change_mode(master, "STABILIZE")  # stabilize mode for throttle override control
        log_event("Drone in STABILIZE hover mode. Starting UGV navigation loop.")
        bridge.send_message("UGV NAV: STARTING APPROACH")

        # -------------------------------------------------------------- #
        # STEP 5: wait until both markers are visible before doing anything#
        # -------------------------------------------------------------- #
        log_event("[NAV] Waiting for both markers to appear in frame...")
        ugv_pose, dest_pose = scan_for_both_markers(cam, estimator)
        if ugv_pose is None:  # timed out without seeing both markers
            log_event("[!] Could not see both markers. Aborting navigation.")
            bridge.send_message("UGV NAV ABORT: MARKERS NOT FOUND")
            return  # bail cleanly

        dist_initial = get_3d_distance(ugv_pose, dest_pose)  # starting distance for reference
        log_event(f"[NAV] Both markers acquired. Initial distance: {dist_initial:.3f}m")
        bridge.send_message(f"UGV NAV: MARKERS FOUND DIST={dist_initial:.2f}m")

        # -------------------------------------------------------------- #
        # STEP 6: PHASE 1 - align the UGV toward the destination marker   #
        # -------------------------------------------------------------- #
        # the throttle must stay alive during navigation or the drone will crash
        set_throttle(master, THROTTLE_HOVER)  # send initial hover pulse
        cmd_seq_counter = phase_align(cam, estimator, bridge, cmd_seq_counter)  # do the alignment
        log_event("[NAV] Alignment phase complete.")
        bridge.send_message("UGV NAV: ALIGNED - STARTING DRIVE")
        set_throttle(master, THROTTLE_HOVER)  # keep hover pulse alive after alignment phase

        # -------------------------------------------------------------- #
        # STEP 7: PHASE 2 - drive the UGV to the destination marker       #
        # -------------------------------------------------------------- #
        cmd_seq_counter = phase_approach(cam, estimator, bridge, cmd_seq_counter)  # do the drive
        log_event("[NAV] Approach phase complete.")
        bridge.send_message("UGV NAV: ARRIVED AT DESTINATION")
        set_throttle(master, THROTTLE_HOVER)  # keep hover pulse alive after approach phase

        # -------------------------------------------------------------- #
        # STEP 8: send a stop command to make sure UGV halts cleanly      #
        # -------------------------------------------------------------- #
        cmd_seq_counter += 1
        bridge.send_command(cmdSeq=cmd_seq_counter, cmd=v2v_bridge.CMD_STOP, estop=0)
        log_event("[NAV] Stop command sent to UGV. Mission complete.")

        # -------------------------------------------------------------- #
        # STEP 9: final confirmation scan - log the achieved distance      #
        # -------------------------------------------------------------- #
        time.sleep(1.0)  # short pause so the UGV fully stops before we measure
        ugv_pose_final, dest_pose_final = scan_for_both_markers(cam, estimator)
        if ugv_pose_final is not None:
            final_dist = get_3d_distance(ugv_pose_final, dest_pose_final)
            log_event(f"[NAV] Final confirmed distance: {final_dist:.3f}m")
            bridge.send_message(f"UGV NAV DONE: FINAL DIST={final_dist:.2f}m")
        else:
            log_event("[NAV] Final scan inconclusive (markers not visible). Mission still complete.")

    except KeyboardInterrupt:  # someone hit ctrl+c
        log_event("[!] Emergency: User interrupt. Sending UGV stop command.")
        bridge.send_command(cmdSeq=999, cmd=v2v_bridge.CMD_STOP, estop=1)  # emergency stop
        change_mode(master, "LAND")  # force land mode immediately
        set_throttle(master, 0)      # release throttle override
        time.sleep(1)                # wait for commands to hit

    except RuntimeError as e:  # navigation logic raised a hard error
        log_event(f"[!] Navigation error: {e}")
        bridge.send_command(cmdSeq=998, cmd=v2v_bridge.CMD_STOP, estop=1)  # safe the UGV
        bridge.send_message(f"UGV NAV ERROR: {str(e)[:40]}")

    finally:  # always clean up
        cam.close()      # close the ZED camera and release resources
        bridge.stop()    # close radio serial wire
        log_event("Navigator finalised.")

if __name__ == "__main__":  # entry point
    main()  # run it
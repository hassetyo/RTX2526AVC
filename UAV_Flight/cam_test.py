from pymavlink import mavutil # using the confirmed mavlink pattern instead of dronekit
import time # for timing and sleeps
import math # for distance calculations
import sys # for clean exits
import v2v_bridge # our custom radio bridge talker
import cv2 # for aruco detection and preview window
import cv2.aruco as aruco # aruco submodule
import numpy as np # math for pixel offsets and transforms
import pyzed.sl as sl # zed 2 sdk on jetson nano
from dataclasses import dataclass # clean data container for marker positions
from typing import Optional, List, Tuple # type hints

# uav aruco marker follower
# uses the zed 2 camera + calibration file to detect an aruco marker via pose estimation
# computes real pixel offset from frame center, logs the offset, nudges drone to center it
# camera and detection pattern taken directly from oldvisiontest.py / calibration_chessboard.yaml

################################# config stuff i setup
# connection settings from your working test script
CONNECTION_STRING = "/dev/ttyACM0"   # drone wire (use COM4 if testing on windows)
BAUD_RATE = 57600                    # using the confirmed 57600 speed
ESP32_PORT = "/dev/ttyUSB0"          # the radio bridge usb wire

# path to the calibration file from oldvisiontest.py
CALIBRATION_FILE = "calibration_chessboard.yaml"  # must be in the same directory as this script

# mission params
TARGET_ALT = 1.3    # hover height in meters (~4.2 ft)

# throttle settings i tuned (same as mission4)
THROTTLE_MIN = 1000   # motors off
THROTTLE_IDLE = 1150  # props spinning but no lift
THROTTLE_CLIMB = 1650 # power to lift off the floor
THROTTLE_HOVER = 1500 # rough middle ground for holding height

# aruco settings (matches oldvisiontest.py)
ARUCO_DICT_TYPE = aruco.DICT_6X6_1000  # dictionary from oldvisiontest.py
MARKER_SIZE = 0.1                       # physical marker size in meters (matches oldvisiontest.py)
ARUCO_LOST_TIMEOUT = 3.0               # seconds marker can be missing mid-follow before we land

# rc channel mapping for ardupilot (standard)
RC_ROLL_CH     = 1  # channel 1 = roll (left/right)
RC_PITCH_CH    = 2  # channel 2 = pitch (forward/back)
RC_THROTTLE_CH = 3  # channel 3 = throttle
RC_YAW_CH      = 4  # channel 4 = yaw (locked at center)

# rc pwm values
RC_CENTER = 1500  # neutral stick - no movement
RC_MIN    = 1000  # full deflection one way
RC_MAX    = 2000  # full deflection other way

# pid gains - map pixel offset from center to a pwm nudge
# tune KP up if drone doesnt react, down if it oscillates
KP_ROLL  = 0.05   # proportional gain for left/right (x axis) correction
KP_PITCH = 0.05   # proportional gain for forward/back (y axis) correction
MAX_NUDGE = 150   # max pwm offset from center - caps the correction size

# deadband in pixels - ignore tiny offsets to stop jitter
PIXEL_DEADBAND = 20  # pixels

# follow loop rate
FOLLOW_HZ = 10  # rc override update rate in hz (10hz is safe for ardupilot)

# preview window
WINDOW_NAME = "UAV ArUco Follower"  # opencv window title

################################# marker position data class (from oldvisiontest.py)

@dataclass
class MarkerPosition: # clean container for one detected markers data
    """Data class for marker position information - taken from oldvisiontest.py"""
    marker_id: int    # the aruco id number
    x: float          # side distance in meters (camera frame x)
    y: float          # vertical distance in meters (camera frame y)
    z: float          # forward depth in meters (camera frame z)
    distance: float   # total euclidean distance in meters
    pixel_cx: int     # center pixel x of the marker in the frame
    pixel_cy: int     # center pixel y of the marker in the frame
    detected: bool = True

################################# camera calibration loader (from oldvisiontest.py)

def load_calibration(file_path: str) -> Tuple[np.ndarray, np.ndarray]: # loads K and D from yaml
    """
    Load camera matrix and distortion coefficients from the calibration YAML file.
    Exact same pattern as oldvisiontest.py ArucoDetector._load_calibration().
    """
    fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ) # open the yaml
    camera_matrix = fs.getNode("K").mat() # grab the intrinsic matrix
    dist_coeffs = fs.getNode("D").mat()   # grab the distortion coefficients
    fs.release() # close the file

    if camera_matrix is None or dist_coeffs is None: # if either is missing
        raise ValueError(f"Invalid calibration file: {file_path}") # blow up loudly

    print(f"[CAL] Loaded calibration from {file_path}") # log success
    print(f"[CAL] Camera matrix:\n{camera_matrix}") # print K so we can verify
    print(f"[CAL] Distortion coeffs: {dist_coeffs.flatten()}") # print D
    return camera_matrix, dist_coeffs # hand both back

################################# zed 2 camera init (from CameraUAVTest.py pattern)

def open_zed_camera(): # opens the zed 2 and returns the camera object
    """
    Opens the ZED 2 camera.
    Depth mode is set to NONE because we are using calibration-based pose estimation
    (estimatePoseSingleMarkers) for 3D position, not the ZED point cloud.
    This matches the oldvisiontest.py ZED setup which also uses DEPTH_MODE.NONE.
    Returns (cam, zed_image) on success or (None, None) on failure.
    """
    print("[ZED] Opening ZED 2 camera...") # log startup

    cam = sl.Camera() # create camera object
    init_params = sl.InitParameters() # default params
    init_params.camera_resolution = sl.RESOLUTION.HD720  # 720p matches oldvisiontest pattern
    init_params.camera_fps = 30 # 30fps
    init_params.depth_mode = sl.DEPTH_MODE.NONE # no zed depth - we use pose estimation instead
    # note: exposure is also set in oldvisiontest.py, uncomment line below if lighting is an issue
    # cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 1)

    status = cam.open(init_params) # open the camera
    if status != sl.ERROR_CODE.SUCCESS: # if it failed
        print(f"[ZED] ZED open failed: {status}") # log error
        return None, None # bail

    zed_image = sl.Mat() # mat to receive raw colour frames
    print("[ZED] Camera open.") # log success
    return cam, zed_image # hand back

################################# aruco detection with pose estimation (from oldvisiontest.py)

def detect_markers(frame_bgr: np.ndarray, aruco_dict, camera_matrix: np.ndarray,
                   dist_coeffs: np.ndarray) -> List[MarkerPosition]: # detects all markers and returns pose data
    """
    Detect ArUco markers and estimate their 3D positions using camera calibration.
    Taken directly from oldvisiontest.py ArucoDetector.detect().
    Uses estimatePoseSingleMarkers for real X/Y/Z in meters.
    Returns a list of MarkerPosition objects.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) # grayscale for detection
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict) # run the detector

    positions = [] # will hold results

    if ids is not None: # if we found at least one marker
        for i, marker_id in enumerate(ids.flatten()): # loop each detection
            # estimatePoseSingleMarkers gives us the translation vector in camera frame
            # this is the same call as oldvisiontest.py
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners[i], MARKER_SIZE,
                camera_matrix, dist_coeffs
            )

            tvec = tvecs[0].flatten()  # translation vector: [x, y, z] in meters
            x, y, z = tvec[0], tvec[1], tvec[2] # unpack camera frame coords
            distance = float(np.linalg.norm(tvec)) # total euclidean distance to marker

            # compute pixel center of this marker using diagonal average (same as CameraUAVTest.py)
            c = corners[i][0]  # shape (4, 2) corner points
            cX = int((c[0][0] + c[2][0]) / 2.0) # diagonal avg x
            cY = int((c[0][1] + c[2][1]) / 2.0) # diagonal avg y

            positions.append(MarkerPosition(
                marker_id=int(marker_id),
                x=float(x),
                y=float(y),
                z=float(z),
                distance=distance,
                pixel_cx=cX,
                pixel_cy=cY,
            ))

    return positions # hand back all detections

################################# overlay drawing (from oldvisiontest.py draw_detections pattern)

def draw_overlay(frame_bgr: np.ndarray, positions: List[MarkerPosition],
                 aruco_dict, camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
                 frame_w: int, frame_h: int, status_text: str,
                 offset_x: int = 0, offset_y: int = 0) -> np.ndarray: # annotates the frame for preview
    """
    Draws onto the BGR frame for the live preview window.
    Follows oldvisiontest.py draw_detections() pattern:
    - aruco.drawDetectedMarkers() for the marker boundary boxes
    - cv2.drawFrameAxes() for the 3D pose axes
    - Position info labels per marker
    - Red crosshair at frame center (the target)
    - Blue offset vector line from center toward where the drone needs to go
    - Black status bar at top
    """
    display = frame_bgr.copy() # copy so we dont mess up the original

    # red crosshair at the frame center - this is where we want the marker to be
    center_x, center_y = frame_w // 2, frame_h // 2 # frame center pixel
    cv2.drawMarker(display, (center_x, center_y), (0, 0, 255), cv2.MARKER_CROSS, 40, 2) # red cross

    if positions: # if we have any detections this frame
        # re-detect corners for drawDetectedMarkers and drawFrameAxes
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) # grayscale needed for corner re-detect
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict) # get corners again for drawing

        if ids is not None: # if detection returned results
            display = aruco.drawDetectedMarkers(display, corners, ids) # draw the green marker boxes

            for i, marker_id in enumerate(ids.flatten()): # loop each marker
                # draw the 3D pose axes (same as oldvisiontest.py)
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], MARKER_SIZE, camera_matrix, dist_coeffs
                )
                cv2.drawFrameAxes(display, camera_matrix, dist_coeffs,
                                  rvecs[0], tvecs[0], MARKER_SIZE * 0.5) # draw XYZ axes on marker

                # find the matching position data for this id
                pos = next((p for p in positions if p.marker_id == int(marker_id)), None)
                if pos: # if we have position data for this marker
                    color = (0, 255, 0) # green for all markers

                    # label matching oldvisiontest.py format + distance
                    label = (f"ID:{pos.marker_id} "
                             f"X:{pos.x:.2f}m Y:{pos.y:.2f}m Z:{pos.z:.2f}m "
                             f"D:{pos.distance:.2f}m")

                    # put label above the marker center
                    cv2.putText(display, label, (pos.pixel_cx - 120, pos.pixel_cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

                    # green crosshair at the marker pixel center
                    cv2.drawMarker(display, (pos.pixel_cx, pos.pixel_cy),
                                   (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

    # if there is a non-zero offset draw a blue arrow from center toward the correction direction
    if offset_x != 0 or offset_y != 0: # if drone needs to move
        arrow_end = (center_x + offset_x // 3, center_y + offset_y // 3) # scale arrow down for display
        cv2.arrowedLine(display, (center_x, center_y), arrow_end, (255, 100, 0), 2) # blue arrow

    # black status bar at the top
    cv2.rectangle(display, (0, 0), (frame_w, 28), (0, 0, 0), -1) # black strip
    cv2.putText(display, status_text, (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) # white text

    return display # hand back annotated frame

################################# mavlink helpers (same as mission4)

def change_mode(master, mode: str): # changes the flight controller mode
    mapping = master.mode_mapping() # ask for the list of modes
    if mode not in mapping: # if the mode is fake
        print(f"Unknown mode '{mode}'") # log the error
        return # bail out
    mode_id = mapping[mode] # find the secret mode id
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id) # blast it
    print(f"Mode set: {mode}") # log the change
    time.sleep(1) # wait for the mode to settle

def arm_drone(master): # engages the scary drone motors
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Arming motors...") # log the arming
    time.sleep(2) # wait for the spinning to start

def disarm_drone(master): # stops the motors safely
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Disarmed.") # log the safety

def set_throttle(master, pwm): # physically pushes the throttle via rc override (same as mission4)
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )

def set_rc_override(master, roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm=RC_CENTER): # full 4-channel rc override
    # channels 1-4 = roll, pitch, throttle, yaw in ardupilot standard layout
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        roll_pwm,     # ch1 roll
        pitch_pwm,    # ch2 pitch
        throttle_pwm, # ch3 throttle
        yaw_pwm,      # ch4 yaw (locked to center)
        0, 0, 0, 0    # ch5-8 unused
    )

def get_lidar_alt(master): # checks the floor distance via lidar (same as mission4)
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0) # wait for lidar packet
    if msg: # if we got a real message
        return msg.current_distance / 100.0 # return height in meters
    return 0.0 # return zero if lidar is dead

################################# altitude hold (same crude logic as mission4)

def compute_throttle_hold(current_alt: float) -> int: # adjusts throttle to maintain TARGET_ALT
    if current_alt < TARGET_ALT - 0.1:   # if we are sinking
        return THROTTLE_HOVER + 100       # give it more juice
    elif current_alt > TARGET_ALT + 0.1: # if we are drifting too high
        return THROTTLE_HOVER - 100       # cut power
    else:                                 # if we are in the sweet spot
        return THROTTLE_HOVER             # keep steady

################################# centering logic - the core of the follow behaviour

def compute_centering_correction(pos: MarkerPosition, frame_w: int,
                                 frame_h: int) -> Tuple[int, int, int, int]: # returns roll_pwm, pitch_pwm, offset_x, offset_y
    """
    Computes how far the marker is from the frame center in pixels,
    logs the offset, then converts it to roll/pitch rc pwm corrections.

    Returns (roll_pwm, pitch_pwm, pixel_offset_x, pixel_offset_y).
    offset_x positive = marker is RIGHT of center  -> drone should roll right
    offset_y positive = marker is BELOW center     -> drone should pitch forward
    NOTE: if the drone moves the wrong direction on an axis, flip the sign in the nudge calc.
    """
    center_x = frame_w / 2.0 # frame horizontal center
    center_y = frame_h / 2.0 # frame vertical center

    # raw pixel offset from frame center to marker center
    offset_x = int(pos.pixel_cx - center_x) # positive = marker is right of center
    offset_y = int(pos.pixel_cy - center_y) # positive = marker is below center

    # --- log the offset every frame so we have a record of how far off we were ---
    in_deadband_x = abs(offset_x) < PIXEL_DEADBAND # is x within the no-move zone
    in_deadband_y = abs(offset_y) < PIXEL_DEADBAND # is y within the no-move zone

    if in_deadband_x and in_deadband_y: # if marker is basically centered
        print(f" [Center] ID:{pos.marker_id} CENTERED  "
              f"offset=({offset_x:+d}px, {offset_y:+d}px)  "
              f"pos=X:{pos.x:.2f}m Y:{pos.y:.2f}m Z:{pos.z:.2f}m  D:{pos.distance:.2f}m",
              end='\r') # log centered state
    else: # marker is off center
        direction_x = "RIGHT" if offset_x > 0 else "LEFT"   # which horizontal direction
        direction_y = "FWD"   if offset_y > 0 else "BACK"   # which vertical direction
        needs_move = []
        if not in_deadband_x: needs_move.append(f"{direction_x} {abs(offset_x)}px") # add x move
        if not in_deadband_y: needs_move.append(f"{direction_y} {abs(offset_y)}px") # add y move
        move_str = " + ".join(needs_move) # combine into one string

        print(f"\n[Center] ID:{pos.marker_id} OFF-CENTER  "
              f"offset=({offset_x:+d}px, {offset_y:+d}px)  "
              f"MOVING: {move_str}  "
              f"pos=X:{pos.x:.2f}m Y:{pos.y:.2f}m Z:{pos.z:.2f}m  D:{pos.distance:.2f}m") # log the move

    # --- apply deadband before computing the pwm correction ---
    err_x = 0.0 if in_deadband_x else float(offset_x) # zero out if inside deadband
    err_y = 0.0 if in_deadband_y else float(offset_y) # zero out if inside deadband

    # proportional correction clamped to MAX_NUDGE
    roll_nudge  = max(-MAX_NUDGE, min(MAX_NUDGE, KP_ROLL  * err_x)) # left/right nudge
    pitch_nudge = max(-MAX_NUDGE, min(MAX_NUDGE, KP_PITCH * err_y)) # fwd/back nudge

    roll_pwm  = int(RC_CENTER + roll_nudge)  # final roll channel pwm
    pitch_pwm = int(RC_CENTER + pitch_nudge) # final pitch channel pwm

    return roll_pwm, pitch_pwm, offset_x, offset_y # hand back pwm and raw offsets for overlay

################################# main mission

def main(): # the main boss function
    print("==========================================")
    print("   UAV ARUCO MARKER FOLLOWER")
    print("   (calibration-based pose estimation)")
    print("==========================================")

    # step 1: load calibration file before anything else
    # this is the same file and same load pattern as oldvisiontest.py
    try:
        camera_matrix, dist_coeffs = load_calibration(CALIBRATION_FILE) # load K and D
    except Exception as e: # if file is missing or malformed
        print(f"[!] Calibration load failed: {e}") # log the error
        return # abort before connecting to anything

    # build the aruco dictionary (matches oldvisiontest.py exactly)
    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT_TYPE) # DICT_6X6_1000

    # step 2: connect to drone
    print(f"Connecting to Drone: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE) # open link
    master.wait_heartbeat() # wait for buzz
    print("Drone Heartbeat OK.")

    # step 3: connect to radio bridge
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # radio bridge
    try: # try to open radio
        bridge.connect() # open serial wires
        bridge.send_message("ARUCO FOLLOWER: START") # yell over air
    except: # if radio is missing
        print("Radio Bridge Fail.")
        return # bail

    # step 4: open the zed camera (DEPTH_MODE.NONE - we use pose estimation, not point cloud)
    cam, zed_image = open_zed_camera() # init zed
    if cam is None: # if camera failed to open
        print("[!] ZED camera failed to open. Aborting.")
        bridge.stop() # close radio
        return # bail

    # open the opencv preview window now so the feed is live from the start
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL) # resizable window
    cv2.resizeWindow(WINDOW_NAME, 1280, 720) # default size

    # grab one frame to get real frame dimensions
    frame_w, frame_h = 1280, 720 # fallback in case first grab fails
    if cam.grab() == sl.ERROR_CODE.SUCCESS: # if we got a frame
        cam.retrieve_image(zed_image, sl.VIEW.LEFT) # pull it
        frame = zed_image.get_data() # raw numpy BGRA
        frame_h, frame_w = frame.shape[:2] # save real dimensions
        print(f"[ZED] Frame size confirmed: {frame_w}x{frame_h}")

    try: # wrap mission in safety block
        # step 5: arm and takeoff - marker searching happens in the air
        change_mode(master, "STABILIZE") # stabilize for manual rc override control
        arm_drone(master) # start the props spinning

        # step 6: takeoff sequence (same as mission4)
        print("Climbing to target altitude...")
        while True: # loop until we hit target height
            alt = get_lidar_alt(master) # check lidar

            if cam.grab() == sl.ERROR_CODE.SUCCESS: # grab frame for preview during climb
                cam.retrieve_image(zed_image, sl.VIEW.LEFT) # pull it
                frame = zed_image.get_data() # raw BGRA
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) # strip alpha
                display = draw_overlay(frame_bgr, [], aruco_dict, camera_matrix, dist_coeffs,
                                       frame_w, frame_h,
                                       f"TAKEOFF  Alt:{alt:.2f}m  Target:{TARGET_ALT}m") # overlay
                cv2.imshow(WINDOW_NAME, display) # push to window
                cv2.waitKey(1) # process window events

            print(f" Altitude: {alt:.2f}m", end='\r') # log height to terminal

            if alt >= TARGET_ALT: # if we hit the hover point
                set_throttle(master, THROTTLE_HOVER) # pull back to hover power
                print(f"\nHover altitude reached: {alt:.2f}m")
                break # done climbing
            set_throttle(master, THROTTLE_CLIMB) # keep pushing up
            time.sleep(0.1) # quick loop

        # step 7: hover and wait for a marker to appear in frame
        # rotors stay on every cycle via set_rc_override - no timeout, just keeps hovering
        print("[ArUco] Hovering and waiting for marker to appear...")
        bridge.send_message("ARUCO FOLLOWER: WAITING FOR MARKER")

        acquired_id = None  # id of the first confirmed marker we lock onto
        scan_start = time.time() # for the scanning elapsed timer

        while acquired_id is None: # keep scanning until we lock onto a valid marker
            # keep throttle alive every cycle - critical so drone does not drop
            alt = get_lidar_alt(master) # check altitude
            throttle_pwm = compute_throttle_hold(alt) # altitude hold throttle
            set_rc_override(master, RC_CENTER, RC_CENTER, throttle_pwm) # hold position and altitude

            if cam.grab() == sl.ERROR_CODE.SUCCESS: # if we got a fresh frame
                cam.retrieve_image(zed_image, sl.VIEW.LEFT) # pull left camera image
                frame = zed_image.get_data() # raw numpy BGRA from ZED
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) # strip alpha

                # detect markers using calibration-based pose estimation (oldvisiontest.py pattern)
                positions = detect_markers(frame_bgr, aruco_dict, camera_matrix, dist_coeffs)

                if positions: # if at least one marker detected with valid pose
                    acquired_id = positions[0].marker_id # lock onto the first one
                    p = positions[0] # grab its data
                    print(f"\n[ArUco] Marker acquired! "
                          f"ID:{acquired_id} "
                          f"X:{p.x:.2f}m Y:{p.y:.2f}m Z:{p.z:.2f}m D:{p.distance:.2f}m")
                    bridge.send_message(f"ARUCO ACQUIRED: ID {acquired_id}") # broadcast

                elapsed = time.time() - scan_start # seconds searching
                status = (f"SEARCHING  Alt:{alt:.2f}m  {elapsed:.1f}s"
                          if acquired_id is None
                          else f"ACQUIRED ID:{acquired_id}")
                display = draw_overlay(frame_bgr, positions, aruco_dict, camera_matrix, dist_coeffs,
                                       frame_w, frame_h, status) # annotate frame
                cv2.imshow(WINDOW_NAME, display) # push to window
                cv2.waitKey(1) # process window events

                if acquired_id is None: # still searching
                    print(f" [ArUco] Hovering, scanning... {elapsed:.1f}s", end='\r')

            time.sleep(0.05) # ~20fps scan rate, easy on the nano

        # step 8: the main centering / follow loop
        print("[Follow] Entering centering loop...")
        bridge.send_message("ARUCO FOLLOWER: TRACKING ACTIVE")

        lost_timer = None  # tracks when we first lose the marker
        loop_interval = 1.0 / FOLLOW_HZ # seconds per loop cycle

        while True: # centering loop runs until land or abort
            loop_start = time.time() # for loop timing

            # ---- grab a fresh frame ----
            if cam.grab() != sl.ERROR_CODE.SUCCESS: # if the camera hiccuped
                print("[ZED] Frame grab failed, retrying...") # log
                time.sleep(0.05) # brief pause
                continue # retry next cycle

            cam.retrieve_image(zed_image, sl.VIEW.LEFT) # pull left camera image
            frame = zed_image.get_data() # raw BGRA
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) # strip alpha

            # detect markers with pose estimation (oldvisiontest.py pattern)
            positions = detect_markers(frame_bgr, aruco_dict, camera_matrix, dist_coeffs)

            # ---- marker found ----
            if positions: # if we see at least one marker
                lost_timer = None # reset lost clock

                # use the first detected marker (same target as acquisition)
                pos = positions[0] # grab first marker

                # compute pixel offset from center, log it, and get rc correction
                roll_pwm, pitch_pwm, off_x, off_y = compute_centering_correction(
                    pos, frame_w, frame_h
                )

                # altitude hold
                alt = get_lidar_alt(master) # lidar altitude
                throttle_pwm = compute_throttle_hold(alt) # hold throttle

                # send the correction to the drone
                set_rc_override(master, roll_pwm, pitch_pwm, throttle_pwm) # fire correction

                # build a status string summarising what we logged and what we sent
                in_db = abs(off_x) < PIXEL_DEADBAND and abs(off_y) < PIXEL_DEADBAND # in deadband?
                move_desc = "HOLDING" if in_db else f"ROLL:{roll_pwm} PITCH:{pitch_pwm}" # describe move
                status_txt = (f"TRACKING ID:{pos.marker_id}  "
                              f"offset=({off_x:+d},{off_y:+d})px  "
                              f"{move_desc}  "
                              f"X:{pos.x:.2f}m Y:{pos.y:.2f}m Z:{pos.z:.2f}m  "
                              f"Alt:{alt:.2f}m")

                display = draw_overlay(frame_bgr, positions, aruco_dict, camera_matrix, dist_coeffs,
                                       frame_w, frame_h, status_txt, off_x, off_y) # annotate

            # ---- marker not found ----
            else:
                if lost_timer is None: # first frame it went missing
                    lost_timer = time.time() # start lost clock
                    print(f"\n[Follow] Marker lost. Holding position...") # log

                # hold position while we wait to reacquire
                alt = get_lidar_alt(master) # altitude check
                throttle_pwm = compute_throttle_hold(alt) # altitude hold
                set_rc_override(master, RC_CENTER, RC_CENTER, throttle_pwm) # hold

                time_lost = time.time() - lost_timer # seconds since marker disappeared
                print(f" [Follow] Waiting to reacquire... {time_lost:.1f}s", end='\r') # log

                display = draw_overlay(frame_bgr, [], aruco_dict, camera_matrix, dist_coeffs,
                                       frame_w, frame_h,
                                       f"SEARCHING  Alt:{alt:.2f}m  lost:{time_lost:.1f}s") # overlay

                if time_lost >= ARUCO_LOST_TIMEOUT: # if marker gone too long
                    print(f"\n[!] Marker lost for {ARUCO_LOST_TIMEOUT}s. Initiating landing.")
                    bridge.send_message("ARUCO LOST: LANDING") # broadcast
                    break # exit loop and land

            # ---- push frame to preview window ----
            cv2.imshow(WINDOW_NAME, display) # show annotated frame
            if cv2.waitKey(1) & 0xFF == ord('q'): # q key = user triggered landing
                print("\n[!] Q pressed - User triggered landing.")
                break # exit loop and land

            # ---- pace the loop ----
            elapsed = time.time() - loop_start # time this cycle took
            sleep_time = loop_interval - elapsed # time left in this cycle
            if sleep_time > 0: # if there is time remaining
                time.sleep(sleep_time) # sleep the remainder

        # step 9: land and shutdown (same as mission4)
        print("\nLanding sequence engaged...")
        change_mode(master, "LAND") # switch to official land mode
        set_rc_override(master, RC_CENTER, RC_CENTER, 0, RC_CENTER) # release all overrides

        while True: # loop until touchdown
            alt = get_lidar_alt(master) # check lidar
            print(f" Land Alt: {alt:.2f}m", end='\r') # log altitude

            if cam.grab() == sl.ERROR_CODE.SUCCESS: # keep preview live during landing
                cam.retrieve_image(zed_image, sl.VIEW.LEFT) # pull frame
                frame = zed_image.get_data() # raw BGRA
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) # strip alpha
                display = draw_overlay(frame_bgr, [], aruco_dict, camera_matrix, dist_coeffs,
                                       frame_w, frame_h, f"LANDING  Alt:{alt:.2f}m") # landing overlay
                cv2.imshow(WINDOW_NAME, display) # push to window
                cv2.waitKey(1) # process window events

            msg = master.recv_match(type='HEARTBEAT', blocking=False) # check for disarm
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("\nTouchdown confirmed. Motors stopped.") # log success
                break # done
            time.sleep(0.5) # slower check loop

    except KeyboardInterrupt: # someone hit ctrl+c
        print("\n[!] Emergency: User Triggered Landing...")
        change_mode(master, "LAND") # force land mode immediately
        set_rc_override(master, RC_CENTER, RC_CENTER, 0, RC_CENTER) # release all overrides
        time.sleep(1) # wait for command to register
    finally: # always clean up
        cv2.destroyAllWindows() # close preview window
        cam.close() # close the zed camera cleanly
        print("[ZED] ZED 2 camera closed.")
        bridge.stop() # close radio wire
        print("Aruco follower mission finalized.")

if __name__ == "__main__": # entry point
    main() # run it

from pymavlink import mavutil # using the confirmed mavlink pattern instead of dronekit
import time # for timing and sleeps
import sys # for clean exits
import v2v_bridge # our custom radio bridge talker
import cv2 # for aruco detection
import pyzed.sl as sl # zed 2 sdk on jetson nano
import numpy as np # math for pixel offsets and vector transforms

# uav aruco marker follower
# uses the zed 2 camera to detect an aruco marker and keeps the drone hovering over it
# if the marker moves, the drone follows it using rc channel overrides on pitch/roll
# built on top of the mission4 / camerauavtest confirmed patterns

################################# config stuff i setup
# connection settings from your working test script
CONNECTION_STRING = "/dev/ttyACM0"   # drone wire (use COM4 if testing on windows)
BAUD_RATE = 57600                    # using the confirmed 57600 speed
ESP32_PORT = "/dev/ttyUSB0"          # the radio bridge usb wire

# mission params
TARGET_ALT = 1.3    # hover height in meters (~4.2 ft)

# throttle settings i tuned (same as mission4)
THROTTLE_MIN = 1000   # motors off
THROTTLE_IDLE = 1150  # props spinning but no lift
THROTTLE_CLIMB = 1650 # power to lift off the floor
THROTTLE_HOVER = 1500 # rough middle ground for holding height

# aruco settings (must match camerauavtest)
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50  # marker dictionary type (change to match your printed marker)
ARUCO_LOST_TIMEOUT = 3.0               # seconds marker can be missing mid-follow before we land

# rc channel mapping for ardupilot (standard)
RC_ROLL_CH     = 1  # channel 1 = roll (left/right drift correction)
RC_PITCH_CH    = 2  # channel 2 = pitch (forward/back drift correction)
RC_THROTTLE_CH = 3  # channel 3 = throttle (altitude hold)
RC_YAW_CH      = 4  # channel 4 = yaw (rotation - we lock this at center for now)

# rc pwm center and limits for pitch/roll nudges
RC_CENTER    = 1500  # neutral stick position (no movement)
RC_MIN       = 1000  # full deflection one way
RC_MAX       = 2000  # full deflection the other way

# pid controller gains for the position hold loop
# these map pixel error (how far the marker is from center) to a pwm nudge
# tune kp up if the drone isnt reacting, down if it oscillates wildly
KP_ROLL     = 0.05   # proportional gain for left/right correction
KP_PITCH    = 0.05   # proportional gain for forward/back correction
MAX_NUDGE   = 150    # maximum rc pwm offset from center (keeps it smooth, prevents flips)

# deadband - if the marker is within this many pixels of center we dont nudge at all
# prevents micro-jitter when the marker is basically already centered
PIXEL_DEADBAND = 20  # pixels

# follow loop rate
FOLLOW_HZ = 10  # how many times per second we update rc commands (10hz is safe for ardupilot)

############################ the mavlink helpers (same as mission4)

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
    # channel 3 is the throttle in ardupilot
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )

def set_rc_override(master, roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm=RC_CENTER): # full 4-channel rc override
    # sends roll, pitch, throttle, and yaw all at once so the drone follows the marker
    # channels 1-4 = roll, pitch, throttle, yaw in ardupilot standard layout
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        roll_pwm,     # ch1 roll
        pitch_pwm,    # ch2 pitch
        throttle_pwm, # ch3 throttle
        yaw_pwm,      # ch4 yaw (locked to center for now)
        0, 0, 0, 0    # ch5-8 unused
    )

def get_lidar_alt(master): # checks the floor distance via lidar (same as mission4)
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0) # wait for lidar packet
    if msg: # if we got a real message
        return msg.current_distance / 100.0 # return height in meters
    return 0.0 # return zero if lidar is dead

############################ zed 2 camera init (pulled from camerauavtest pattern)

def open_zed_camera(): # opens the zed 2 and builds the aruco detector
    """
    Opens the ZED 2 camera using the same init pattern as CameraUAVTest.py.
    Returns (cam, detector, zed_image) tuple on success, or (None, None, None) on failure.
    """
    print("[ZED] Opening ZED 2 camera...") # log startup

    cam = sl.Camera() # create camera object
    init_params = sl.InitParameters() # default params
    init_params.camera_resolution = sl.RESOLUTION.HD720  # 720p is fine for aruco, easier on the nano
    init_params.camera_fps = 30 # 30fps
    init_params.depth_mode = sl.DEPTH_MODE.NONE # no depth needed here, saves cpu on the nano

    status = cam.open(init_params) # open the camera
    if status != sl.ERROR_CODE.SUCCESS: # if it failed
        print(f"[ZED] ZED open failed: {status}") # log the error
        return None, None, None # bail with nothing useful

    # build the aruco detector (same dict as camerauavtest)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE) # load marker dictionary
    aruco_params = cv2.aruco.DetectorParameters() # default detection params
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params) # build detector object

    zed_image = sl.Mat() # mat to receive raw frames
    print("[ZED] Camera open and detector ready.") # log success
    return cam, detector, zed_image # hand everything back to caller


############################ the pid correction math i wrote

def compute_rc_correction(marker_center_x, marker_center_y, frame_w, frame_h):
    """
    Takes the marker's pixel position in the frame and computes the roll/pitch
    rc pwm corrections needed to drift the drone back over the marker.
    
    Pixel origin (0,0) is top-left of the frame.
    Frame center is (frame_w/2, frame_h/2).
    
    Positive x error (marker is right of center) -> roll right to chase it.
    Positive y error (marker is below center) -> pitch forward to chase it.
    Pitch/Roll axis mapping depends on camera mounting orientation on the drone.
    NOTE: if the drone chases the wrong direction, flip the sign on that axis below.
    """
    # center of the frame - this is where we WANT the marker to sit
    cx = frame_w / 2.0 # horizontal center pixel
    cy = frame_h / 2.0 # vertical center pixel

    # pixel error = how far the marker is from where we want it
    err_x = marker_center_x - cx  # positive = marker is RIGHT of center
    err_y = marker_center_y - cy  # positive = marker is BELOW center (y flips in image coords)

    # apply deadband - dont nudge if the error is tiny
    if abs(err_x) < PIXEL_DEADBAND: # if x error is inside the deadband
        err_x = 0.0 # treat it as zero
    if abs(err_y) < PIXEL_DEADBAND: # if y error is inside the deadband
        err_y = 0.0 # treat it as zero

    # proportional controller: nudge = center + (gain * error), clamped to safe range
    # roll correction: marker right of center -> roll right (pwm > 1500)
    roll_nudge = KP_ROLL * err_x # raw correction value
    roll_nudge = max(-MAX_NUDGE, min(MAX_NUDGE, roll_nudge)) # clamp it so we dont flip
    roll_pwm = int(RC_CENTER + roll_nudge) # final pwm value

    # pitch correction: marker below center in image = marker is FORWARD of drone (camera faces down)
    # so positive y error -> pitch forward -> pwm > 1500
    pitch_nudge = KP_PITCH * err_y # raw correction value
    pitch_nudge = max(-MAX_NUDGE, min(MAX_NUDGE, pitch_nudge)) # clamp it
    pitch_pwm = int(RC_CENTER + pitch_nudge) # final pwm value

    return roll_pwm, pitch_pwm # hand back both channel corrections

def get_marker_center(corners): # extracts the center pixel of the first detected marker
    """
    Given the corners array from detectMarkers, returns the (x, y) pixel center
    of the first marker in the list.
    corners[0] shape is (1, 4, 2) - one marker, 4 corners, xy each.
    """
    c = corners[0][0]  # shape (4, 2), the 4 corner points of the first marker
    cx = float(np.mean(c[:, 0]))  # average of all 4 x coordinates
    cy = float(np.mean(c[:, 1]))  # average of all 4 y coordinates
    return cx, cy # return center x, center y

############################ altitude hold helper (same crude logic as mission4)

def compute_throttle_hold(current_alt):
    """
    Same crude altitude hold logic from mission4 - adjusts throttle to stay near TARGET_ALT.
    Returns the throttle pwm value to use this cycle.
    """
    if current_alt < TARGET_ALT - 0.1:  # if we are sinking
        return THROTTLE_HOVER + 100  # give it more juice
    elif current_alt > TARGET_ALT + 0.1:  # if we are drifting too high
        return THROTTLE_HOVER - 100  # cut power
    else:  # if we are golden
        return THROTTLE_HOVER  # keep steady

#################### the main follow mission

def main(): # the main boss function
    print("==========================================")
    print("   UAV ARUCO MARKER FOLLOWER")
    print("==========================================")

    # step 1: connect to the wires (same as mission4)
    print(f"Connecting to Drone: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE) # open link
    master.wait_heartbeat() # wait for buzz
    print("Drone Heartbeat OK.")

    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # radio bridge
    try: # try to open radio
        bridge.connect() # open serial wires
        bridge.send_message("ARUCO FOLLOWER: START") # yell over air
    except: # if radio is missing
        print("Radio Bridge Fail.")
        return # bail

    # step 2: open the zed camera before we arm - so it is ready the moment we reach altitude
    cam, detector, zed_image = open_zed_camera() # init zed and aruco
    if cam is None: # if camera failed to open
        print("[!] ZED camera failed to open. Aborting.")
        bridge.stop() # close radio
        return # bail

    # grab one frame to get frame dimensions before the mission starts
    frame_w, frame_h = 0, 0 # will be filled below
    if cam.grab() == sl.ERROR_CODE.SUCCESS: # if we got a frame
        cam.retrieve_image(zed_image, sl.VIEW.LEFT) # pull it
        frame = zed_image.get_data() # raw numpy BGRA
        frame_h, frame_w = frame.shape[:2] # save the dimensions
        print(f"[ZED] Frame size confirmed: {frame_w}x{frame_h}")

    try: # wrap mission in safety block
        # step 3: arm and takeoff FIRST - no aruco gate before flight
        change_mode(master, "STABILIZE") # stabilize for manual rc override control
        arm_drone(master) # start the props spinning

        # step 4: takeoff sequence (same as mission4)
        print("Climbing to target altitude...")
        while True: # loop until we hit target height
            alt = get_lidar_alt(master) # check lidar
            print(f" Altitude: {alt:.2f}m", end='\r') # log height
            if alt >= TARGET_ALT: # if we hit the hover point
                set_throttle(master, THROTTLE_HOVER) # pull back to hover power
                print(f"\nHover altitude reached: {alt:.2f}m")
                break # done climbing
            set_throttle(master, THROTTLE_CLIMB) # keep pushing up
            time.sleep(0.1) # quick loop

        # step 5: now that we are in the air, wait for the aruco marker to appear
        # the drone holds a hover while the camera scans - no timeout abort here,
        # it will just keep hovering and scanning until it sees the marker
        print("[ArUco] Hovering and waiting for marker to appear...")
        bridge.send_message("ARUCO FOLLOWER: WAITING FOR MARKER")

        marker_id = None # will hold the first detected marker id
        scan_start = time.time() # track how long we have been waiting
        while marker_id is None: # keep scanning until we see something
            # keep the throttle alive every cycle so the drone doesnt cut out
            alt = get_lidar_alt(master) # check altitude
            throttle_pwm = compute_throttle_hold(alt) # altitude hold throttle
            set_rc_override(master, RC_CENTER, RC_CENTER, throttle_pwm) # hold position while waiting

            if cam.grab() == sl.ERROR_CODE.SUCCESS: # if we got a fresh frame
                cam.retrieve_image(zed_image, sl.VIEW.LEFT) # pull left camera image
                frame = zed_image.get_data() # raw numpy BGRA from ZED
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) # strip alpha channel
                gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) # grayscale for aruco

                corners, ids, rejected = detector.detectMarkers(gray) # run the detector

                if ids is not None and len(ids) > 0: # if we found at least one marker
                    marker_id = int(ids[0][0]) # grab the first marker id
                    print(f"\n[ArUco] Marker detected! ID: {marker_id}. Starting follow.")
                    bridge.send_message(f"ARUCO MARKER ACQUIRED: ID {marker_id}")
                else:
                    elapsed = time.time() - scan_start # seconds waiting
                    print(f" [ArUco] Hovering, scanning for marker... {elapsed:.1f}s", end='\r') # live timer

            time.sleep(0.05) # ~20fps scan rate, easy on the nano

        # step 6: the main follow loop
        print("[Follow] Entering marker follow loop...")
        bridge.send_message("ARUCO FOLLOWER: TRACKING ACTIVE")

        lost_timer = None  # will track when we first lose sight of the marker

        loop_interval = 1.0 / FOLLOW_HZ # seconds per loop cycle
        while True: # follow loop runs until we land or abort
            loop_start = time.time() # for timing the loop

            # ---- grab a fresh frame from the zed ----
            if cam.grab() != sl.ERROR_CODE.SUCCESS: # if the camera hiccuped
                print("[ZED] Frame grab failed, retrying...") # log the skip
                time.sleep(0.05) # brief pause before retry
                continue # try again next cycle

            cam.retrieve_image(zed_image, sl.VIEW.LEFT) # pull left camera image
            frame = zed_image.get_data() # raw numpy BGRA from ZED
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) # strip alpha
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) # grayscale for aruco

            corners, ids, rejected = detector.detectMarkers(gray) # run the detector

            # ---- marker found this frame ----
            if ids is not None and len(ids) > 0: # if we see at least one marker
                lost_timer = None # reset the lost timer since we can see it

                marker_cx, marker_cy = get_marker_center(corners) # pixel center of marker

                # compute the rc corrections needed to fly toward the marker
                roll_pwm, pitch_pwm = compute_rc_correction(marker_cx, marker_cy, frame_w, frame_h)

                # compute throttle hold (same crude logic as mission4)
                alt = get_lidar_alt(master) # check lidar for altitude
                throttle_pwm = compute_throttle_hold(alt) # get hold throttle

                # send the full 4-channel override to move and hold altitude
                set_rc_override(master, roll_pwm, pitch_pwm, throttle_pwm) # fire the commands

                print(
                    f" [Follow] Marker@({marker_cx:.0f},{marker_cy:.0f}) "
                    f"Roll:{roll_pwm} Pitch:{pitch_pwm} Thr:{throttle_pwm} Alt:{alt:.2f}m",
                    end='\r'
                ) # compact live status line

            # ---- marker NOT found this frame ----
            else:
                if lost_timer is None: # if this is the first frame we lost it
                    lost_timer = time.time() # start the lost clock
                    print(f"\n[Follow] Marker lost. Holding position...") # log the loss

                # while lost: send neutral roll/pitch (stop drifting) but keep altitude hold
                alt = get_lidar_alt(master) # still need lidar for throttle
                throttle_pwm = compute_throttle_hold(alt) # keep altitude
                set_rc_override(master, RC_CENTER, RC_CENTER, throttle_pwm) # hold position

                time_lost = time.time() - lost_timer # how long since we lost the marker
                print(f" [Follow] Searching... {time_lost:.1f}s", end='\r') # live lost timer

                if time_lost >= ARUCO_LOST_TIMEOUT: # if marker has been gone too long
                    print(f"\n[!] Marker lost for {ARUCO_LOST_TIMEOUT}s. Initiating landing.")
                    bridge.send_message("ARUCO LOST: LANDING") # broadcast the bad news
                    break # exit follow loop and land

            # ---- keep the loop running at FOLLOW_HZ ----
            elapsed = time.time() - loop_start # how long this cycle took
            sleep_time = loop_interval - elapsed # remaining time to hit target hz
            if sleep_time > 0: # if there is time left in this cycle
                time.sleep(sleep_time) # sleep the remainder

        # step 7: land and shutdown (same as mission4)
        print("\nLanding sequence engaged...")
        change_mode(master, "LAND") # switch to official land mode for graceful touchdown
        set_rc_override(master, RC_CENTER, RC_CENTER, 0, RC_CENTER) # release all overrides

        while True: # loop until we hit the floor
            alt = get_lidar_alt(master) # check lidar
            print(f" Land Alt: {alt:.2f}m", end='\r') # log altitude

            # check if the flight controller disarmed itself after touchdown
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("\nTouchdown confirmed. Motors stopped.")
                break # done
            time.sleep(0.5) # slower check loop

    except KeyboardInterrupt: # someone hit ctrl+c
        print("\n[!] Emergency: User Triggered Landing...")
        change_mode(master, "LAND") # force land mode immediately
        set_rc_override(master, RC_CENTER, RC_CENTER, 0, RC_CENTER) # release all overrides
        time.sleep(1) # wait for command to register
    finally: # always clean up
        cam.close() # close the zed camera cleanly (same pattern as camerauavtest)
        print("[ZED] ZED 2 camera closed.")
        bridge.stop() # close radio wire
        print("Aruco follower mission finalized.")

if __name__ == "__main__": # entry point
    main() # run it

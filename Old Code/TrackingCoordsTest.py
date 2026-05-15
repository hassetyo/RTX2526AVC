from pymavlink import mavutil # using the confirmed mavlink pattern instead of dronekit
import time # for timing and sleeps
import math # for NaN checks
import sys # for clean exits
#(uncomment)
#import v2v_bridge # our custom radio bridge talker
import cv2 # for aruco detection
import pyzed.sl as sl # zed 2 sdk on jetson nano

# uav mission 4 - autonomous flight (throttle override) + ugv circle
# this uses the pattern the user confirmed works (stabilize + rc override)

################################# config stuff i setup
# connection settings from your working test script
CONNECTION_STRING = "/dev/ttyACM0"   # drone wire (use COM4 if testing on windows)
BAUD_RATE = 57600                    # using the confirmed 57600 speed
ESP32_PORT = "/dev/ttyUSB0"          # the radio bridge usb wire

# mission params
TARGET_ALT = 1.3    # hover height in meters (4.2 ft)
CIRCLE_TIME = 18.0  # duration for the rover maneuvers

# throttle settings i tuned
THROTTLE_MIN = 1000   # motors off
THROTTLE_IDLE = 1150  # props spinning but no lift
THROTTLE_CLIMB = 1650 # power to lift off the floor
THROTTLE_HOVER = 1500 # rough middle ground for holding height

# aruco settings
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50  # marker dictionary type (change to match your printed marker)
ARUCO_SCAN_TIMEOUT = 30.0               # max seconds to wait for a marker before aborting mission

############################ the mavlink helpers i wrote

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

def disarm_drone(master): # stops the motors securely
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0 
    )
    print("Disarmed.") # log the safety

def set_throttle(master, pwm): # physically pushes the throttle via rc override
    # channel 3 is the throttle in ardupilot
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )

def get_lidar_alt(master): # checks the floor distance via lidar
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0) # wait for lidar packet
    if msg: # if we got a real message
        return msg.current_distance / 100.0 # return height in meters
    return 0.0 # return zero if lidar is dead

############################ aruco scan via zed 2

def scan_aruco_marker():
    """
    Opens the ZED 2 camera and scans for any ArUco marker from the configured dictionary.
    Blocks until a marker is found or ARUCO_SCAN_TIMEOUT seconds pass.
    Returns the detected marker ID (int) on success, or None on timeout.
    """
    print("[ArUco] Opening ZED 2 camera...") # log startup

    # init zed camera
    cam = sl.Camera() # create camera object
    init_params = sl.InitParameters() # default params
    init_params.camera_resolution = sl.RESOLUTION.HD720  # 720p is plenty for aruco and easier on the nano
    init_params.camera_fps = 30 # 30fps
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE # need depth for local mapping of arucos
    init_params.coordinate_units = sl.UNIT.METER # use meters for depth values

    status = cam.open(init_params) # open the camera
    if status != sl.ERROR_CODE.SUCCESS: # if it failed
        print(f"[ArUco] ZED open failed: {status}") # log the error
        return None # bail with no detection

    # build the aruco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE) # load marker dictionary
    aruco_params = cv2.aruco.DetectorParameters() # default detection params
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params) # build detector object

    # zed image container
    zed_image = sl.Mat() # mat to receive raw frame
    point_cloud = sl.Mat() # mat to receive depth data

    detected_id = [] # will hold the found marker id
    scan_start = time.time() # start the timeout clock
    print(f"[ArUco] Scanning for marker... (timeout: {ARUCO_SCAN_TIMEOUT}s)") # log scan start

    while True: # scan loop
        x, y, z = float('nan'), float('nan'), float('nan') # Initialize every frame
        # check timeout first
        elapsed = time.time() - scan_start # seconds spent scanning
        if elapsed >= ARUCO_SCAN_TIMEOUT: # if we ran out of time
            print(f"[ArUco] Timeout after {ARUCO_SCAN_TIMEOUT}s. No marker found.") # log fail
            break # exit loop with detected_id = None

        if cam.grab() == sl.ERROR_CODE.SUCCESS: # if we got a fresh frame
            cam.retrieve_image(zed_image, sl.VIEW.LEFT) # pull left camera image
            cam.retrieve_measure(point_cloud, sl.MEASURE.XYZ) # pull depth data

            frame = zed_image.get_data() # raw numpy array (BGRA from ZED)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) # strip alpha channel for opencv
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) # aruco works best on grayscale

            corners, ids, rejected = detector.detectMarkers(gray) # run the detector

            if ids is not None and len(ids) > 0: # if we found at least one marker
                for i in range(len(ids)): # loop through all detections
                     m_id = int(ids[i][0]) # grab the marker id

                     #calculate the center pixel of the marker
                     c = corners[i][0] # corners of the detected marker
                     cX = int((c[0][0] + c[2][0]) / 2.0) # average x of corners
                     cY = int((c[0][1] + c[2][1]) / 2.0) # average y of corners

                     # get the depth at the center pixel
                     err, point3D = point_cloud.get_value(cX, cY) # get the depth at the center pixel
                     if err == sl.ERROR_CODE.SUCCESS: # if we got a valid depth point
                         x, y, z = point3D[0], point3D[1], point3D[2] # unpack the 3D coordinates

                     #NaN detection
                     if math.isnan(x) or math.isnan(y) or math.isnan(z): # if the depth point is invalid
                        print(f"[!] ID {m_id} found, but 3D data is NaN (depth lost).")
                     else:
                        detected_id.append(m_id) # save the id
                        print(f"[ArUco] Marker detected! ID: {m_id} at Local Coords: X:{x:.2f}m, Y:{y:.2f}m, Z:{z:.2f}m")
                        break # exit FOR loop after first valid detection
                if detected_id: # if we found at least one valid marker
                    break # exit the scan loop (WHILE loop)

            # progress dot so we know its alive
            print(f"[ArUco] Scanning... {elapsed:.1f}s", end='\r') # live timer

        time.sleep(0.05) # ~20fps check rate, not hammering the nano

    cam.close() # always close the camera cleanly to free resources
    print("[ArUco] ZED 2 camera closed.") # log close
    return detected_id # return found id or None

#################### the main mission 4 logic

def main(): # the main boss function
    print("==========================================") # header
    print("   UAV Coordinates Test   ") # title
    print("==========================================") # footer

    # step 1: connect to the wires
    print(f"Connecting to Drone: {CONNECTION_STRING}...") # login
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE) # open link
    master.wait_heartbeat() # wait for buzz
    print("Drone Heartbeat OK.") # success

    # we dont need to connect to rover for this test (uncomment)
    '''
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # radio bridge
    try: # try to open radio
        bridge.connect() # open serial wires
        bridge.send_message("MISSION 4: MAVLINK MODE START") # yell over air
    except: # if radio is missing
        print("Radio Bridge Fail.") # log fail
        return # bail
    '''

    try: # wrapping mission in safety block
        # step 2: arm drone and scan aruco before doing anything else
        change_mode(master, "STABILIZE") # switch to stabilize for manual throttle control
        arm_drone(master) # start the props

        # ---- aruco scan gate ----
        # motors are now armed (spinning at idle). we scan for the marker before committing to flight.
        # if we timeout without finding one we disarm cleanly and abort the mission.
        marker_id = scan_aruco_marker() # open zed, scan, return id or None

        if not marker_id: # no marker found or depth data is NaN
            print("[!] ArUco marker detection failed. Aborting mission and disarming.") # log abort
            disarm_drone(master) # kill the motors safely
            #(uncomment)
            #bridge.stop() # close radio
            return # exit mission
        for i in range(len(marker_id)):
         print(f"[ArUco] Marker ID {marker_id[i]} confirmed. Continuing mission...") # log go-ahead
         #(uncomment)
         #bridge.send_message(f"ARUCO GATE PASSED: MARKER {marker_id[i]}") # broadcast over radio
         time.sleep(0.5) # brief pause before next steps
        # ---- end aruco gate ----

        # we dont need to connect to rover for this test (uncomment)
         '''
        # step 3: takeoff sequence (using your working pattern)
        print("Climbing to 1.3m...") # log the climb
        while True: # loop until target height
            alt = get_lidar_alt(master) # check lidar
            print(f" Altitude: {alt:.2f}m", end='\r') # log height
            if alt >= TARGET_ALT: # if we hit the hover point
                set_throttle(master, THROTTLE_HOVER) # pull back to hover power
                print(f"\nHover altitude reached: {alt:.2f}m") # declare success
                break # break the climb
            set_throttle(master, THROTTLE_CLIMB) # keep pushing up
            time.sleep(0.1) # quick loop

        # step 4: sync with ground rover
        print("Waiting for UGV sync...") # logging wait
        while True: # loop until radio sync
            set_throttle(master, THROTTLE_HOVER) # MUST keep sending hover pulse or it crashes
            data = bridge.get_telemetry() # pull from mailbox
            if data: # if we got a packet
                print("UGV Ready. Initiating Circles.") # log coordination
                break # done
            time.sleep(0.2) # check 5 times a second to keep rc heartbeat alive

        # step 5: command the rover work
        bridge.send_command(cmdSeq=400, cmd=v2v_bridge.CMD_CIRCLE, estop=0) # blast command
        
        start_t = time.time() # start clock
        while (time.time() - start_t) < CIRCLE_TIME: # loop for duration
            data = bridge.get_telemetry() # check status
            if data: # if real status
                print(f" UGV Speed: {data[2]:.2f} m/s", end='\r') # log rover stats

            # small "crude" altitude hold logic i added
            alt = get_lidar_alt(master) # check lidar
            if alt < TARGET_ALT - 0.1: # if we are sinking
                set_throttle(master, THROTTLE_HOVER + 100) # give it more juice
            elif alt > TARGET_ALT + 0.1: # if we are drifting too high
                set_throttle(master, THROTTLE_HOVER - 100) # cut power
            else: # if we are golden
                set_throttle(master, THROTTLE_HOVER) # keep steady

            time.sleep(0.1) # 10hz loop
         '''

        # step 6: land and shutdown
        print("\nLanding sequence engaged...") # start descent
        change_mode(master, "LAND") # switch to official land mode for graceful touchdown
        set_throttle(master, 0) # release throttle override so autopilot takes over

        while True: # loop until we hit the floor
            alt = get_lidar_alt(master) # check lidar
            print(f" Land Alt: {alt:.2f}m", end='\r') # log altitude

            # checking if the drone disarmed itself (autopilot does this after landing)
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("\nTouchdown confirmed. Motors stopped.") # log success
                break # exit
            time.sleep(0.5) # slower loop for checking

    except KeyboardInterrupt: # someone hit ctrl+c
        print("\n[!] Emergency: User Triggered Landing...") # abort log
        change_mode(master, "LAND") # force land mode immediately
        set_throttle(master, 0) # release override
        time.sleep(1) # wait for command to hit
    finally: # final chores
        #(uncomment)
        #bridge.stop() # close radio wire
        print("Mission finalized.") # end log

if __name__ == "__main__": # entry point
    main() # run it
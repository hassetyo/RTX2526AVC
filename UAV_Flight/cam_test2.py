from pymavlink import mavutil  # using the confirmed mavlink pattern instead of dronekit
import time                    # for timing and sleeps
import sys                     # for clean exits
import v2v_bridge              # our custom radio bridge talker
import cv2                     # for aruco detection
import pyzed.sl as sl          # zed 2 sdk on jetson nano

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
ARUCO_SCAN_TIMEOUT = 30.0                # max seconds to wait for a marker before aborting mission

WINDOW_NAME = "ZED ArUco View"

############################ the mavlink helpers i wrote

def change_mode(master, mode: str):  # changes the flight controller mode
    mapping = master.mode_mapping()  # ask for the list of modes
    if mode not in mapping:          # if the mode is fake
        print(f"Unknown mode '{mode}'")  # log the error
        return False                 # bail out
    mode_id = mapping[mode]          # find the secret mode id
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )  # blast it
    print(f"Mode set: {mode}")       # log the change
    time.sleep(1)                    # wait for the mode to settle
    return True

def arm_drone(master):  # engages the scary drone motors
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Arming motors...")  # log the arming
    time.sleep(2)              # wait for the spinning to start

def disarm_drone(master):  # stops the motors securely
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Disarmed.")  # log the safety

def set_throttle(master, pwm):  # physically pushes the throttle via rc override
    # channel 3 is the throttle in ardupilot
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )

def get_lidar_alt(master):  # checks the floor distance via lidar
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0)  # wait for lidar packet
    if msg:  # if we got a real message
        return msg.current_distance / 100.0  # return height in meters
    return 0.0  # return zero if lidar is dead

############################ zed + popup helpers

def open_zed_camera():
    print("[ZED] Opening camera...")

    cam = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    status = cam.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ZED] Open failed: {status}")
        return None

    print("[ZED] Camera opened.")
    return cam

def create_aruco_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    aruco_params = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

def get_zed_frame(cam):
    zed_image = sl.Mat()
    if cam.grab() != sl.ERROR_CODE.SUCCESS:
        return None
    cam.retrieve_image(zed_image, sl.VIEW.LEFT)
    frame = zed_image.get_data()
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame_bgr

def update_camera_popup(cam, detector=None, status_text="", alt_text="", marker_text=""):
    """
    Grab one frame, optionally run ArUco detection, draw overlays, and keep popup alive.
    Returns: frame_bgr, corners, ids, key
    """
    frame_bgr = get_zed_frame(cam)
    if frame_bgr is None:
        return None, None, None, -1

    corners, ids = None, None
    if detector is not None:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame_bgr, corners, ids)

    # text overlays
    y = 35
    if status_text:
        cv2.putText(frame_bgr, status_text, (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        y += 35

    if alt_text:
        cv2.putText(frame_bgr, alt_text, (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        y += 35

    if marker_text:
        cv2.putText(frame_bgr, marker_text, (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        y += 35

    cv2.putText(frame_bgr, "Press q to abort/quit", (20, frame_bgr.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow(WINDOW_NAME, frame_bgr)
    key = cv2.waitKey(1) & 0xFF

    return frame_bgr, corners, ids, key

############################ aruco scan via zed 2

def scan_aruco_marker(cam, detector):
    """
    Uses the already-open ZED 2 camera and scans for any ArUco marker from the configured dictionary.
    Keeps the popup open the whole time.
    Returns the detected marker ID (int) on success, or None on timeout.
    """
    detected_id = None
    scan_start = time.time()
    print(f"[ArUco] Scanning for marker... (timeout: {ARUCO_SCAN_TIMEOUT}s)")

    while True:
        elapsed = time.time() - scan_start
        if elapsed >= ARUCO_SCAN_TIMEOUT:
            print(f"[ArUco] Timeout after {ARUCO_SCAN_TIMEOUT}s. No marker found.")
            break

        frame_bgr, corners, ids, key = update_camera_popup(
            cam,
            detector=detector,
            status_text=f"[ArUco] Scanning... {elapsed:.1f}s",
            marker_text="Looking for marker..."
        )

        if key == ord('q'):
            print("[ArUco] User quit scan window.")
            break

        if ids is not None and len(ids) > 0:
            detected_id = int(ids[0][0])
            print(f"[ArUco] Marker detected! ID: {detected_id}")

            # redraw one more time with detected id
            update_camera_popup(
                cam,
                detector=detector,
                status_text="[ArUco] Marker detected!",
                marker_text=f"Detected ID: {detected_id}"
            )
            break

        time.sleep(0.05)

    return detected_id

#################### the main mission 4 logic

def main():  # the main boss function
    print("==========================================")  # header
    print("   UAV MISSION 4 - CONFIRMED MAVLINK PATTERN")  # title
    print("==========================================")  # footer

    cam = None
    bridge = None

    # step 1: connect to the wires
    print(f"Connecting to Drone: {CONNECTION_STRING}...")  # login
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)  # open link
    master.wait_heartbeat()  # wait for buzz
    print("Drone Heartbeat OK.")  # success

    try:
        # open camera once and keep popup alive the entire mission
        cam = open_zed_camera()
        if cam is None:
            return

        detector = create_aruco_detector()
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        # show a live frame immediately
        update_camera_popup(
            cam,
            detector=detector,
            status_text="Camera online",
            marker_text="Preparing mission..."
        )

        bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")  # radio bridge
        try:  # try to open radio
            bridge.connect()  # open serial wires
            bridge.send_message("MISSION 4: MAVLINK MODE START")  # yell over air
        except Exception as e:  # if radio is missing
            print(f"Radio Bridge Fail: {e}")  # log fail
            return  # bail

        # step 2: arm drone and scan aruco before doing anything else
        if not change_mode(master, "STABILIZE"):
            return
        arm_drone(master)  # start the props

        # ---- aruco scan gate ----
        # motors are now armed (spinning at idle). we scan for the marker before committing to flight.
        # if we timeout without finding one we disarm cleanly and abort the mission.
        marker_id = scan_aruco_marker(cam, detector)  # use already-open cam/popup

        if marker_id is None:  # if we never found a marker
            print("[!] No ArUco marker detected. Aborting mission and disarming.")  # log abort
            disarm_drone(master)  # kill the motors safely
            return  # exit mission

        print(f"[ArUco] Marker ID {marker_id} confirmed. Continuing mission...")  # log go-ahead
        bridge.send_message(f"ARUCO GATE PASSED: MARKER {marker_id}")  # broadcast over radio
        # ---- end aruco gate ----

        # step 3: takeoff sequence (using your working pattern)
        print("Climbing to 1.3m...")  # log the climb
        while True:  # loop until target height
            alt = get_lidar_alt(master)  # check lidar
            print(f" Altitude: {alt:.2f}m", end='\r')  # log height

            _, _, _, key = update_camera_popup(
                cam,
                detector=detector,
                status_text="Takeoff phase",
                alt_text=f"Altitude: {alt:.2f}m",
                marker_text=f"Marker locked: ID {marker_id}"
            )
            if key == ord('q'):
                print("\n[!] User aborted during takeoff.")
                change_mode(master, "LAND")
                set_throttle(master, 0)
                time.sleep(1)
                return

            if alt >= TARGET_ALT:  # if we hit the hover point
                set_throttle(master, THROTTLE_HOVER)  # pull back to hover power
                print(f"\nHover altitude reached: {alt:.2f}m")  # declare success
                break  # break the climb

            set_throttle(master, THROTTLE_CLIMB)  # keep pushing up
            time.sleep(0.1)  # quick loop

        # step 4: sync with ground rover
        print("Waiting for UGV sync...")  # logging wait
        while True:  # loop until radio sync
            set_throttle(master, THROTTLE_HOVER)  # MUST keep sending hover pulse or it crashes
            data = bridge.get_telemetry()  # pull from mailbox

            _, _, _, key = update_camera_popup(
                cam,
                detector=detector,
                status_text="Waiting for UGV sync...",
                alt_text=f"Altitude: {get_lidar_alt(master):.2f}m",
                marker_text=f"Marker locked: ID {marker_id}"
            )
            if key == ord('q'):
                print("\n[!] User aborted during sync wait.")
                change_mode(master, "LAND")
                set_throttle(master, 0)
                time.sleep(1)
                return

            if data:  # if we got a packet
                print("UGV Ready. Initiating Circles.")  # log coordination
                break  # done
            time.sleep(0.2)  # check 5 times a second to keep rc heartbeat alive

        # step 5: command the rover work
        bridge.send_command(cmdSeq=400, cmd=v2v_bridge.CMD_CIRCLE, estop=0)  # blast command

        start_t = time.time()  # start clock
        while (time.time() - start_t) < CIRCLE_TIME:  # loop for duration
            data = bridge.get_telemetry()  # check status
            if data:  # if real status
                print(f" UGV Speed: {data[2]:.2f} m/s", end='\r')  # log rover stats

            # small "crude" altitude hold logic i added
            alt = get_lidar_alt(master)  # check lidar
            if alt < TARGET_ALT - 0.1:  # if we are sinking
                set_throttle(master, THROTTLE_HOVER + 100)  # give it more juice
            elif alt > TARGET_ALT + 0.1:  # if we are drifting too high
                set_throttle(master, THROTTLE_HOVER - 100)  # cut power
            else:  # if we are golden
                set_throttle(master, THROTTLE_HOVER)  # keep steady

            ugv_text = "UGV Speed: N/A"
            if data:
                ugv_text = f"UGV Speed: {data[2]:.2f} m/s"

            _, _, _, key = update_camera_popup(
                cam,
                detector=detector,
                status_text="Circle phase running",
                alt_text=f"Altitude: {alt:.2f}m",
                marker_text=f"Marker ID {marker_id} | {ugv_text}"
            )
            if key == ord('q'):
                print("\n[!] User aborted during circle phase.")
                break

            time.sleep(0.1)  # 10hz loop

        # step 6: land and shutdown
        print("\nLanding sequence engaged...")  # start descent
        change_mode(master, "LAND")  # switch to official land mode for graceful touchdown
        set_throttle(master, 0)  # release throttle override so autopilot takes over

        while True:  # loop until we hit the floor
            alt = get_lidar_alt(master)  # check lidar
            print(f" Land Alt: {alt:.2f}m", end='\r')  # log altitude

            _, _, _, key = update_camera_popup(
                cam,
                detector=detector,
                status_text="Landing...",
                alt_text=f"Altitude: {alt:.2f}m",
                marker_text=f"Landing after marker ID {marker_id}"
            )
            if key == ord('q'):
                print("\n[Landing] q pressed, continuing landing.")

            # checking if the drone disarmed itself (autopilot does this after landing)
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("\nTouchdown confirmed. Motors stopped.")  # log success
                break  # exit
            time.sleep(0.5)  # slower loop for checking

    except KeyboardInterrupt:  # someone hit ctrl+c
        print("\n[!] Emergency: User Triggered Landing...")  # abort log
        try:
            change_mode(master, "LAND")  # force land mode immediately
            set_throttle(master, 0)      # release override
            time.sleep(1)                # wait for command to hit
        except Exception:
            pass
    finally:  # final chores
        if bridge is not None:
            try:
                bridge.stop()  # close radio wire
            except Exception:
                pass

        if cam is not None:
            try:
                cam.close()
            except Exception:
                pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        print("Mission finalized.")  # end log

if __name__ == "__main__":  # entry point
    main()  # run it

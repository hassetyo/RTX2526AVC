from pymavlink import mavutil # using the confirmed mavlink pattern instead of dronekit
import time # for timing and sleeps
import math # for NaN checks on depth data
import sys # for clean exits
import v2v_bridge # our custom radio bridge talker
import cv2 # for aruco detection and preview window
import pyzed.sl as sl # zed 2 sdk on jetson nano
import numpy as np # math for pixel offsets and vector transforms

# uav aruco marker follower
# uses the zed 2 camera to detect an aruco marker and keeps the drone hovering over it
# if the marker moves, the drone follows it using rc channel overrides on pitch/roll
# built on top of the mission4 / camerauavtest confirmed patterns
# camera usage mirrors CameraUAVTest.py: PERFORMANCE depth mode + point cloud for real 3D coords

################################# config stuff i setup
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

# aruco settings (must match CameraUAVTest.py)
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50  # marker dictionary type (change to match your printed marker)
ARUCO_LOST_TIMEOUT = 3.0                 # seconds marker can be missing mid-follow before we land

# how many consecutive frames the marker must be visible before we lock on
# this prevents a single noisy false-positive from triggering the follow loop
ARUCO_CONFIRM_FRAMES = 3  # consecutive detections required to confirm the marker

# rc channel mapping for ardupilot (standard)
RC_ROLL_CH     = 1  # channel 1 = roll (left/right drift correction)
RC_PITCH_CH    = 2  # channel 2 = pitch (forward/back drift correction)
RC_THROTTLE_CH = 3  # channel 3 = throttle (altitude hold)
RC_YAW_CH      = 4  # channel 4 = yaw (rotation - we lock this at center)

RC_CENTER = 1500  # neutral stick position (no movement)
RC_MIN    = 1000  # full deflection one way
RC_MAX    = 2000  # full deflection the other way

# pid controller gains for the position hold loop
KP_ROLL  = 0.05   # proportional gain for left/right correction
KP_PITCH = 0.05   # proportional gain for forward/back correction
MAX_NUDGE = 150   # maximum rc pwm offset from center (keeps it smooth, prevents flips)

PIXEL_DEADBAND = 20  # pixels - ignore error smaller than this to prevent micro-jitter

FOLLOW_HZ = 10  # how many times per second we update rc commands

WINDOW_NAME = "ZED ArUco Follower"  # title of the opencv display window

############################ the mavlink helpers (same as mission4)

def change_mode(master, mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"Unknown mode '{mode}'")
        return
    mode_id = mapping[mode]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    print(f"Mode set: {mode}")
    time.sleep(1)

def arm_drone(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Arming motors...")
    time.sleep(2)

def disarm_drone(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Disarmed.")

def set_throttle(master, pwm):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )

def set_rc_override(master, roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm=RC_CENTER):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        roll_pwm, pitch_pwm, throttle_pwm, yaw_pwm,
        0, 0, 0, 0
    )

def get_lidar_alt(master, blocking=False):
    # FIX: blocking defaults to FALSE now so the follow loop never stalls waiting for lidar.
    # The old blocking=True with timeout=1.0 could freeze the entire loop for a full second,
    # starving the RC override heartbeat and causing ArduPilot to drop control.
    # Non-blocking returns the most recent message if one is ready, or 0.0 if not.
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=blocking, timeout=0.05)
    if msg:
        return msg.current_distance / 100.0
    return 0.0

############################ zed 2 camera init (mirrors CameraUAVTest.py)

def open_zed_camera():
    print("[ZED] Opening ZED 2 camera...")
    cam = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER

    status = cam.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ZED] ZED open failed: {status}")
        return None, None, None, None

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    zed_image = sl.Mat()
    point_cloud = sl.Mat()

    print("[ZED] Camera open and detector ready.")
    return cam, detector, zed_image, point_cloud

############################ the pid correction math

def compute_rc_correction(marker_center_x, marker_center_y, frame_w, frame_h):
    cx = frame_w / 2.0
    cy = frame_h / 2.0
    err_x = marker_center_x - cx
    err_y = marker_center_y - cy

    if abs(err_x) < PIXEL_DEADBAND:
        err_x = 0.0
    if abs(err_y) < PIXEL_DEADBAND:
        err_y = 0.0

    roll_nudge  = max(-MAX_NUDGE, min(MAX_NUDGE, KP_ROLL  * err_x))
    pitch_nudge = max(-MAX_NUDGE, min(MAX_NUDGE, KP_PITCH * err_y))

    return int(RC_CENTER + roll_nudge), int(RC_CENTER + pitch_nudge)

def get_marker_center(corners):
    c = corners[0][0]
    cX = int((c[0][0] + c[2][0]) / 2.0)
    cY = int((c[0][1] + c[2][1]) / 2.0)
    return cX, cY

############################ altitude hold helper

def compute_throttle_hold(current_alt):
    if current_alt < TARGET_ALT - 0.1:
        return THROTTLE_HOVER + 100
    elif current_alt > TARGET_ALT + 0.1:
        return THROTTLE_HOVER - 100
    else:
        return THROTTLE_HOVER

############################ overlay drawing for the preview window

def draw_overlay(frame_bgr, corners, ids, point_cloud, frame_w, frame_h, status_text):
    display = frame_bgr.copy()

    # red crosshair at the frame center
    cv2.drawMarker(display, (frame_w // 2, frame_h // 2), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)

    if ids is not None and len(ids) > 0:
        for i in range(len(ids)):
            m_id = int(ids[i][0])
            c = corners[i][0]

            pts = c.reshape((-1, 1, 2)).astype(np.int32)
            cv2.polylines(display, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            cX = int((c[0][0] + c[2][0]) / 2.0)
            cY = int((c[0][1] + c[2][1]) / 2.0)

            cv2.drawMarker(display, (cX, cY), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

            # depth is optional for display - we try but dont require it
            coord_label = f"ID:{m_id}"
            if point_cloud is not None:
                err, point3D = point_cloud.get_value(cX, cY)
                if err == sl.ERROR_CODE.SUCCESS and not math.isnan(point3D[0]):
                    x, y, z = point3D[0], point3D[1], point3D[2]
                    coord_label = f"ID:{m_id} X:{x:.2f}m Y:{y:.2f}m Z:{z:.2f}m"
                else:
                    coord_label = f"ID:{m_id} depth:N/A"

            cv2.putText(display, coord_label, (cX - 100, cY - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    cv2.rectangle(display, (0, 0), (frame_w, 28), (0, 0, 0), -1)
    cv2.putText(display, status_text, (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return display

#################### the main follow mission

def main():
    print("==========================================")
    print("   UAV ARUCO MARKER FOLLOWER")
    print("==========================================")

    print(f"Connecting to Drone: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print("Drone Heartbeat OK.")

    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        bridge.send_message("ARUCO FOLLOWER: START")
    except:
        print("Radio Bridge Fail.")
        return

    cam, detector, zed_image, point_cloud = open_zed_camera()
    if cam is None:
        print("[!] ZED camera failed to open. Aborting.")
        bridge.stop()
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    frame_w, frame_h = 1280, 720
    if cam.grab() == sl.ERROR_CODE.SUCCESS:
        cam.retrieve_image(zed_image, sl.VIEW.LEFT)
        frame = zed_image.get_data()
        frame_h, frame_w = frame.shape[:2]
        print(f"[ZED] Frame size confirmed: {frame_w}x{frame_h}")

    try:
        # arm and takeoff
        change_mode(master, "STABILIZE")
        arm_drone(master)

        print("Climbing to target altitude...")
        while True:
            alt = get_lidar_alt(master, blocking=True)  # blocking is OK here, we are not time-critical yet

            if cam.grab() == sl.ERROR_CODE.SUCCESS:
                cam.retrieve_image(zed_image, sl.VIEW.LEFT)
                frame = zed_image.get_data()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                display = draw_overlay(frame_bgr, None, None, None, frame_w, frame_h,
                                       f"TAKEOFF  Alt:{alt:.2f}m  Target:{TARGET_ALT}m")
                cv2.imshow(WINDOW_NAME, display)
                cv2.waitKey(1)

            print(f" Altitude: {alt:.2f}m", end='\r')

            if alt >= TARGET_ALT:
                set_throttle(master, THROTTLE_HOVER)
                print(f"\nHover altitude reached: {alt:.2f}m")
                break
            set_throttle(master, THROTTLE_CLIMB)
            time.sleep(0.1)

        # -----------------------------------------------------------------------
        # MARKER SCAN LOOP
        # FIX: The old code only confirmed a marker if ZED point cloud depth was
        # non-NaN at that exact pixel. Depth can be NaN for many reasons:
        # wrong distance, lighting, marker surface, edge of depth range etc.
        # This caused it to see the marker visually but loop forever printing
        # "visible but 3D depth is NaN - still searching".
        #
        # Fix: we now confirm the marker on VISUAL detection alone (corners + ids).
        # Depth is still retrieved and shown in the overlay, but it does NOT gate
        # the lock-on. We require ARUCO_CONFIRM_FRAMES consecutive detections to
        # avoid locking on to a single noisy false positive.
        # -----------------------------------------------------------------------
        print("[ArUco] Hovering and waiting for marker to appear...")
        bridge.send_message("ARUCO FOLLOWER: WAITING FOR MARKER")

        marker_id = None
        confirm_count = 0     # consecutive frames the marker has been visible
        scan_start = time.time()

        while marker_id is None:
            # keep throttle alive every cycle
            alt = get_lidar_alt(master)  # non-blocking, fast
            throttle_pwm = compute_throttle_hold(alt)
            set_rc_override(master, RC_CENTER, RC_CENTER, throttle_pwm)

            if cam.grab() == sl.ERROR_CODE.SUCCESS:
                cam.retrieve_image(zed_image, sl.VIEW.LEFT)
                cam.retrieve_measure(point_cloud, sl.MEASURE.XYZ)  # still pull depth for display

                frame = zed_image.get_data()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

                corners, ids, rejected = detector.detectMarkers(gray)

                if ids is not None and len(ids) > 0:
                    # marker is visible this frame - increment the confirmation counter
                    confirm_count += 1
                    candidate_id = int(ids[0][0])
                    cX = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2.0)
                    cY = int((corners[0][0][0][1] + corners[0][0][2][1]) / 2.0)

                    # try depth for display only - not required for lock-on
                    err, point3D = point_cloud.get_value(cX, cY)
                    if err == sl.ERROR_CODE.SUCCESS and not math.isnan(point3D[0]):
                        depth_str = f"X:{point3D[0]:.2f}m Y:{point3D[1]:.2f}m Z:{point3D[2]:.2f}m"
                    else:
                        depth_str = "depth:N/A"  # depth unavailable but we still accept visually

                    print(f" [ArUco] ID:{candidate_id} seen ({confirm_count}/{ARUCO_CONFIRM_FRAMES}) {depth_str}", end='\r')

                    if confirm_count >= ARUCO_CONFIRM_FRAMES:
                        # confirmed by consecutive visual detection - lock on
                        marker_id = candidate_id
                        print(f"\n[ArUco] Marker confirmed! ID:{marker_id}  {depth_str}")
                        bridge.send_message(f"ARUCO ACQUIRED: ID {marker_id}")

                else:
                    # lost sight this frame - reset the counter so we need consecutive frames again
                    if confirm_count > 0:
                        print(f"\n[ArUco] Marker disappeared mid-confirm, resetting counter...")
                    confirm_count = 0

                elapsed = time.time() - scan_start
                status = (f"ACQUIRED ID:{marker_id}" if marker_id
                          else f"SEARCHING  Alt:{alt:.2f}m  {elapsed:.1f}s  confirm:{confirm_count}/{ARUCO_CONFIRM_FRAMES}")
                display = draw_overlay(frame_bgr, corners, ids, point_cloud,
                                       frame_w, frame_h, status)
                cv2.imshow(WINDOW_NAME, display)
                cv2.waitKey(1)

            time.sleep(0.05)  # ~20fps scan rate

        # -----------------------------------------------------------------------
        # FOLLOW LOOP
        # -----------------------------------------------------------------------
        print("[Follow] Entering marker follow loop...")
        bridge.send_message("ARUCO FOLLOWER: TRACKING ACTIVE")

        lost_timer = None
        loop_interval = 1.0 / FOLLOW_HZ

        while True:
            loop_start = time.time()

            if cam.grab() != sl.ERROR_CODE.SUCCESS:
                print("[ZED] Frame grab failed, retrying...")
                time.sleep(0.05)
                continue

            cam.retrieve_image(zed_image, sl.VIEW.LEFT)
            cam.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

            frame = zed_image.get_data()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = detector.detectMarkers(gray)

            # always get lidar non-blocking so the loop never stalls
            alt = get_lidar_alt(master)  # non-blocking
            throttle_pwm = compute_throttle_hold(alt)

            if ids is not None and len(ids) > 0:
                lost_timer = None  # marker is back, reset lost clock

                cX, cY = get_marker_center(corners)

                err, point3D = point_cloud.get_value(cX, cY)
                if err == sl.ERROR_CODE.SUCCESS and not math.isnan(point3D[0]):
                    coord_str = f"X:{point3D[0]:.2f}m Y:{point3D[1]:.2f}m Z:{point3D[2]:.2f}m"
                else:
                    coord_str = "depth:N/A"

                roll_pwm, pitch_pwm = compute_rc_correction(cX, cY, frame_w, frame_h)
                set_rc_override(master, roll_pwm, pitch_pwm, throttle_pwm)

                print(
                    f" [Follow] ID:{int(ids[0][0])} px:({cX},{cY}) {coord_str} "
                    f"Roll:{roll_pwm} Pitch:{pitch_pwm} Thr:{throttle_pwm} Alt:{alt:.2f}m",
                    end='\r'
                )

                status_txt = f"TRACKING ID:{int(ids[0][0])}  {coord_str}  Alt:{alt:.2f}m"
                display = draw_overlay(frame_bgr, corners, ids, point_cloud,
                                       frame_w, frame_h, status_txt)

            else:
                if lost_timer is None:
                    lost_timer = time.time()
                    print(f"\n[Follow] Marker lost. Holding position...")

                # hold position, keep altitude
                set_rc_override(master, RC_CENTER, RC_CENTER, throttle_pwm)

                time_lost = time.time() - lost_timer
                print(f" [Follow] Searching... {time_lost:.1f}s", end='\r')

                display = draw_overlay(frame_bgr, None, None, point_cloud, frame_w, frame_h,
                                       f"SEARCHING  Alt:{alt:.2f}m  lost:{time_lost:.1f}s")

                if time_lost >= ARUCO_LOST_TIMEOUT:
                    print(f"\n[!] Marker lost for {ARUCO_LOST_TIMEOUT}s. Initiating landing.")
                    bridge.send_message("ARUCO LOST: LANDING")
                    break

            cv2.imshow(WINDOW_NAME, display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[!] Q pressed - User triggered landing.")
                break

            elapsed = time.time() - loop_start
            sleep_time = loop_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # land
        print("\nLanding sequence engaged...")
        change_mode(master, "LAND")
        set_rc_override(master, RC_CENTER, RC_CENTER, 0, RC_CENTER)

        while True:
            alt = get_lidar_alt(master)
            print(f" Land Alt: {alt:.2f}m", end='\r')

            if cam.grab() == sl.ERROR_CODE.SUCCESS:
                cam.retrieve_image(zed_image, sl.VIEW.LEFT)
                frame = zed_image.get_data()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                display = draw_overlay(frame_bgr, None, None, point_cloud,
                                       frame_w, frame_h, f"LANDING  Alt:{alt:.2f}m")
                cv2.imshow(WINDOW_NAME, display)
                cv2.waitKey(1)

            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("\nTouchdown confirmed. Motors stopped.")
                break
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[!] Emergency: User Triggered Landing...")
        change_mode(master, "LAND")
        set_rc_override(master, RC_CENTER, RC_CENTER, 0, RC_CENTER)
        time.sleep(1)
    finally:
        cv2.destroyAllWindows()
        cam.close()
        print("[ZED] ZED 2 camera closed.")
        bridge.stop()
        print("Aruco follower mission finalized.")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Challenge 1 - Autonomous UAV landing on moving UGV
Using:
- ZED X camera via ZED SDK Python API
- ArUco marker ID 5 on the UGV
- Smaller ArUco marker ID 6 nested for close-range precision
- ArduPilot GUIDED mode
- MAVLink body-frame velocity commands

Mission behavior:
1. Connect to the drone through MAVLink.
2. Open the ZED camera and read camera intrinsics from the SDK.
3. Detect ArUco markers, giving priority to small marker ID 6 before ID 5.
4. Estimate the marker pose relative to the camera.
5. Convert that pose into body-frame forward/right/down errors.
6. Switch the drone to GUIDED, arm, and take off.
7. After takeoff, follow the target for 10 seconds before beginning precision landing.
   - The 10-second timer starts only when marker ID 5 is first acquired.
   - During this follow phase, ID 6 still has priority over ID 5 for tracking if visible.
8. After the follow phase completes, begin precision descent only when centered enough.
9. Switch to LAND when low and tightly centered.

Important:
- No RC override.
- No manual stick corrections required.
- No descent if the marker is not visible.
- If the marker is lost, command zero velocity so the drone holds.
"""

import time
import math
import signal
import cv2
import numpy as np
from pymavlink import mavutil

# ZED SDK Python API
import pyzed.sl as sl


# ============================================================
# USER SETTINGS
# ============================================================

# MAVLink connection
MAVLINK_CONNECTION = "/dev/ttyACM0"
MAVLINK_BAUD = 921600

# Marker settings
MARKER_ID = 5
MARKER_SIZE_M = 0.3048          # 12 inches
SMALL_MARKER_ID = 6
SMALL_MARKER_SIZE_M = 0.1016    # 4 inches

# Camera mounting assumption
# True  = top of camera image points toward front of drone
# False = top of camera image points toward rear of drone
CAMERA_TOP_OF_IMAGE_IS_DRONE_FRONT = True

# Flip these if test results show axis directions are reversed
FORWARD_SIGN = 1.0
RIGHT_SIGN = 1.0

# Autonomous mission behavior
CRUISE_ALT_M = 2.0
TAKEOFF_TIMEOUT_S = 20.0
FOLLOW_BEFORE_LAND_S = 10.0

# Positioning / descent logic
CENTER_TOLERANCE_M = 0.08
FINAL_CENTER_TOLERANCE_M = 0.03
FINAL_SWITCH_TO_LAND_DOWN_M = 0.35

# Velocities
MAX_XY_SPEED_MPS = 0.40
DESCENT_SPEED_MPS = 0.12

# Proportional control gains
KP_FORWARD = 0.65
KP_RIGHT = 0.65

# Marker-loss / detection stability
MARKER_LOST_HOLD_S = 0.50
MIN_STABLE_FRAMES = 8

# ZED camera settings
ZED_RESOLUTION = sl.RESOLUTION.HD1080
ZED_FPS = 30

# Display
SHOW_DEBUG_WINDOW = True

# ============================================================
# END USER SETTINGS
# ============================================================


running = True


def clamp(value, low, high):
    return max(low, min(high, value))


def handle_sigint(sig, frame):
    global running
    running = False
    print("\n[INFO] Stopping script...")


signal.signal(signal.SIGINT, handle_sigint)


# ------------------------------------------------------------
# MAVLink helpers
# ------------------------------------------------------------

def connect_mavlink():
    print(f"[INFO] Connecting to MAVLink: {MAVLINK_CONNECTION}")
    master = mavutil.mavlink_connection(MAVLINK_CONNECTION, baud=MAVLINK_BAUD)
    master.wait_heartbeat()
    print("[INFO] MAVLink heartbeat received")
    print(f"[INFO] System ID: {master.target_system}, Component ID: {master.target_component}")
    return master


def get_mode_mapping(master):
    mapping = master.mode_mapping()
    if mapping is None:
        raise RuntimeError("Could not get mode mapping from ArduPilot")
    return mapping


def set_mode(master, mode_name):
    mapping = get_mode_mapping(master)
    if mode_name not in mapping:
        raise RuntimeError(f"Flight mode '{mode_name}' not available")
    mode_id = mapping[mode_name]
    master.set_mode(mode_id)
    print(f"[INFO] Requested mode: {mode_name}")


def arm_vehicle(master):
    print("[INFO] Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("[INFO] Vehicle armed")


def disarm_vehicle(master):
    print("[INFO] Disarming vehicle...")
    master.arducopter_disarm()
    time.sleep(1.0)


def disable_ugv_avoidance(master):
    """
    Optional mission-specific fix:
    Disable ArduPilot proximity/object avoidance so the UAV does not
    try to dodge the UGV when attempting to land on it.
    """
    print("[INFO] Force-disabling AVOID_ENABLE")
    try:
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            b'AVOID_ENABLE',
            0,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.5)
    except Exception:
        pass


def takeoff(master, altitude_m):
    print(f"[INFO] Taking off to {altitude_m:.2f} m")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0, altitude_m
    )


def send_body_velocity(master, vx, vy, vz):
    """
    Send body-frame velocity in GUIDED mode.

    vx = forward (m/s)
    vy = right   (m/s)
    vz = down    (m/s)

    Uses SET_POSITION_TARGET_LOCAL_NED in MAV_FRAME_BODY_NED.
    """
    frame_ned = mavutil.mavlink.MAV_FRAME_BODY_NED

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
        master.target_system,
        master.target_component,
        frame_ned,
        type_mask,
        0.0, 0.0, 0.0,
        vx, vy, vz,
        0.0, 0.0, 0.0,
        0.0, 0.0
    )


def get_relative_altitude_m(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg is None:
        return None
    return msg.relative_alt / 1000.0


def wait_until_altitude(master, target_alt_m, timeout_s):
    start = time.time()
    while time.time() - start < timeout_s and running:
        alt = get_relative_altitude_m(master)
        if alt is not None:
            print(f"[INFO] Current altitude: {alt:.2f} m")
            if alt >= target_alt_m * 0.85:
                print("[INFO] Takeoff altitude reached")
                return True
        time.sleep(0.2)
    return False


# ------------------------------------------------------------
# ZED helpers
# ------------------------------------------------------------

def open_zed():
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = ZED_RESOLUTION
    init_params.camera_fps = ZED_FPS
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to open ZED camera: {status}")

    runtime_params = sl.RuntimeParameters()
    image_mat = sl.Mat()

    cam_info = zed.get_camera_information()
    left_calib = cam_info.camera_configuration.calibration_parameters.left_cam

    camera_matrix = np.array([
        [left_calib.fx, 0.0, left_calib.cx],
        [0.0, left_calib.fy, left_calib.cy],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)

    dist_coeffs = np.array(left_calib.disto[:5], dtype=np.float32)

    print("[INFO] ZED camera opened")
    print("[INFO] Camera matrix:")
    print(camera_matrix)

    return zed, runtime_params, image_mat, camera_matrix, dist_coeffs


def get_bgr_frame(zed, runtime_params, image_mat):
    if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
        return None
    zed.retrieve_image(image_mat, sl.VIEW.LEFT)
    rgba = image_mat.get_data()
    bgr = cv2.cvtColor(rgba, cv2.COLOR_BGRA2BGR)
    return bgr


# ------------------------------------------------------------
# ArUco helpers
# ------------------------------------------------------------

def create_aruco_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)

    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)

        def detect_fn(frame):
            corners, ids, rejected = detector.detectMarkers(frame)
            return corners, ids, rejected
    else:
        def detect_fn(frame):
            corners, ids, rejected = cv2.aruco.detectMarkers(
                frame, dictionary, parameters=params
            )
            return corners, ids, rejected

    return detect_fn


def estimate_single_marker_pose(corners, ids, camera_matrix, dist_coeffs, wanted_id, marker_size_m):
    if ids is None or len(ids) == 0:
        return False, None, None

    ids_flat = ids.flatten()
    matches = np.where(ids_flat == wanted_id)[0]
    if len(matches) == 0:
        return False, None, None

    idx = int(matches[0])

    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size_m, camera_matrix, dist_coeffs
        )
        tvec = tvecs[idx].reshape(3)
        return True, tvec, corners[idx]

    obj_pts = np.array([
        [-marker_size_m / 2,  marker_size_m / 2, 0],
        [ marker_size_m / 2,  marker_size_m / 2, 0],
        [ marker_size_m / 2, -marker_size_m / 2, 0],
        [-marker_size_m / 2, -marker_size_m / 2, 0],
    ], dtype=np.float32)

    img_pts = corners[idx].reshape(4, 2).astype(np.float32)

    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, img_pts, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if not ok:
        return False, None, None

    return True, tvec.reshape(3), corners[idx]


def camera_pose_to_body_errors(tvec):
    """
    Convert OpenCV camera pose to body-frame landing errors.
    """
    x_cam, y_cam, z_cam = tvec

    right_error_m = RIGHT_SIGN * x_cam

    if CAMERA_TOP_OF_IMAGE_IS_DRONE_FRONT:
        forward_error_m = FORWARD_SIGN * (-y_cam)
    else:
        forward_error_m = FORWARD_SIGN * (y_cam)

    down_distance_m = z_cam

    return forward_error_m, right_error_m, down_distance_m


# ------------------------------------------------------------
# Mission logic
# ------------------------------------------------------------

def main():
    master = connect_mavlink()
    disable_ugv_avoidance(master)

    zed, runtime_params, image_mat, camera_matrix, dist_coeffs = open_zed()
    detect_markers = create_aruco_detector()

    print("\n[INFO] Starting autonomous Challenge 1 logic")
    print("[INFO] Switching to GUIDED, arming, and taking off")

    set_mode(master, "GUIDED")
    time.sleep(1.0)

    arm_vehicle(master)
    time.sleep(1.0)

    takeoff(master, CRUISE_ALT_M)
    reached = wait_until_altitude(master, CRUISE_ALT_M, TAKEOFF_TIMEOUT_S)
    if not reached:
        print("[WARN] Target altitude not confirmed in time; continuing carefully")

    print("[INFO] Starting visual tracking and autonomous approach")

    last_marker_seen_time = 0.0
    stable_frames = 0
    final_land_sent = False
    follow_start_time = None

    while running:
        frame = get_bgr_frame(zed, runtime_params, image_mat)
        if frame is None:
            continue

        corners, ids, _ = detect_markers(frame)

        # Priority 1: small marker ID 6
        used_id = SMALL_MARKER_ID
        found, tvec, marker_corners = estimate_single_marker_pose(
            corners, ids, camera_matrix, dist_coeffs,
            SMALL_MARKER_ID, SMALL_MARKER_SIZE_M
        )

        # Priority 2: larger marker ID 5
        if not found:
            used_id = MARKER_ID
            found, tvec, marker_corners = estimate_single_marker_pose(
                corners, ids, camera_matrix, dist_coeffs,
                MARKER_ID, MARKER_SIZE_M
            )

        now = time.time()

        follow_elapsed = 0.0
        follow_done = False

        if follow_start_time is not None:
            follow_elapsed = now - follow_start_time
            follow_done = follow_elapsed >= FOLLOW_BEFORE_LAND_S

        if found:
            last_marker_seen_time = now
            stable_frames += 1

            # Start the 10-second follow timer only when ID 5 is first acquired
            if follow_start_time is None and ids is not None:
                ids_flat = ids.flatten()
                if MARKER_ID in ids_flat:
                    follow_start_time = now
                    follow_elapsed = 0.0
                    follow_done = False
                    print(f"[INFO] Marker ID {MARKER_ID} acquired -> starting {FOLLOW_BEFORE_LAND_S:.1f}s follow phase")

            forward_error_m, right_error_m, down_distance_m = camera_pose_to_body_errors(tvec)
            xy_error_m = math.hypot(forward_error_m, right_error_m)

            vx = clamp(KP_FORWARD * forward_error_m, -MAX_XY_SPEED_MPS, MAX_XY_SPEED_MPS)
            vy = clamp(KP_RIGHT * right_error_m, -MAX_XY_SPEED_MPS, MAX_XY_SPEED_MPS)
            vz = 0.0

            if not final_land_sent:
                # Phase 1: follow only, no descent yet
                if not follow_done:
                    vz = 0.0
                    send_body_velocity(master, vx, vy, vz)

                # Phase 2: precision landing after follow phase is complete
                else:
                    if stable_frames >= MIN_STABLE_FRAMES and xy_error_m < CENTER_TOLERANCE_M:
                        if down_distance_m > FINAL_SWITCH_TO_LAND_DOWN_M:
                            vz = DESCENT_SPEED_MPS
                        else:
                            if xy_error_m < FINAL_CENTER_TOLERANCE_M:
                                print("[INFO] Follow complete, low and centered -> switching to LAND")
                                send_body_velocity(master, 0.0, 0.0, 0.0)
                                set_mode(master, "LAND")
                                final_land_sent = True
                            else:
                                vz = 0.0

                    if not final_land_sent:
                        send_body_velocity(master, vx, vy, vz)

            if SHOW_DEBUG_WINDOW:
                if marker_corners is not None:
                    cv2.aruco.drawDetectedMarkers(frame, [marker_corners], np.array([[used_id]]))

                text1 = f"marker={used_id} fwd_err={forward_error_m:+.2f}m right_err={right_error_m:+.2f}m"
                text2 = f"down={down_distance_m:.2f}m xy_err={xy_error_m:.2f}m stable={stable_frames}"

                if follow_start_time is None:
                    text3 = f"follow=waiting for ID {MARKER_ID}"
                else:
                    text3 = f"follow={follow_elapsed:.1f}/{FOLLOW_BEFORE_LAND_S:.1f}s"

                if final_land_sent:
                    mode_text = "LAND"
                elif not follow_done:
                    mode_text = "FOLLOW_PHASE"
                else:
                    mode_text = "PRECISION_LAND"

                cv2.putText(frame, text1, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                cv2.putText(frame, text2, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                cv2.putText(frame, text3, (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
                cv2.putText(frame, mode_text, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

        else:
            stable_frames = 0

            if not final_land_sent:
                send_body_velocity(master, 0.0, 0.0, 0.0)

            if SHOW_DEBUG_WINDOW:
                lost_time = now - last_marker_seen_time if last_marker_seen_time > 0 else 999.0
                cv2.putText(frame, f"MARKER LOST {lost_time:.2f}s", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                if follow_start_time is None:
                    cv2.putText(frame, f"waiting for ID {MARKER_ID} to start follow timer", (20, 65),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                else:
                    cv2.putText(frame, f"follow={follow_elapsed:.1f}/{FOLLOW_BEFORE_LAND_S:.1f}s", (20, 65),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if SHOW_DEBUG_WINDOW:
            cv2.imshow("Challenge1_ZED_X_Aruco", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        if not final_land_sent:
            if last_marker_seen_time > 0 and (now - last_marker_seen_time) > MARKER_LOST_HOLD_S:
                send_body_velocity(master, 0.0, 0.0, 0.0)

        time.sleep(0.05)

    print("[INFO] Exiting. Sending zero velocity before shutdown.")
    try:
        send_body_velocity(master, 0.0, 0.0, 0.0)
    except Exception:
        pass

    if SHOW_DEBUG_WINDOW:
        cv2.destroyAllWindows()

    zed.close()
    print("[INFO] Done")


if __name__ == "__main__":
    main()
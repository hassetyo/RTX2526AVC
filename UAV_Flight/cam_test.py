from pymavlink import mavutil
import time
import cv2
import cv2.aruco as aruco
import numpy as np

# keep old vision logic in its own file
from oldvisiontest import UAVVision


# =========================================================
# CONFIG
# =========================================================
CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE = 57600

CALIBRATION_FILE = "calibration_chessboard.yaml"
USE_ZED = True                 # REQUIRED by your request
MARKER_SIZE_M = 0.1
TARGET_MARKER_ID = 0

TARGET_ALT = 1.3
ALT_TOL = 0.10

THROTTLE_MIN = 1000
THROTTLE_IDLE = 1150
THROTTLE_CLIMB = 1650
THROTTLE_HOVER = 1500

# RC neutral values
RC_NEUTRAL = 1500

# correction tuning
ROLL_STEP = 70                 # side correction strength
PITCH_STEP = 70                # forward/back correction strength
THROTTLE_STEP = 100            # altitude correction strength
MAX_RC_DELTA = 120             # clamp for safety

# center deadband in pixels
CENTER_DEADBAND_X = 60
CENTER_DEADBAND_Y = 45

# loop timing
LOOP_DELAY = 0.08


# =========================================================
# MAVLINK HELPERS (mission 4 style)
# =========================================================
def change_mode(master, mode: str):
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
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming motors...")
    time.sleep(2)


def set_rc_override(master, roll=0, pitch=0, throttle=0, yaw=0):
    # channels: 1 roll, 2 pitch, 3 throttle, 4 yaw
    # 0 means "leave unchanged" in this style
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        roll, pitch, throttle, yaw,
        0, 0, 0, 0
    )


def clear_rc_override(master):
    set_rc_override(master, 0, 0, 0, 0)


def get_lidar_alt(master):
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    if msg:
        return msg.current_distance / 100.0
    return None


def clamp_rc(value):
    low = RC_NEUTRAL - MAX_RC_DELTA
    high = RC_NEUTRAL + MAX_RC_DELTA
    return max(low, min(high, value))


# =========================================================
# VISION HELPER
# =========================================================
def get_target_marker_info(vision):
    """
    Uses the oldvisiontest camera + detector flow without changing its logic.
    We grab a frame from its camera, run its detector, draw with its detector,
    and separately compute the target marker pixel center for centering control.
    """
    frame = vision.camera.get_frame()
    if frame is None:
        return None, None, None, None, None

    positions = vision.detector.detect(frame)
    annotated = vision.detector.draw_detections(frame.copy(), positions)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, vision.detector.aruco_dict)

    target_position = next(
        (p for p in positions if p.marker_id == vision.target_marker_id),
        None
    )

    target_center = None

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if int(marker_id) == vision.target_marker_id:
                corner = corners[i][0]
                center = np.mean(corner, axis=0)
                target_center = (int(center[0]), int(center[1]))
                break

    return frame, annotated, positions, target_position, target_center


def compute_tracking_rc(frame_shape, marker_center):
    """
    Convert marker offset from image center into roll/pitch RC corrections.
    """
    h, w = frame_shape[:2]
    cx = w // 2
    cy = h // 2

    mx, my = marker_center
    err_x = mx - cx
    err_y = my - cy

    roll_cmd = RC_NEUTRAL
    pitch_cmd = RC_NEUTRAL

    # marker right of center -> move right
    if err_x > CENTER_DEADBAND_X:
        roll_cmd = clamp_rc(RC_NEUTRAL + ROLL_STEP)
    elif err_x < -CENTER_DEADBAND_X:
        roll_cmd = clamp_rc(RC_NEUTRAL - ROLL_STEP)

    # marker below center -> move forward/back depending on your camera orientation
    # for a forward-facing camera:
    # marker appears low -> usually drone should move down/forward depending on setup.
    # here we use pitch only for horizontal image centering.
    if err_y > CENTER_DEADBAND_Y:
        pitch_cmd = clamp_rc(RC_NEUTRAL + PITCH_STEP)
    elif err_y < -CENTER_DEADBAND_Y:
        pitch_cmd = clamp_rc(RC_NEUTRAL - PITCH_STEP)

    return roll_cmd, pitch_cmd, err_x, err_y, (cx, cy)


def compute_hover_throttle(master):
    alt = get_lidar_alt(master)
    if alt is None:
        return THROTTLE_HOVER, None

    if alt < TARGET_ALT - ALT_TOL:
        return THROTTLE_HOVER + THROTTLE_STEP, alt
    elif alt > TARGET_ALT + ALT_TOL:
        return THROTTLE_HOVER - THROTTLE_STEP, alt
    else:
        return THROTTLE_HOVER, alt


# =========================================================
# MAIN
# =========================================================
def main():
    print("==========================================")
    print(" UAV VISION TRACKING MISSION")
    print(" Ctrl+C at any time to initiate LAND mode")
    print("==========================================")

    print(f"Connecting to drone: {CONNECTION_STRING}")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print("Heartbeat received.")

    print("Initializing vision with ZED camera...")
    vision = UAVVision(
        calibration_file=CALIBRATION_FILE,
        marker_size=MARKER_SIZE_M,
        use_zed=USE_ZED
    )
    vision.target_marker_id = TARGET_MARKER_ID

    try:
        # -------------------------------------------------
        # Arm and take off similar to mission 4
        # -------------------------------------------------
        change_mode(master, "STABILIZE")
        arm_drone(master)

        print(f"Climbing to {TARGET_ALT:.2f}m...")
        while True:
            alt = get_lidar_alt(master)
            if alt is not None:
                print(f"Altitude: {alt:.2f}m", end="\r")

            if alt is not None and alt >= TARGET_ALT:
                set_rc_override(master, roll=0, pitch=0, throttle=THROTTLE_HOVER, yaw=0)
                print(f"\nHover altitude reached: {alt:.2f}m")
                break

            set_rc_override(master, roll=0, pitch=0, throttle=THROTTLE_CLIMB, yaw=0)
            time.sleep(0.1)

        print("Hovering and scanning for ArUco marker...")
        print("Press Ctrl+C to initiate landing.")

        # -------------------------------------------------
        # Main hover + scan + tracking loop
        # -------------------------------------------------
        while True:
            frame, annotated, positions, target_pos, target_center = get_target_marker_info(vision)

            throttle_cmd, alt = compute_hover_throttle(master)
            roll_cmd = RC_NEUTRAL
            pitch_cmd = RC_NEUTRAL

            if frame is not None and annotated is not None:
                h, w = frame.shape[:2]
                img_center = (w // 2, h // 2)

                # draw camera center crosshair
                cv2.line(annotated, (img_center[0] - 20, img_center[1]), (img_center[0] + 20, img_center[1]), (255, 255, 0), 2)
                cv2.line(annotated, (img_center[0], img_center[1] - 20), (img_center[0], img_center[1] + 20), (255, 255, 0), 2)

                if target_center is not None:
                    roll_cmd, pitch_cmd, err_x, err_y, _ = compute_tracking_rc(frame.shape, target_center)

                    # draw target center and line to image center
                    cv2.circle(annotated, target_center, 6, (0, 255, 255), -1)
                    cv2.line(annotated, img_center, target_center, (0, 255, 255), 2)

                    cv2.putText(
                        annotated,
                        f"TARGET ID {TARGET_MARKER_ID}  err_x:{err_x}  err_y:{err_y}",
                        (10, h - 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA
                    )

                    if target_pos is not None:
                        cv2.putText(
                            annotated,
                            f"Marker XYZ(m): x={target_pos.x:.2f} y={target_pos.y:.2f} z={target_pos.z:.2f} d={target_pos.distance:.2f}",
                            (10, h - 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 255, 255),
                            2,
                            cv2.LINE_AA
                        )

                    print(
                        f"[TRACK] alt={alt if alt is not None else -1:.2f}  "
                        f"roll={roll_cmd} pitch={pitch_cmd} throttle={throttle_cmd}  "
                        f"marker_center={target_center}",
                        end="\r"
                    )
                else:
                    # no marker found: keep hover only
                    cv2.putText(
                        annotated,
                        "Target marker not found - holding hover",
                        (10, h - 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA
                    )

                    print(
                        f"[SCAN] alt={alt if alt is not None else -1:.2f}  "
                        f"holding hover - target not found",
                        end="\r"
                    )

                # keep popup open entire time
                cv2.imshow("UAV Vision Tracking", annotated)
                cv2.waitKey(1)

            # send RC every loop so hover stays active
            set_rc_override(
                master,
                roll=roll_cmd,
                pitch=pitch_cmd,
                throttle=throttle_cmd,
                yaw=0
            )

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\n\n[!] User interrupt received. Initiating LAND mode...")
        change_mode(master, "LAND")
        clear_rc_override(master)

        # keep camera window alive while vehicle is landing
        while True:
            alt = get_lidar_alt(master)

            frame, annotated, positions, target_pos, target_center = get_target_marker_info(vision)
            if annotated is not None:
                h, w = annotated.shape[:2]
                cv2.putText(
                    annotated,
                    "LAND MODE ACTIVE",
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    3,
                    cv2.LINE_AA
                )
                if alt is not None:
                    cv2.putText(
                        annotated,
                        f"Altitude: {alt:.2f}m",
                        (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA
                    )
                cv2.imshow("UAV Vision Tracking", annotated)
                cv2.waitKey(1)

            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Touchdown confirmed. Motors stopped.")
                break

            time.sleep(0.2)

    finally:
        try:
            clear_rc_override(master)
        except Exception:
            pass

        vision.close()
        print("Mission finished. Vision closed.")


if __name__ == "__main__":
    main()

import time
import math
from typing import Dict, Optional

import cv2
import cv2.aruco as aruco
import numpy as np

import v2v_bridge
from ArUcoDistance import CameraInterface, ArucoDistanceEstimator, MarkerPose


# =============================================================================
# CONFIG
# =============================================================================

# Bridge / radio
ESP32_PORT = "/dev/ttyUSB0"
ESP32_BAUD = 115200

# Camera
USE_ZED = True
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_FPS = 30
CALIBRATION_FILE = "calibration_chessboard.yaml"   # only used if USE_ZED=False

# ArUco
DICT_NAME = aruco.DICT_6X6_1000
MARKER_SIZE_M = 0.10
UGV_MARKER_ID = 5
DEST_MARKER_ID = 0

# UGV marker forward direction in marker coordinates.
# If the printed marker's TOP points forward on the UGV, keep this as +Y.
# If the turn logic is backwards, try [0.0, -1.0, 0.0].
UGV_FORWARD_AXIS_MARKER = np.array([0.0, 1.0, 0.0], dtype=np.float64)

# Camera ground-plane mapping.
# We use camera X-Z as the floor plane.
PLANE_X_INDEX = 0
PLANE_Y_INDEX = 2

# Turn tuning
ANGLE_TOL_DEG = 8.0
REALIGN_ANGLE_DEG = 15.0
TURN_STABLE_FRAMES = 5
TURN_CMD_RESEND_SEC = 0.50
STOP_CMD_RESEND_SEC = 0.40

# Forward tuning
FINAL_DIST_TOL_M = 0.15
MOVE_CMD_RESEND_SEC = 0.80
DONE_STABLE_FRAMES = 5

# Movement command choice
# Safer default: repeated 2ft steps
USE_LONG_FORWARD_WHEN_FAR = False
LONG_STEP_THRESHOLD_M = 2.5

# Display
WINDOW_NAME = "UAV UGV Face-and-Drive (Commands Only)"
SHOW_WINDOW = True

# Flip this if turn direction is reversed in real testing
POSITIVE_ANGLE_MEANS_TURN_RIGHT = True


# =============================================================================
# MATH HELPERS
# =============================================================================

def normalize_2d(v: np.ndarray) -> Optional[np.ndarray]:
    n = np.linalg.norm(v)
    if n < 1e-9:
        return None
    return v / n


def rotation_matrix_from_rvec(rvec: np.ndarray) -> np.ndarray:
    rmat, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    return rmat


def marker_forward_vector_on_ground(pose: MarkerPose) -> Optional[np.ndarray]:
    """
    Convert the UGV marker orientation into a 2D forward direction on the ground plane.
    """
    rmat = rotation_matrix_from_rvec(pose.rvec)
    forward_cam = (rmat @ UGV_FORWARD_AXIS_MARKER.reshape(3, 1)).reshape(3)

    ground_vec = np.array(
        [forward_cam[PLANE_X_INDEX], forward_cam[PLANE_Y_INDEX]],
        dtype=np.float64,
    )
    return normalize_2d(ground_vec)


def ugv_to_dest_vector_on_ground(ugv_pose: MarkerPose, dest_pose: MarkerPose):
    diff = dest_pose.tvec.reshape(3) - ugv_pose.tvec.reshape(3)
    dx = float(diff[PLANE_X_INDEX])
    dy = float(diff[PLANE_Y_INDEX])
    vec = np.array([dx, dy], dtype=np.float64)
    return vec, dx, dy


def signed_angle_deg(v_from: np.ndarray, v_to: np.ndarray) -> float:
    a = normalize_2d(v_from)
    b = normalize_2d(v_to)
    if a is None or b is None:
        return 0.0

    cross = a[0] * b[1] - a[1] * b[0]
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return math.degrees(math.atan2(cross, dot))


def choose_turn_cmd(angle_deg: float) -> int:
    if POSITIVE_ANGLE_MEANS_TURN_RIGHT:
        return v2v_bridge.CMD_TURN_RIGHT if angle_deg > 0 else v2v_bridge.CMD_TURN_LEFT
    return v2v_bridge.CMD_TURN_LEFT if angle_deg > 0 else v2v_bridge.CMD_TURN_RIGHT


def choose_forward_cmd(dist_m: float) -> int:
    if USE_LONG_FORWARD_WHEN_FAR and dist_m >= LONG_STEP_THRESHOLD_M:
        return v2v_bridge.CMD_MOVE_FORWARD
    return v2v_bridge.CMD_MOVE_2FT


# =============================================================================
# DISPLAY
# =============================================================================

def draw_overlay(
    frame: np.ndarray,
    estimator: ArucoDistanceEstimator,
    poses: Dict[int, MarkerPose],
    phase: str,
    status: str,
    dist_m: Optional[float],
    heading_err_deg: Optional[float],
):
    out = frame.copy()
    out = estimator.draw_markers(out, poses)

    ugv_pose = poses.get(UGV_MARKER_ID)
    dest_pose = poses.get(DEST_MARKER_ID)

    if ugv_pose is not None and dest_pose is not None:
        cv2.line(out, ugv_pose.center_px, dest_pose.center_px, (0, 255, 255), 2)

        fwd = marker_forward_vector_on_ground(ugv_pose)
        if fwd is not None:
            cx, cy = ugv_pose.center_px
            end_pt = (int(cx + fwd[0] * 120), int(cy + fwd[1] * 120))
            cv2.arrowedLine(out, (cx, cy), end_pt, (255, 0, 255), 3, tipLength=0.2)

    lines = [
        f"Phase: {phase}",
        status,
        f"UGV={UGV_MARKER_ID} DEST={DEST_MARKER_ID}",
        f"distance={dist_m:.3f} m" if dist_m is not None else "distance=---",
        f"heading_err={heading_err_deg:.2f} deg" if heading_err_deg is not None else "heading_err=---",
    ]

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.65
    thick = 2
    pad = 10
    line_h = 28

    max_w = 0
    for line in lines:
        (tw, _), _ = cv2.getTextSize(line, font, scale, thick)
        max_w = max(max_w, tw)

    box_w = max_w + pad * 2
    box_h = len(lines) * line_h + pad * 2

    overlay = out.copy()
    cv2.rectangle(overlay, (10, 10), (10 + box_w, 10 + box_h), (0, 0, 0), -1)
    out = cv2.addWeighted(overlay, 0.55, out, 0.45, 0)

    y = 10 + pad + 20
    for line in lines:
        cv2.putText(out, line, (20, y), font, scale, (255, 255, 255), thick, cv2.LINE_AA)
        y += line_h

    return out


# =============================================================================
# MAIN
# =============================================================================

def main():
    print("===================================================")
    print(" UAV UGV FACE-AND-DRIVE (COMMANDS ONLY)")
    print(" Marker 5 turns to face marker 0, then advances")
    print(" using only existing movement commands")
    print("===================================================")
    print("Press q to quit.\n")

    cam = CameraInterface(
        use_zed=USE_ZED,
        camera_index=CAMERA_INDEX,
        width=FRAME_WIDTH,
        height=FRAME_HEIGHT,
        fps=FRAME_FPS,
    )

    if not USE_ZED:
        cam.load_standard_calibration(CALIBRATION_FILE)

    estimator = ArucoDistanceEstimator(
        camera_matrix=cam.camera_matrix,
        dist_coeffs=cam.dist_coeffs,
        marker_size_m=MARKER_SIZE_M,
        dictionary_name=DICT_NAME,
    )

    bridge = v2v_bridge.V2VBridge(ESP32_PORT, baud=ESP32_BAUD, name="UAV-Bridge")
    bridge.connect()
    bridge.send_message("ugv face-and-drive cmds-only live")

    if SHOW_WINDOW:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, FRAME_WIDTH, FRAME_HEIGHT)

    phase = "SEARCH"
    status = "Waiting for marker 5 and marker 0"
    cmd_seq = 400

    turn_stable = 0
    done_stable = 0
    last_turn_cmd_time = 0.0
    last_stop_cmd_time = 0.0
    last_move_cmd_time = 0.0
    current_turn_cmd = None

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                blank = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
                shown = draw_overlay(blank, estimator, {}, phase, "No camera frame", None, None)
                if SHOW_WINDOW:
                    cv2.imshow(WINDOW_NAME, shown)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                continue

            poses = estimator.detect_markers(frame)
            ugv_pose = poses.get(UGV_MARKER_ID)
            dest_pose = poses.get(DEST_MARKER_ID)

            dist_m = None
            heading_err_deg = None

            if ugv_pose is not None and dest_pose is not None:
                target_vec, _, _ = ugv_to_dest_vector_on_ground(ugv_pose, dest_pose)
                dist_m = float(np.linalg.norm(target_vec))

                ugv_fwd = marker_forward_vector_on_ground(ugv_pose)
                target_dir = normalize_2d(target_vec)

                if ugv_fwd is not None and target_dir is not None:
                    heading_err_deg = signed_angle_deg(ugv_fwd, target_dir)

            # -------------------------------------------------------------
            # SEARCH
            # -------------------------------------------------------------
            if phase == "SEARCH":
                if ugv_pose is not None and dest_pose is not None and heading_err_deg is not None:
                    phase = "TURN_TO_FACE"
                    status = "Both markers visible. Turning to face destination."
                else:
                    status = "Need both markers visible"

            # -------------------------------------------------------------
            # TURN_TO_FACE
            # -------------------------------------------------------------
            elif phase == "TURN_TO_FACE":
                if ugv_pose is None or dest_pose is None or heading_err_deg is None:
                    turn_stable = 0
                    status = "Lost marker 5 or marker 0 during turn"

                    if current_turn_cmd is not None and (time.time() - last_stop_cmd_time) >= STOP_CMD_RESEND_SEC:
                        bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_STOP, estop=0)
                        cmd_seq += 1
                        last_stop_cmd_time = time.time()
                        current_turn_cmd = None
                else:
                    if abs(heading_err_deg) <= ANGLE_TOL_DEG:
                        turn_stable += 1
                        status = f"Facing destination ({turn_stable}/{TURN_STABLE_FRAMES})"

                        if current_turn_cmd is not None and (time.time() - last_stop_cmd_time) >= STOP_CMD_RESEND_SEC:
                            bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_STOP, estop=0)
                            cmd_seq += 1
                            last_stop_cmd_time = time.time()
                            current_turn_cmd = None

                        if turn_stable >= TURN_STABLE_FRAMES:
                            phase = "DRIVE_FORWARD"
                            turn_stable = 0
                            status = "Turn complete. Driving forward."
                    else:
                        turn_stable = 0
                        wanted_cmd = choose_turn_cmd(heading_err_deg)
                        status = f"Turning. Heading error = {heading_err_deg:.2f} deg"

                        if current_turn_cmd != wanted_cmd or (time.time() - last_turn_cmd_time) >= TURN_CMD_RESEND_SEC:
                            bridge.send_command(cmdSeq=cmd_seq, cmd=wanted_cmd, estop=0)
                            cmd_seq += 1
                            current_turn_cmd = wanted_cmd
                            last_turn_cmd_time = time.time()

            # -------------------------------------------------------------
            # DRIVE_FORWARD
            # -------------------------------------------------------------
            elif phase == "DRIVE_FORWARD":
                if ugv_pose is None or dest_pose is None or dist_m is None:
                    done_stable = 0
                    status = "Lost marker 5 or marker 0 during forward drive"
                else:
                    if heading_err_deg is not None and abs(heading_err_deg) > REALIGN_ANGLE_DEG:
                        phase = "TURN_TO_FACE"
                        status = "Heading drifted. Re-aligning."
                    elif dist_m <= FINAL_DIST_TOL_M:
                        done_stable += 1
                        status = f"At destination ({done_stable}/{DONE_STABLE_FRAMES})"

                        if done_stable >= DONE_STABLE_FRAMES:
                            bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_STOP, estop=0)
                            cmd_seq += 1
                            phase = "DONE"
                            status = "UGV marker is next to destination marker"
                    else:
                        done_stable = 0
                        move_cmd = choose_forward_cmd(dist_m)
                        move_name = "MOVE_FORWARD" if move_cmd == v2v_bridge.CMD_MOVE_FORWARD else "MOVE_2FT"
                        status = f"Driving forward with {move_name}. Remaining distance = {dist_m:.3f} m"

                        if (time.time() - last_move_cmd_time) >= MOVE_CMD_RESEND_SEC:
                            bridge.send_command(cmdSeq=cmd_seq, cmd=move_cmd, estop=0)
                            cmd_seq += 1
                            last_move_cmd_time = time.time()

            elif phase == "DONE":
                status = "Finished. Press q to quit."

            shown = draw_overlay(frame, estimator, poses, phase, status, dist_m, heading_err_deg)

            if SHOW_WINDOW:
                cv2.imshow(WINDOW_NAME, shown)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

    finally:
        try:
            bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_STOP, estop=0)
        except Exception:
            pass

        try:
            bridge.stop()
        except Exception:
            pass

        cam.close()
        print("UAV commands-only script closed.")


if __name__ == "__main__":
    main()
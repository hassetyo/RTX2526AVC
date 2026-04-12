import time
from typing import Dict, Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np

import v2v_bridge
from ArUcoDistance import CameraInterface, ArucoDistanceEstimator, MarkerPose


# =============================================================================
# CONFIG
# =============================================================================

# --- Bridge / radio ---
ESP32_PORT = "/dev/ttyUSB0"
ESP32_BAUD = 115200

# --- Camera ---
USE_ZED = True
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_FPS = 30
CALIBRATION_FILE = "calibration_chessboard.yaml"   # only used for non-ZED

# --- ArUco ---
DICT_NAME = "DICT_6X6_1000"
MARKER_SIZE_M = 0.10
UGV_MARKER_ID = 5
DEST_MARKER_ID = 0

# --- Field-axis mapping from pose tvec ---
# Raw tvec is camera-frame [x, y, z].
# For many UAV views, field-left/right is tvec[0] and field-forward/back is tvec[2].
# If your setup behaves differently, these are the two lines to change.
FIELD_X_TVEC_INDEX = 0
FIELD_Y_TVEC_INDEX = 2

# --- Routing tolerances ---
AXIS_TOL_M = 0.08
STABLE_FRAMES_NEEDED = 5
CMD_RESEND_SEC = 2.0
TURN_SETTLE_SEC = 2.7

# --- Display ---
WINDOW_NAME = "Challenge 2 - UGV Axis Route"
SHOW_AXES = True


# =============================================================================
# HELPERS
# =============================================================================

def get_dictionary_by_name(name: str) -> int:
    if not hasattr(aruco, name):
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return getattr(aruco, name)


def get_marker_pair(
    poses: Dict[int, MarkerPose],
    ugv_id: int,
    dest_id: int,
) -> Tuple[Optional[MarkerPose], Optional[MarkerPose]]:
    return poses.get(ugv_id), poses.get(dest_id)


def compute_field_deltas(dest_pose: MarkerPose, ugv_pose: MarkerPose) -> Tuple[float, float, np.ndarray]:
    """
    Returns:
        dx_field_m: destination x - ugv x
        dy_field_m: destination y - ugv y
        diff: full 3D camera-frame difference vector
    """
    diff = dest_pose.tvec.reshape(3) - ugv_pose.tvec.reshape(3)
    dx_field_m = float(diff[FIELD_X_TVEC_INDEX])
    dy_field_m = float(diff[FIELD_Y_TVEC_INDEX])
    return dx_field_m, dy_field_m, diff


def draw_overlay(
    frame: np.ndarray,
    estimator: ArucoDistanceEstimator,
    poses: Dict[int, MarkerPose],
    phase: str,
    status: str,
    dx_field: Optional[float],
    dy_field: Optional[float],
) -> np.ndarray:
    out = frame.copy()
    out = estimator.draw_markers(out, poses)

    if UGV_MARKER_ID in poses and DEST_MARKER_ID in poses:
        ugv_pose = poses[UGV_MARKER_ID]
        dest_pose = poses[DEST_MARKER_ID]
        cv2.line(out, ugv_pose.center_px, dest_pose.center_px, (0, 255, 255), 2)

    lines = [
        f"Phase: {phase}",
        status,
        f"UGV ID: {UGV_MARKER_ID} | DEST ID: {DEST_MARKER_ID}",
        f"dx_field: {dx_field:.3f} m" if dx_field is not None else "dx_field: ---",
        f"dy_field: {dy_field:.3f} m" if dy_field is not None else "dy_field: ---",
        f"Axis map: X=tvec[{FIELD_X_TVEC_INDEX}]  Y=tvec[{FIELD_Y_TVEC_INDEX}]",
    ]

    padding = 10
    line_h = 28
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.65
    thickness = 2

    max_w = 0
    for line in lines:
        (tw, _), _ = cv2.getTextSize(line, font, scale, thickness)
        max_w = max(max_w, tw)

    box_w = max_w + padding * 2
    box_h = line_h * len(lines) + padding * 2

    overlay = out.copy()
    cv2.rectangle(overlay, (10, 10), (10 + box_w, 10 + box_h), (0, 0, 0), -1)
    out = cv2.addWeighted(overlay, 0.55, out, 0.45, 0)

    y = 10 + padding + 20
    for line in lines:
        cv2.putText(out, line, (20, y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
        y += line_h

    return out


def send_forward_leg(bridge: v2v_bridge.V2VBridge, meters: float) -> None:
    """
    Ground station expects GOTO:x,y with x=forward/back and y=right/left.
    For the first leg we only move along the current forward axis.
    """
    bridge.send_challenge2_coords(meters, 0.0)


def send_stop(cmd_seq: int, bridge: v2v_bridge.V2VBridge) -> int:
    bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_STOP, estop=0)
    return cmd_seq + 1


# =============================================================================
# MAIN
# =============================================================================

def main():
    print("====================================================")
    print("  UAV CHALLENGE 2 AXIS ROUTE")
    print("  Marker 5 -> align to marker 0 y-axis -> turn -> x")
    print("====================================================")
    print("Press q to quit.\n")

    dictionary = get_dictionary_by_name(DICT_NAME)

    # Camera
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
        dictionary_name=dictionary,
    )

    # Bridge
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, baud=ESP32_BAUD, name="UAV-Bridge")
    bridge.connect()
    bridge.send_message("challenge2 axis route live")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, FRAME_WIDTH, FRAME_HEIGHT)

    phase = "SEARCH_BOTH"
    status = "Waiting for both markers"
    stable_count = 0
    aligned_y_count = 0
    aligned_x_count = 0

    last_cmd_time = 0.0
    turn_direction = 0          # +1 = right, -1 = left
    cmd_seq = 100
    cached_dx = None
    cached_dy = None

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                status = "No camera frame"
                blank = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
                shown = draw_overlay(blank, estimator, {}, phase, status, None, None)
                cv2.imshow(WINDOW_NAME, shown)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            poses = estimator.detect_markers(frame)
            ugv_pose, dest_pose = get_marker_pair(poses, UGV_MARKER_ID, DEST_MARKER_ID)

            dx_field = None
            dy_field = None
            if ugv_pose is not None and dest_pose is not None:
                dx_field, dy_field, _ = compute_field_deltas(dest_pose, ugv_pose)
                cached_dx = dx_field
                cached_dy = dy_field

            # -----------------------------------------------------------------
            # PHASE: SEARCH_BOTH
            # -----------------------------------------------------------------
            if phase == "SEARCH_BOTH":
                if ugv_pose is not None and dest_pose is not None:
                    stable_count += 1
                    status = f"Both markers visible ({stable_count}/{STABLE_FRAMES_NEEDED})"
                    if stable_count >= STABLE_FRAMES_NEEDED:
                        if abs(dy_field) > AXIS_TOL_M:
                            send_forward_leg(bridge, dy_field)
                            last_cmd_time = time.time()
                            phase = "WAIT_Y_ALIGN"
                            status = f"Sent Y-axis move: {dy_field:.3f} m"
                        else:
                            phase = "TURN_TO_X"
                            status = "Already aligned on destination y-axis"
                else:
                    stable_count = 0
                    status = "Need both marker 5 and marker 0 visible"

            # -----------------------------------------------------------------
            # PHASE: WAIT_Y_ALIGN
            # -----------------------------------------------------------------
            elif phase == "WAIT_Y_ALIGN":
                if ugv_pose is not None and dest_pose is not None:
                    if abs(dy_field) <= AXIS_TOL_M:
                        aligned_y_count += 1
                        status = f"Y-axis aligned ({aligned_y_count}/{STABLE_FRAMES_NEEDED})"
                        if aligned_y_count >= STABLE_FRAMES_NEEDED:
                            phase = "TURN_TO_X"
                            aligned_y_count = 0
                    else:
                        aligned_y_count = 0
                        status = f"Waiting on Y alignment, error={dy_field:.3f} m"

                        if (time.time() - last_cmd_time) >= CMD_RESEND_SEC:
                            send_forward_leg(bridge, dy_field)
                            last_cmd_time = time.time()
                else:
                    aligned_y_count = 0
                    status = "Lost one of the markers during Y alignment"

            # -----------------------------------------------------------------
            # PHASE: TURN_TO_X
            # -----------------------------------------------------------------
            elif phase == "TURN_TO_X":
                if ugv_pose is not None and dest_pose is not None:
                    if abs(dx_field) <= AXIS_TOL_M:
                        phase = "DONE"
                        cmd_seq = send_stop(cmd_seq, bridge)
                        status = "Already at destination x position too"
                    else:
                        if dx_field > 0:
                            bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_TURN_RIGHT, estop=0)
                            turn_direction = +1
                            status = "Sent TURN_RIGHT (90 deg)"
                        else:
                            bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_TURN_LEFT, estop=0)
                            turn_direction = -1
                            status = "Sent TURN_LEFT (90 deg)"

                        cmd_seq += 1
                        last_cmd_time = time.time()
                        phase = "WAIT_TURN"
                else:
                    status = "Need both markers visible before turn"

            # -----------------------------------------------------------------
            # PHASE: WAIT_TURN
            # -----------------------------------------------------------------
            elif phase == "WAIT_TURN":
                remaining = TURN_SETTLE_SEC - (time.time() - last_cmd_time)
                if remaining <= 0:
                    phase = "SEND_X_MOVE"
                    status = "Turn settled"
                else:
                    status = f"Waiting for turn settle: {remaining:.1f}s"

            # -----------------------------------------------------------------
            # PHASE: SEND_X_MOVE
            # -----------------------------------------------------------------
            elif phase == "SEND_X_MOVE":
                if ugv_pose is not None and dest_pose is not None:
                    # After the 90-degree turn, "forward" should reduce x error.
                    forward_cmd_m = turn_direction * dx_field

                    if abs(dx_field) <= AXIS_TOL_M:
                        phase = "DONE"
                        cmd_seq = send_stop(cmd_seq, bridge)
                        status = "X already aligned after turn"
                    else:
                        send_forward_leg(bridge, forward_cmd_m)
                        last_cmd_time = time.time()
                        phase = "WAIT_X_ALIGN"
                        status = f"Sent X-axis move: {forward_cmd_m:.3f} m"
                elif cached_dx is not None:
                    forward_cmd_m = turn_direction * cached_dx
                    send_forward_leg(bridge, forward_cmd_m)
                    last_cmd_time = time.time()
                    phase = "WAIT_X_ALIGN"
                    status = f"Markers lost, sent cached X move: {forward_cmd_m:.3f} m"
                else:
                    status = "Cannot send X move without marker data"

            # -----------------------------------------------------------------
            # PHASE: WAIT_X_ALIGN
            # -----------------------------------------------------------------
            elif phase == "WAIT_X_ALIGN":
                if ugv_pose is not None and dest_pose is not None:
                    if abs(dx_field) <= AXIS_TOL_M:
                        aligned_x_count += 1
                        status = f"X-axis aligned ({aligned_x_count}/{STABLE_FRAMES_NEEDED})"
                        if aligned_x_count >= STABLE_FRAMES_NEEDED:
                            phase = "DONE"
                            cmd_seq = send_stop(cmd_seq, bridge)
                            status = "Route complete, sent STOP"
                    else:
                        aligned_x_count = 0
                        forward_cmd_m = turn_direction * dx_field
                        status = f"Waiting on X alignment, error={dx_field:.3f} m"

                        if (time.time() - last_cmd_time) >= CMD_RESEND_SEC:
                            send_forward_leg(bridge, forward_cmd_m)
                            last_cmd_time = time.time()
                else:
                    aligned_x_count = 0
                    status = "Lost one of the markers during X alignment"

            # -----------------------------------------------------------------
            # PHASE: DONE
            # -----------------------------------------------------------------
            elif phase == "DONE":
                status = "Finished. Press q to quit."

            shown = draw_overlay(frame, estimator, poses, phase, status, dx_field, dy_field)
            cv2.imshow(WINDOW_NAME, shown)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    finally:
        try:
            bridge.stop()
        except Exception:
            pass
        cam.close()
        print("Axis-route script closed.")


if __name__ == "__main__":
    main()
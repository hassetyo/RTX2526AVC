import time          # for timing and sleeps
import math          # for angle math
import threading     # for running navigation in background while display runs in main thread
import numpy as np   # for vector ops
import cv2           # for aruco detection and display

import v2v_bridge    # our custom radio bridge talker

# import the camera and estimator classes from the shared aruco utility
from ArUcoDistance import CameraInterface, ArucoDistanceEstimator

################################# config
ESP32_PORT = "/dev/ttyUSB0"   
UGV_MARKER_ID  = 5   
DEST_MARKER_ID = 0   
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50  
MARKER_SIZE_M   = 0.254                  
ANGLE_DEADBAND_DEG = 5.0   
ARRIVAL_DIST_M     = 0.20  
UGV_YAW_RATE_DEG_S  = 60.0  
UGV_DRIVE_SPEED_MPS = 1.5   
MANOEUVRE_BUFFER_S  = 0.75  

WINDOW_NAME    = "UGV Auto-Navigator"
HUD_PANEL_W    = 340        
FONT           = cv2.FONT_HERSHEY_SIMPLEX
LOG_MAX_LINES  = 12         

# HUD colour palette (BGR)
COL_BG         = (18,  22,  28)   
COL_ACCENT     = (0,   220, 140)  
COL_DEST       = (30,  160, 255)  
COL_WARN       = (0,   180, 255)  
COL_ERROR      = (50,  50,  220)  
COL_LINE       = (0,   220, 255)  
COL_MUTED      = (100, 110, 120)  
COL_WHITE      = (235, 240, 245)  

PHASE_IDLE         = "IDLE"
PHASE_ALIGNING     = "ALIGNING"
PHASE_APPROACHING = "APPROACHING"
PHASE_ARRIVED     = "ARRIVED"
PHASE_PAUSED      = "PAUSED"
PHASE_ERROR       = "ERROR"

################################# shared state
_lock          = threading.Lock()
_latest_poses  = {}          
_nav_phase     = PHASE_IDLE  
_heading_error = 0.0         
_distance_m    = 0.0         
_last_cmd_str  = "WAITING FOR TARGET"
_log_lines     = []          
_nav_active    = True        # AUTOMATICALLY START
_camera_matrix = None        
_dist_coeffs   = None        

def _log(msg):
    ts   = time.strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    with _lock:
        _log_lines.insert(0, line)
        del _log_lines[LOG_MAX_LINES:]

def _set_phase(phase):
    global _nav_phase
    with _lock:
        _nav_phase = phase
    _log(f"Phase -> {phase}")

################################# camera helpers
def open_camera():
    _log("Opening ZED 2 camera...")
    cam = CameraInterface(use_zed=True, fps=30)
    return cam

def build_estimator(cam):
    return ArucoDistanceEstimator(
        camera_matrix   = cam.camera_matrix,
        dist_coeffs     = cam.dist_coeffs,
        marker_size_m   = MARKER_SIZE_M,
        dictionary_name = ARUCO_DICT_TYPE
    )

################################# navigation maths
def _ugv_forward_2d(ugv_pose):
    R, _  = cv2.Rodrigues(ugv_pose.rvec)
    fwd3  = R @ np.array([0.0, 1.0, 0.0])
    fwd2  = np.array([fwd3[0], fwd3[2]])
    n     = np.linalg.norm(fwd2)
    return fwd2 / n if n > 1e-6 else np.array([0.0, 1.0])

def _dest_vector_2d(ugv_pose, dest_pose):
    diff   = dest_pose.tvec - ugv_pose.tvec
    diff2  = np.array([diff[0], diff[2]])
    dist2  = np.linalg.norm(diff2)
    if dist2 < 1e-6:
        return np.array([0.0, 1.0]), 0.0
    return diff2 / dist2, float(dist2)

def _heading_error_deg(ugv_fwd, dest_dir):
    fx, fz = ugv_fwd
    dx, dz = dest_dir
    return math.degrees(math.atan2(fx * dz - fz * dx, fx * dx + fz * dz))

def _3d_dist(ugv_pose, dest_pose):
    return float(np.linalg.norm(dest_pose.tvec - ugv_pose.tvec))

################################# navigation thread
def _nav_loop(bridge):
    global _heading_error, _distance_m, _last_cmd_str, _nav_active

    cmd_seq = 600

    def get_poses():
        while True:
            with _lock:
                active = _nav_active
                poses  = dict(_latest_poses)
            if not active: return None, None
            ugv   = poses.get(UGV_MARKER_ID)
            dest  = poses.get(DEST_MARKER_ID)
            if ugv and dest: return ugv, dest
            time.sleep(0.05)

    def send_turn(deg, direction_right):
        nonlocal cmd_seq
        cmd_seq  += 1
        direction = "RIGHT" if direction_right else "LEFT"
        label     = f"TURN {direction} {deg:.0f} deg"
        _log(f"CMD -> {label}")
        with _lock: _last_cmd_str = label
        code = v2v_bridge.CMD_TURN_ANGLE_RIGHT if direction_right else v2v_bridge.CMD_TURN_ANGLE_LEFT
        bridge.send_command(cmdSeq=int(round(deg)), cmd=code, estop=0)
        time.sleep((deg / UGV_YAW_RATE_DEG_S) + MANOEUVRE_BUFFER_S)

    def send_drive(dist_m):
        nonlocal cmd_seq
        cmd_seq  += 1
        dist_cm   = max(1, int(round(dist_m * 100.0)))
        label     = f"DRIVE {dist_m:.2f}m"
        _log(f"CMD -> {label}")
        with _lock: _last_cmd_str = label
        bridge.send_command(cmdSeq=dist_cm, cmd=v2v_bridge.CMD_MOVE_DIST, estop=0)
        time.sleep((dist_m / UGV_DRIVE_SPEED_MPS) + MANOEUVRE_BUFFER_S)

    _log("Autonomous Navigation Thread Active. Searching for markers...")

    # PHASE 1 - ALIGN
    _set_phase(PHASE_ALIGNING)
    while True:
        ugv_pose, dest_pose = get_poses()
        if ugv_pose is None: break
        
        fwd   = _ugv_forward_2d(ugv_pose)
        dir2d, _ = _dest_vector_2d(ugv_pose, dest_pose)
        err   = _heading_error_deg(fwd, dir2d)
        
        with _lock: _heading_error = err

        if abs(err) <= ANGLE_DEADBAND_DEG:
            _log("[ALIGN] Heading Locked.")
            break
        send_turn(abs(err), direction_right=(err > 0))

    # PHASE 2 - APPROACH
    _set_phase(PHASE_APPROACHING)
    while True:
        ugv_pose, dest_pose = get_poses()
        if ugv_pose is None: break
        
        dist = _3d_dist(ugv_pose, dest_pose)
        with _lock: _distance_m = dist

        if dist <= ARRIVAL_DIST_M: break

        send_drive(dist)

        # Mid-course correction
        ugv_pose, dest_pose = get_poses()
        if ugv_pose is None: break
        fwd = _ugv_forward_2d(ugv_pose)
        dir2d, _ = _dest_vector_2d(ugv_pose, dest_pose)
        err = _heading_error_deg(fwd, dir2d)
        if abs(err) > ANGLE_DEADBAND_DEG:
            send_turn(abs(err), direction_right=(err > 0))

    # DONE
    bridge.send_command(cmdSeq=cmd_seq+1, cmd=v2v_bridge.CMD_STOP, estop=0)
    _set_phase(PHASE_ARRIVED)
    _log("Target reached. System shutting down.")

################################# drawing helpers (Standard UI)
def draw_ugv_forward_arrow(frame, ugv_pose, camera_matrix, dist_coeffs):
    tip_obj = np.array([[0.0, 0.15, 0.0]], dtype=np.float32)
    tip_px, _ = cv2.projectPoints(tip_obj, ugv_pose.rvec.reshape(3, 1), ugv_pose.tvec.reshape(3, 1), camera_matrix, dist_coeffs)
    tip = tuple(tip_px[0][0].astype(int))
    cv2.arrowedLine(frame, ugv_pose.center_px, tip, COL_ACCENT, 3, tipLength=0.35)

def draw_connecting_line(frame, ugv_pose, dest_pose, dist_m):
    cv2.line(frame, ugv_pose.center_px, dest_pose.center_px, COL_LINE, 2)
    mid = ((ugv_pose.center_px[0] + dest_pose.center_px[0]) // 2, (ugv_pose.center_px[1] + dest_pose.center_px[1]) // 2)
    cv2.putText(frame, f"{dist_m:.2f}m", mid, FONT, 0.65, COL_LINE, 2)

def draw_heading_arc(frame, ugv_pose, heading_err_deg):
    cx, cy = ugv_pose.center_px
    colour = COL_WARN if abs(heading_err_deg) > ANGLE_DEADBAND_DEG else COL_ACCENT
    cv2.ellipse(frame, (cx, cy), (45, 45), 0, -90, -90 + heading_err_deg, colour, 2)

def draw_hud_panel(canvas, panel_x, phase, heading_err, dist, last_cmd, log_lines, radio_ok):
    h = canvas.shape[0]
    cv2.rectangle(canvas, (panel_x, 0), (canvas.shape[1], h), COL_BG, -1)
    cv2.putText(canvas, "AUTO-NAV ACTIVE", (panel_x + 14, 36), FONT, 0.72, COL_ACCENT, 2)
    cv2.putText(canvas, f"PHASE: {phase}", (panel_x + 14, 80), FONT, 0.7, COL_WHITE, 2)
    cv2.putText(canvas, f"ERR: {heading_err:+.1f} deg", (panel_x + 14, 120), FONT, 0.6, COL_WARN, 2)
    cv2.putText(canvas, f"DIST: {dist:.3f} m", (panel_x + 14, 160), FONT, 0.6, COL_LINE, 2)
    cv2.putText(canvas, f"CMD: {last_cmd}", (panel_x + 14, 210), FONT, 0.5, COL_WHITE, 1)

################################# main loop
def main():
    global _nav_active, _latest_poses, _heading_error, _distance_m, _camera_matrix, _dist_coeffs

    cam = open_camera()
    estimator = build_estimator(cam)
    _camera_matrix, _dist_coeffs = cam.camera_matrix, cam.dist_coeffs

    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="Auto-UGV")
    radio_ok = False
    try:
        bridge.connect()
        radio_ok = True
    except: _log("Radio Offline.")

    threading.Thread(target=_nav_loop, args=(bridge,), daemon=True).start()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    while True:
        frame = cam.get_frame()
        if frame is None: continue

        poses = estimator.detect_markers(frame)
        with _lock: _latest_poses = dict(poses)

        h, w = frame.shape[:2]
        canvas = np.zeros((h, w + HUD_PANEL_W, 3), dtype=np.uint8)
        canvas[:, :w] = frame

        ugv_pose = poses.get(UGV_MARKER_ID)
        dest_pose = poses.get(DEST_MARKER_ID)

        if ugv_pose: draw_ugv_forward_arrow(canvas[:, :w], ugv_pose, _camera_matrix, _dist_coeffs)
        if ugv_pose and dest_pose:
            draw_connecting_line(canvas[:, :w], ugv_pose, dest_pose, _distance_m)
            draw_heading_arc(canvas[:, :w], ugv_pose, _heading_error)

        with _lock:
            draw_hud_panel(canvas, w, _nav_phase, _heading_error, _distance_m, _last_cmd_str, _log_lines, radio_ok)
            if _nav_phase == PHASE_ARRIVED:
                time.sleep(2) # Show "Arrived" for 2 seconds before closing
                break

        cv2.imshow(WINDOW_NAME, canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cam.close()
    if radio_ok: bridge.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
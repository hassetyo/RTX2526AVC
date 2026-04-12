import time          # for timing and sleeps
import math          # for angle math
import threading     # for running navigation in background while display runs in main thread
import numpy as np   # for vector ops
import cv2           # for aruco detection and display

import v2v_bridge    # our custom radio bridge talker

# import the camera and estimator classes from the shared aruco utility
from ArUcoDistance import CameraInterface, ArucoDistanceEstimator

# -----------------------------------------------------------------------
# UGV VISUAL NAVIGATOR  -  Camera GUI Edition
# -----------------------------------------------------------------------
# The script opens the ZED 2 camera and shows a live HUD displaying:
#   - Real-time ArUco marker detection with 6-DOF axes drawn
#   - A forward-direction arrow projected from the UGV marker (ID 5)
#   - A line from the UGV to the destination marker (ID 0) with distance
#   - A right-side status panel showing navigation phase, heading error,
#     distance, last command, and a scrolling log
#
# Keyboard controls:
#   S     - Start / resume navigation
#   SPACE - Pause navigation (stops sending commands; camera keeps running)
#   E     - Emergency stop (sends CMD_STOP to UGV via radio)
#   Q / ESC - Quit
#
# Navigation logic (closed-loop visual servo):
#   PHASE 1 - ALIGN:   Turn UGV right/left until heading error < deadband
#   PHASE 2 - APPROACH: Drive forward, re-align between segments, until close
#
# Coordinate system (nadir-mounted ZED 2):
#   Camera +X = drone right  |  Camera +Y = down  |  Camera +Z = into ground
#   Ground plane = XZ plane.  UGV forward = R @ [0,1,0] projected to (x,z).
# -----------------------------------------------------------------------

################################# config

# hardware ports
ESP32_PORT = "/dev/ttyUSB0"   # radio bridge USB wire to ESP32

# aruco marker ids
UGV_MARKER_ID  = 5   # tag mounted flat on UGV, local +Y = UGV forward
DEST_MARKER_ID = 0   # stationary target marker on the ground

# aruco / camera settings
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50  # marker dictionary (match your printed markers)
MARKER_SIZE_M   = 0.254                  # physical side length of each printed marker, metres

# navigation thresholds
ANGLE_DEADBAND_DEG = 5.0   # stop turning when heading error is within this many degrees
ARRIVAL_DIST_M     = 0.20  # consider UGV arrived when markers are this close (metres)

# ugv motion speeds (must match ground_station.py constants)
UGV_YAW_RATE_DEG_S  = 60.0  # degrees per second from execute_turn in ground_station
UGV_DRIVE_SPEED_MPS = 1.5   # metres per second from SPEED_MPS in ground_station
MANOEUVRE_BUFFER_S  = 0.75  # extra wait added on top of calculated manoeuvre time

# display settings
WINDOW_NAME    = "UGV Visual Navigator"
HUD_PANEL_W    = 340        # width of the right-side status panel in pixels
FONT           = cv2.FONT_HERSHEY_SIMPLEX
LOG_MAX_LINES  = 12         # max scrolling log lines shown in panel

# HUD colour palette (BGR)
COL_BG         = (18,  22,  28)   # almost-black panel background
COL_ACCENT     = (0,   220, 140)  # bright teal - primary text / UGV elements
COL_DEST       = (30,  160, 255)  # amber-orange - destination elements
COL_WARN       = (0,   180, 255)  # orange - warnings / active commands
COL_ERROR      = (50,  50,  220)  # red - errors
COL_LINE       = (0,   220, 255)  # yellow - connecting line UGV to dest
COL_MUTED      = (100, 110, 120)  # grey - muted / inactive text
COL_WHITE      = (235, 240, 245)  # near-white for labels

# navigation phases (string tags used throughout)
PHASE_IDLE        = "IDLE"
PHASE_ALIGNING    = "ALIGNING"
PHASE_APPROACHING = "APPROACHING"
PHASE_ARRIVED     = "ARRIVED"
PHASE_PAUSED      = "PAUSED"
PHASE_ERROR       = "ERROR"

################################# shared navigation state
# the nav thread writes here; the display thread reads here
# everything under _lock is safe to access from either thread

_lock          = threading.Lock()
_latest_poses  = {}          # {marker_id: MarkerPose} from last detection
_nav_phase     = PHASE_IDLE  # current navigation phase string
_heading_error = 0.0         # degrees, positive = UGV must turn right
_distance_m    = 0.0         # metres between the two markers
_last_cmd_str  = "\u2014"    # human-readable description of last command sent
_log_lines     = []          # scrolling log (newest at index 0)
_nav_active    = False       # True while the nav thread should keep running
_camera_matrix = None        # set after camera opens so draw helpers can use it
_dist_coeffs   = None        # same


def _log(msg):  # thread-safe log: prepends timestamp, appends to scrolling list
    ts   = time.strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    with _lock:
        _log_lines.insert(0, line)       # newest at top
        del _log_lines[LOG_MAX_LINES:]   # trim old entries


def _set_phase(phase):  # thread-safe phase setter
    global _nav_phase
    with _lock:
        _nav_phase = phase
    _log(f"Phase -> {phase}")


################################# camera helpers

def open_camera():  # opens the ZED 2 using the shared CameraInterface
    _log("Opening ZED 2 camera...")
    cam = CameraInterface(use_zed=True, fps=30)
    _log("ZED 2 open OK.")
    return cam


def build_estimator(cam):  # builds ArucoDistanceEstimator from ZED intrinsics
    return ArucoDistanceEstimator(
        camera_matrix   = cam.camera_matrix,
        dist_coeffs     = cam.dist_coeffs,
        marker_size_m   = MARKER_SIZE_M,
        dictionary_name = ARUCO_DICT_TYPE
    )


################################# navigation maths

def _ugv_forward_2d(ugv_pose):
    """
    Returns the UGV's current forward direction as a normalised 2D vector
    [x, z] in camera space (ground plane = XZ for nadir camera).
    The UGV marker's local +Y axis encodes the physical forward direction.
    """
    R, _  = cv2.Rodrigues(ugv_pose.rvec)           # rvec -> rotation matrix
    fwd3  = R @ np.array([0.0, 1.0, 0.0])          # marker +Y in camera frame
    fwd2  = np.array([fwd3[0], fwd3[2]])            # project to XZ ground plane
    n     = np.linalg.norm(fwd2)
    return fwd2 / n if n > 1e-6 else np.array([0.0, 1.0])


def _dest_vector_2d(ugv_pose, dest_pose):
    """
    Returns (unit_direction [x,z], horizontal_distance_m) from UGV to
    destination, projected onto the XZ ground plane.
    """
    diff   = dest_pose.tvec - ugv_pose.tvec        # 3D vector UGV -> dest
    diff2  = np.array([diff[0], diff[2]])           # ground plane projection
    dist2  = np.linalg.norm(diff2)
    if dist2 < 1e-6:
        return np.array([0.0, 1.0]), 0.0
    return diff2 / dist2, float(dist2)


def _heading_error_deg(ugv_fwd, dest_dir):
    """
    Signed angle from ugv_fwd to dest_dir (degrees).
    Positive = turn right (CW viewed from above).
    Negative = turn left  (CCW viewed from above).
    """
    fx, fz = ugv_fwd
    dx, dz = dest_dir
    return math.degrees(math.atan2(fx * dz - fz * dx, fx * dx + fz * dz))


def _3d_dist(ugv_pose, dest_pose):  # euclidean distance between tvec positions (metres)
    return float(np.linalg.norm(dest_pose.tvec - ugv_pose.tvec))


################################# navigation thread

def _nav_loop(bridge):
    """
    Background navigation thread.
    Reads _latest_poses, computes errors, sends v2v commands.
    Waits for _nav_active to become True before starting.
    """
    global _heading_error, _distance_m, _last_cmd_str, _nav_active

    cmd_seq = 500  # start high so it does not collide with mission-4 sequence numbers

    def get_poses():
        # blocks until both markers are visible or nav is paused/stopped
        while True:
            with _lock:
                active = _nav_active
                poses  = dict(_latest_poses)
            if not active:
                return None, None
            ugv  = poses.get(UGV_MARKER_ID)
            dest = poses.get(DEST_MARKER_ID)
            if ugv and dest:
                return ugv, dest
            time.sleep(0.05)   # wait for detection before retrying

    def send_turn(deg, direction_right):  # helper: send a turn command and update status display
        nonlocal cmd_seq
        cmd_seq  += 1
        direction = "RIGHT" if direction_right else "LEFT"
        label     = f"TURN {direction} {deg:.0f} deg"
        _log(f"CMD -> {label}  (seq={cmd_seq})")
        with _lock:
            _last_cmd_str = label
        code = v2v_bridge.CMD_TURN_ANGLE_RIGHT if direction_right else v2v_bridge.CMD_TURN_ANGLE_LEFT
        bridge.send_command(cmdSeq=int(round(deg)), cmd=code, estop=0)
        wait = (deg / UGV_YAW_RATE_DEG_S) + MANOEUVRE_BUFFER_S  # calculated wait + buffer
        _log(f"Waiting {wait:.1f}s for turn...")
        time.sleep(wait)

    def send_drive(dist_m):  # helper: send a drive command and update status display
        nonlocal cmd_seq
        cmd_seq  += 1
        dist_cm   = max(1, int(round(dist_m * 100.0)))
        label     = f"DRIVE {dist_m:.2f}m ({dist_cm}cm)"
        _log(f"CMD -> {label}  (seq={cmd_seq})")
        with _lock:
            _last_cmd_str = label
        bridge.send_command(cmdSeq=dist_cm, cmd=v2v_bridge.CMD_MOVE_DIST, estop=0)
        wait = (dist_m / UGV_DRIVE_SPEED_MPS) + MANOEUVRE_BUFFER_S  # calculated wait + buffer
        _log(f"Waiting {wait:.1f}s for drive...")
        time.sleep(wait)

    # ------------------------------------------------------------------ #
    # wait until the user presses S to start                             #
    # ------------------------------------------------------------------ #
    _log("Nav thread ready. Press S to start navigation.")
    while True:
        with _lock:
            active = _nav_active
        if active:
            break
        time.sleep(0.1)

    # ------------------------------------------------------------------ #
    # PHASE 1 - ALIGN                                                     #
    # ------------------------------------------------------------------ #
    _set_phase(PHASE_ALIGNING)
    _log("=== PHASE 1: ALIGN ===")

    while True:
        # respect pause: if nav was paused mid-alignment, hold here until resumed
        with _lock:
            if not _nav_active:
                _set_phase(PHASE_PAUSED)
                while not _nav_active:
                    time.sleep(0.1)
                _set_phase(PHASE_ALIGNING)

        ugv_pose, dest_pose = get_poses()
        if ugv_pose is None:
            break  # navigation was stopped externally

        fwd      = _ugv_forward_2d(ugv_pose)
        dir2d, _ = _dest_vector_2d(ugv_pose, dest_pose)
        err      = _heading_error_deg(fwd, dir2d)

        with _lock:
            _heading_error = err

        _log(f"[ALIGN] error={err:+.1f} deg")

        if abs(err) <= ANGLE_DEADBAND_DEG:   # within deadband - alignment complete
            _log(f"[ALIGN] Locked. Error {err:+.1f} deg is within {ANGLE_DEADBAND_DEG} deg deadband.")
            break

        send_turn(abs(err), direction_right=(err > 0))

    # ------------------------------------------------------------------ #
    # PHASE 2 - APPROACH                                                  #
    # ------------------------------------------------------------------ #
    _set_phase(PHASE_APPROACHING)
    _log("=== PHASE 2: APPROACH ===")

    while True:
        # respect pause: hold here if navigation was paused mid-approach
        with _lock:
            if not _nav_active:
                _set_phase(PHASE_PAUSED)
                while not _nav_active:
                    time.sleep(0.1)
                _set_phase(PHASE_APPROACHING)

        ugv_pose, dest_pose = get_poses()
        if ugv_pose is None:
            break  # navigation was stopped externally

        dist = _3d_dist(ugv_pose, dest_pose)

        with _lock:
            _distance_m = dist

        _log(f"[APPROACH] dist={dist:.3f}m")

        if dist <= ARRIVAL_DIST_M:           # close enough - arrived
            break

        send_drive(dist)                     # drive the remaining distance

        # re-check alignment after each drive segment to correct heading drift
        _log("[APPROACH] Re-checking alignment...")
        ugv_pose, dest_pose = get_poses()
        if ugv_pose is None:
            break

        fwd      = _ugv_forward_2d(ugv_pose)
        dir2d, _ = _dest_vector_2d(ugv_pose, dest_pose)
        err      = _heading_error_deg(fwd, dir2d)

        with _lock:
            _heading_error = err

        if abs(err) > ANGLE_DEADBAND_DEG:    # significant drift - correct before next drive
            _log(f"[APPROACH] Heading drifted to {err:+.1f} deg. Correcting...")
            send_turn(abs(err), direction_right=(err > 0))

    # ------------------------------------------------------------------ #
    # DONE                                                                #
    # ------------------------------------------------------------------ #
    cmd_seq += 1
    bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_STOP, estop=0)  # halt UGV cleanly
    with _lock:
        _last_cmd_str = "STOP"
    _set_phase(PHASE_ARRIVED)
    _log("=== NAVIGATION COMPLETE ===")


################################# drawing helpers

def draw_ugv_forward_arrow(frame, ugv_pose, camera_matrix, dist_coeffs):
    """
    Projects a point 15 cm along the UGV marker's local +Y axis and draws
    a bold green arrow from the marker centre to that projected point.
    This arrow shows which direction the UGV is physically facing.
    """
    tip_obj = np.array([[0.0, 0.15, 0.0]], dtype=np.float32)   # 15 cm along marker +Y
    tip_px, _ = cv2.projectPoints(
        tip_obj,
        ugv_pose.rvec.reshape(3, 1),
        ugv_pose.tvec.reshape(3, 1),
        camera_matrix,
        dist_coeffs
    )
    tip    = tuple(tip_px[0][0].astype(int))
    center = ugv_pose.center_px

    cv2.arrowedLine(frame, center, tip, (10, 30, 10),  6, tipLength=0.35)  # dark shadow for contrast
    cv2.arrowedLine(frame, center, tip, COL_ACCENT,    3, tipLength=0.35)  # bright teal arrow on top


def draw_connecting_line(frame, ugv_pose, dest_pose, dist_m):
    """
    Draws a dashed line from the UGV marker centre to the destination
    marker centre, with the distance label floating at the midpoint.
    """
    p1 = ugv_pose.center_px
    p2 = dest_pose.center_px

    # build dashed line by drawing alternating segments along the direction vector
    dx         = p2[0] - p1[0]
    dy         = p2[1] - p1[1]
    seg_len    = 12   # pixels per dash
    gap_len    = 8    # pixels per gap
    total_len  = math.hypot(dx, dy)
    if total_len < 1:
        return

    ux   = dx / total_len   # unit direction vector
    uy   = dy / total_len
    pos  = 0.0
    draw = True

    while pos < total_len:
        seg_end = min(pos + (seg_len if draw else gap_len), total_len)
        if draw:
            a = (int(p1[0] + ux * pos),     int(p1[1] + uy * pos))
            b = (int(p1[0] + ux * seg_end), int(p1[1] + uy * seg_end))
            cv2.line(frame, a, b, COL_LINE, 2)
        pos  += (seg_len if draw else gap_len)
        draw  = not draw

    # distance label floating at the midpoint of the line
    mid  = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
    text = f"{dist_m:.2f} m"
    (tw, th), _ = cv2.getTextSize(text, FONT, 0.65, 2)
    cv2.rectangle(frame,
                  (mid[0] - tw//2 - 6, mid[1] - th - 6),
                  (mid[0] + tw//2 + 6, mid[1] + 6),
                  (20, 20, 20), -1)          # dark pill background
    cv2.putText(frame, text,
                (mid[0] - tw//2, mid[1] - 2),
                FONT, 0.65, COL_LINE, 2, cv2.LINE_AA)


def draw_heading_arc(frame, ugv_pose, heading_err_deg):
    """
    Draws a small arc around the UGV marker to visualise the heading error.
    The arc sweeps from the UGV's current forward direction by the error angle.
    Teal = within deadband (aligned); orange = correction needed.
    """
    cx, cy = ugv_pose.center_px
    radius = 45

    if abs(heading_err_deg) < 0.5:   # essentially zero - skip the arc
        return

    # sweep from 270 deg (top of circle = forward reference) by the error amount
    start_angle = -90                        # top of the circle in OpenCV convention
    end_angle   = start_angle + heading_err_deg

    colour = COL_WARN if abs(heading_err_deg) > ANGLE_DEADBAND_DEG else COL_ACCENT
    cv2.ellipse(frame, (cx, cy), (radius, radius),
                0, start_angle, end_angle, colour, 2)

    # small dot at the arc tip to make it look like a needle
    end_rad = math.radians(end_angle)
    tip_x   = int(cx + radius * math.cos(end_rad))
    tip_y   = int(cy + radius * math.sin(end_rad))
    cv2.circle(frame, (tip_x, tip_y), 4, colour, -1)

    # angle label below the arc
    label = f"{heading_err_deg:+.1f} deg"
    cv2.putText(frame, label,
                (cx - 32, cy + radius + 20),
                FONT, 0.52, colour, 1, cv2.LINE_AA)


def draw_marker_label(frame, pose, marker_id):
    """
    Draws a compact ID label tag next to the marker centre.
    UGV marker (ID 5) is tagged in teal; destination (ID 0) in amber-orange.
    """
    colour = COL_ACCENT if marker_id == UGV_MARKER_ID else COL_DEST
    tag    = "UGV" if marker_id == UGV_MARKER_ID else "DEST"
    text   = f"[{tag}] ID {marker_id}"
    cx, cy = pose.center_px

    (tw, th), _ = cv2.getTextSize(text, FONT, 0.6, 2)
    bx = cx + 14
    by = cy - th - 10
    cv2.rectangle(frame, (bx - 4, by - 4), (bx + tw + 4, by + th + 6),
                  (15, 15, 15), -1)          # dark fill
    cv2.rectangle(frame, (bx - 4, by - 4), (bx + tw + 4, by + th + 6),
                  colour, 1)                 # coloured border
    cv2.putText(frame, text, (bx, by + th), FONT, 0.6, colour, 2, cv2.LINE_AA)


def draw_hud_panel(canvas, panel_x, phase, heading_err, dist, last_cmd, log_lines, radio_ok):
    """
    Draws the full right-side status HUD panel onto canvas starting at panel_x.
    """
    h = canvas.shape[0]

    # solid dark background for the whole panel
    cv2.rectangle(canvas, (panel_x, 0), (canvas.shape[1], h), COL_BG, -1)

    # thin accent separator line between camera feed and panel
    cv2.line(canvas, (panel_x, 0), (panel_x, h), COL_ACCENT, 2)

    # ---- title bar ----
    cv2.putText(canvas, "UGV NAVIGATOR",
                (panel_x + 14, 36), FONT, 0.72, COL_ACCENT, 2, cv2.LINE_AA)
    cv2.line(canvas, (panel_x + 10, 48),
             (canvas.shape[1] - 10, 48), COL_MUTED, 1)

    # ---- phase badge (coloured filled pill) ----
    phase_colours = {
        PHASE_IDLE:        COL_MUTED,
        PHASE_ALIGNING:    COL_WARN,
        PHASE_APPROACHING: (0, 200, 255),
        PHASE_ARRIVED:     COL_ACCENT,
        PHASE_PAUSED:      COL_WARN,
        PHASE_ERROR:       COL_ERROR,
    }
    p_col = phase_colours.get(phase, COL_WHITE)
    (pw, ph), _ = cv2.getTextSize(phase, FONT, 0.72, 2)
    bx1, by1 = panel_x + 14, 60
    bx2, by2 = bx1 + pw + 20, by1 + ph + 14
    cv2.rectangle(canvas, (bx1, by1), (bx2, by2), p_col, -1)
    cv2.putText(canvas, phase, (bx1 + 10, by2 - 7),
                FONT, 0.72, COL_BG, 2, cv2.LINE_AA)

    y = by2 + 26   # running y cursor for the rest of the panel

    # ---- reusable section helper ----
    def section(label, value, value_col, yy):
        cv2.putText(canvas, label, (panel_x + 14, yy),
                    FONT, 0.45, COL_MUTED, 1, cv2.LINE_AA)
        cv2.putText(canvas, value, (panel_x + 14, yy + 24),
                    FONT, 0.75, value_col, 2, cv2.LINE_AA)
        return yy + 52

    # ---- heading error section ----
    err_col = COL_WARN if abs(heading_err) > ANGLE_DEADBAND_DEG else COL_ACCENT
    y = section("HEADING ERROR", f"{heading_err:+.1f} deg", err_col, y)

    # ---- distance section ----
    dist_col = COL_WARN if dist > ARRIVAL_DIST_M else COL_ACCENT
    y = section("DISTANCE TO DEST", f"{dist:.3f} m", dist_col, y)

    cv2.line(canvas, (panel_x + 10, y - 6),
             (canvas.shape[1] - 10, y - 6), COL_MUTED, 1)

    # ---- last command section ----
    y = section("LAST COMMAND", last_cmd, COL_WHITE, y)

    # ---- radio / connection status ----
    r_col   = COL_ACCENT if radio_ok else COL_ERROR
    r_label = "RADIO: CONNECTED" if radio_ok else "RADIO: OFFLINE"
    cv2.putText(canvas, r_label, (panel_x + 14, y),
                FONT, 0.5, r_col, 1, cv2.LINE_AA)
    y += 28

    # ---- marker visibility status ----
    with _lock:
        poses_now = dict(_latest_poses)

    ugv_vis  = UGV_MARKER_ID  in poses_now
    dest_vis = DEST_MARKER_ID in poses_now

    cv2.putText(canvas,
                f"UGV  (ID {UGV_MARKER_ID}) : {'VISIBLE' if ugv_vis  else 'NOT FOUND'}",
                (panel_x + 14, y), FONT, 0.46,
                COL_ACCENT if ugv_vis else COL_ERROR, 1, cv2.LINE_AA)
    y += 22
    cv2.putText(canvas,
                f"DEST (ID {DEST_MARKER_ID}) : {'VISIBLE' if dest_vis else 'NOT FOUND'}",
                (panel_x + 14, y), FONT, 0.46,
                COL_DEST if dest_vis else COL_ERROR, 1, cv2.LINE_AA)
    y += 32

    cv2.line(canvas, (panel_x + 10, y - 6),
             (canvas.shape[1] - 10, y - 6), COL_MUTED, 1)

    # ---- scrolling log ----
    cv2.putText(canvas, "LOG", (panel_x + 14, y + 4),
                FONT, 0.45, COL_MUTED, 1, cv2.LINE_AA)
    y += 22

    with _lock:
        lines = list(log_lines)

    max_chars = (HUD_PANEL_W - 28) // 7   # rough character budget per line at scale 0.37
    for line in lines:
        if y > h - 28:
            break
        display = line if len(line) <= max_chars else "..." + line[-(max_chars - 3):]
        cv2.putText(canvas, display, (panel_x + 14, y),
                    FONT, 0.37, COL_MUTED, 1, cv2.LINE_AA)
        y += 17

    # ---- keymap footer ----
    cv2.putText(canvas,
                "[S] Start  [SPACE] Pause  [E] E-Stop  [Q] Quit",
                (panel_x + 8, h - 12),
                FONT, 0.36, COL_MUTED, 1, cv2.LINE_AA)


################################# main

def main():  # the main boss function
    global _nav_active, _latest_poses, _heading_error, _distance_m
    global _camera_matrix, _dist_coeffs

    _log("==========================================")
    _log("   UGV VISUAL NAVIGATOR - GUI EDITION    ")
    _log(f"   UGV marker  : ID {UGV_MARKER_ID}    ")
    _log(f"   Dest marker : ID {DEST_MARKER_ID}   ")
    _log("==========================================")

    # ------------------------------------------------------------------ #
    # open camera and build estimator                                     #
    # ------------------------------------------------------------------ #
    cam       = open_camera()
    estimator = build_estimator(cam)

    # store intrinsics globally so draw helpers and nav thread can use them
    _camera_matrix = cam.camera_matrix
    _dist_coeffs   = cam.dist_coeffs

    # ------------------------------------------------------------------ #
    # connect radio bridge (non-fatal - GUI still runs without radio)     #
    # ------------------------------------------------------------------ #
    radio_ok = False
    bridge   = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Navigator")
    try:
        bridge.connect()
        bridge.send_message("UGV NAVIGATOR: GUI MODE ACTIVE")
        radio_ok = True
        _log("Radio bridge connected.")
    except Exception as e:
        _log(f"Radio bridge OFFLINE ({e}). Commands will not reach UGV.")

    # ------------------------------------------------------------------ #
    # start the navigation background thread (paused until S pressed)    #
    # ------------------------------------------------------------------ #
    nav_thread = threading.Thread(target=_nav_loop, args=(bridge,), daemon=True)
    nav_thread.start()

    # ------------------------------------------------------------------ #
    # OpenCV display window setup                                         #
    # ------------------------------------------------------------------ #
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280 + HUD_PANEL_W, 720)
    _log("Display ready. Press S to start navigation.")

    # ------------------------------------------------------------------ #
    # main display loop                                                   #
    # ------------------------------------------------------------------ #
    while True:
        # ---- grab frame from ZED 2 ----
        frame = cam.get_frame()
        if frame is None:
            _log("Frame read failed - retrying...")
            time.sleep(0.05)
            continue

        # ---- detect aruco markers ----
        poses = estimator.detect_markers(frame)

        # update shared pose state for the navigation thread to read
        with _lock:
            _latest_poses = dict(poses)

        # ---- build the display canvas: camera feed + HUD panel side by side ----
        h, w    = frame.shape[:2]
        panel_x = w    # the HUD panel starts immediately after the camera frame
        canvas  = np.zeros((h, w + HUD_PANEL_W, 3), dtype=np.uint8)
        canvas[:, :w] = frame   # paste the raw camera frame on the left side

        # ---- draw ArUco axes and corner squares from ArucoDistanceEstimator ----
        estimator.draw_markers(canvas[:, :w], poses)   # operates on camera region only

        # ---- draw custom navigation overlays ----
        ugv_pose  = poses.get(UGV_MARKER_ID)
        dest_pose = poses.get(DEST_MARKER_ID)

        if ugv_pose:
            # green forward arrow showing which way the UGV is pointing
            draw_ugv_forward_arrow(
                canvas[:, :w], ugv_pose, _camera_matrix, _dist_coeffs
            )
            draw_marker_label(canvas[:, :w], ugv_pose,  UGV_MARKER_ID)

        if dest_pose:
            draw_marker_label(canvas[:, :w], dest_pose, DEST_MARKER_ID)

        if ugv_pose and dest_pose:
            dist = _3d_dist(ugv_pose, dest_pose)
            with _lock:
                _distance_m = dist

            # dashed line with distance label between the two markers
            draw_connecting_line(canvas[:, :w], ugv_pose, dest_pose, dist)

            # heading error arc around the UGV marker
            fwd   = _ugv_forward_2d(ugv_pose)
            d2d, _ = _dest_vector_2d(ugv_pose, dest_pose)
            err   = _heading_error_deg(fwd, d2d)
            with _lock:
                _heading_error = err
            draw_heading_arc(canvas[:, :w], ugv_pose, err)

        # ---- draw the HUD panel on the right side ----
        with _lock:
            phase  = _nav_phase
            h_err  = _heading_error
            dist_v = _distance_m
            last_c = _last_cmd_str
            log_l  = list(_log_lines)

        draw_hud_panel(canvas, panel_x, phase, h_err, dist_v, last_c, log_l, radio_ok)

        # ---- render frame to screen ----
        cv2.imshow(WINDOW_NAME, canvas)

        # ---- keyboard input ----
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), 27):        # Q or ESC - quit the application
            _log("Quit requested.")
            with _lock:
                _nav_active = False
            break

        elif key == ord('s'):            # S - start or resume navigation
            with _lock:
                already_done = _nav_phase in (PHASE_ARRIVED, PHASE_ERROR)
            if already_done:
                _log("Navigation already finished. Restart the script to run again.")
            else:
                _log("Navigation STARTED (or RESUMED).")
                with _lock:
                    _nav_active = True

        elif key == ord(' '):            # SPACE - pause navigation
            with _lock:
                _nav_active = False
            _log("Navigation PAUSED. Press S to resume.")

        elif key == ord('e'):            # E - emergency stop
            with _lock:
                _nav_active = False
            _log("[!] EMERGENCY STOP. Sending CMD_STOP to UGV.")
            if radio_ok:
                bridge.send_command(cmdSeq=999, cmd=v2v_bridge.CMD_STOP, estop=1)
            _set_phase(PHASE_IDLE)

    # ------------------------------------------------------------------ #
    # cleanup                                                             #
    # ------------------------------------------------------------------ #
    cam.close()       # close ZED 2 and release camera resources
    if radio_ok:
        bridge.stop() # close radio serial wire cleanly
    cv2.destroyAllWindows()
    _log("Navigator shut down.")


if __name__ == "__main__":  # entry point
    main()  # run it
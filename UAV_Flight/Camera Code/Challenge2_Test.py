"""
challenge2_mission.py
=====================
Full Challenge 2 UAV mission — merged from VLandArUco, moveToCenter2, and ArUcoMovement.

Mission sequence
----------------
  1. Connect to flight controller (pymavlink, raw MAVLink).
  2. Set GUIDED mode, arm motors, issue MAV_CMD_NAV_TAKEOFF.
  3. Wait for target altitude.
  4. Hover briefly (HOVER_WAIT_S) while sending zero-velocity hold commands.
  5. moveToCenter — fly toward field centre while climbing until BOTH ArUco
     markers are visible; uses body-frame velocity commands throughout.
  6. guideUGV — overhead ArUco nav loop that commands the ground vehicle
     via the ESP32 V2V bridge until the UGV reaches the destination marker.
  7. Land safely.

Camera
------
  Always uses the ZED X camera via pyzed.sl (no fallback to USB cam).
  Intrinsics are read directly from the SDK — no YAML file required.

Flight controller
-----------------
  All UAV movement is issued as SET_POSITION_TARGET_LOCAL_NED velocity
  commands (MAV_FRAME_BODY_NED) so the vehicle stays in GUIDED mode
  throughout. No dronekit dependency.

Usage
-----
  python3 challenge2_mission.py [options]

  --bridge-port   /dev/ttyUSB0   ESP32 V2V bridge serial port
  --ugv-marker-id 5              ArUco ID on the UGV
  --dest-marker-id 0             Destination ArUco ID
  --marker-size   0.254          Marker side length in metres
  --target-alt    0.30           Takeoff altitude in metres
  --testing                      Dry-run: skip actual arm/takeoff
"""

# ─── stdlib ──────────────────────────────────────────────────────────────────
import argparse
import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# ─── third-party ─────────────────────────────────────────────────────────────
import cv2
import cv2.aruco as aruco
import numpy as np
from pymavlink import mavutil

# ─── local ───────────────────────────────────────────────────────────────────
import v2v_bridge

# ═════════════════════════════════════════════════════════════════════════════
# GLOBAL CONSTANTS
# ═════════════════════════════════════════════════════════════════════════════

# --- Flight controller serial port ---
FC_CONNECTION_STRING = "/dev/ttyACM0"
FC_BAUD_RATE         = 57600

# --- Takeoff / hover ---
HOVER_WAIT_S         = 5.0     # seconds to hover and stabilise after reaching alt
ALT_TOLERANCE_M      = 0.10    # within this of target = "reached"
TAKEOFF_TIMEOUT_S    = 25.0
TAKEOFF_MIN_CLIMB_M  = 0.10    # consider airborne once above this height
ARM_TIMEOUT_S        = 10.0
CLIMB_LOOP_DT        = 0.10
HOVER_LOOP_DT        = 0.10
LAND_TIMEOUT_S       = 60.0

# --- RC emergency switch ---
EMERGENCY_LAND_CH            = 7
EMERGENCY_LAND_PWM_THRESHOLD = 1800

# --- ArUco detection ---
MARKER_SIZE_M        = 0.254   # default marker side in metres (10 in); override with --marker-size
SEND_HZ              = 15.0    # velocity command rate during ArUco tracking

# --- moveToCenter tuning ---
MAX_CLIMB_ALT_M      = 5.0     # ceiling while searching for markers
CENTER_FORWARD_MPS   = 1.5     # forward speed toward field centre
CENTER_STEP_S        = 0.5     # duration of each velocity step
CENTER_CLIMB_STEP_M  = 0.10    # altitude increment per step when markers not seen

# --- UGV nav / ground station interface ---
TURN_STEP_DEG        = 12.0    # fixed yaw correction sent to UGV per ArUco turn cmd
TURN_RATE_DEG_S      = 10.0
TURN_TOLERANCE_DEG   = 5.0
STEP_MIN_M           = 0.20    # min forward distance per UGV step
STEP_MAX_M           = 0.60    # max forward distance per UGV step
UGV_DRIVE_SPEED_MPS  = 1.5     # used only for cooldown timing
MARKER_TIMEOUT_S     = 0.75    # stop UGV if markers lost > this
TURN_THRESHOLD_DEG   = 12.0    # heading error to trigger a turn command
STOP_DISTANCE_M      = 0.28    # UGV considers destination reached at this range

# ═════════════════════════════════════════════════════════════════════════════
# LOGGING
# ═════════════════════════════════════════════════════════════════════════════

LOG_FILE = "Challenge2_official_log.txt"

def log_event(text): # helper to write required logs
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}\n"
    print(line.strip())
    with open(LOG_FILE, "a") as f: f.write(line)


# ═════════════════════════════════════════════════════════════════════════════
# ZED X CAMERA INTERFACE
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class MarkerPose:
    marker_id:  int
    rvec:       np.ndarray
    tvec:       np.ndarray
    center_px:  Tuple[int, int]
    corners:    np.ndarray


class ZEDCamera:
    """
    Wraps the ZED X camera via pyzed.sl.

    Always uses HD1080 resolution and reads its own calibration parameters
    from the SDK so no external YAML file is needed.
    """

    def __init__(self, fps: int = 30):
        try:
            import pyzed.sl as sl
        except ImportError as exc:
            raise RuntimeError(
                "pyzed.sl is not installed.  Install the ZED SDK Python API."
            ) from exc

        self.sl  = sl
        self.zed = sl.Camera()
        self.fps = fps

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs:   Optional[np.ndarray] = None

        self._open()

    def _open(self):
        sl = self.sl
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD1080
        init.camera_fps        = self.fps
        init.depth_mode        = sl.DEPTH_MODE.NONE   # depth not needed

        err = self.zed.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED X failed to open: {err}")

        info  = self.zed.get_camera_information()
        calib = info.camera_configuration.calibration_parameters.left_cam

        self.camera_matrix = np.array(
            [[calib.fx, 0.0,      calib.cx],
             [0.0,      calib.fy, calib.cy],
             [0.0,      0.0,      1.0     ]],
            dtype=np.float64,
        )

        dist = np.array(calib.disto, dtype=np.float64).flatten()
        if dist.size >= 5:
            self.dist_coeffs = dist[:5].reshape(-1, 1)
        elif dist.size > 0:
            self.dist_coeffs = dist.reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        log_event(f"ZED X opened — fx={calib.fx:.1f}  fy={calib.fy:.1f}  "
            f"cx={calib.cx:.1f}  cy={calib.cy:.1f}")

    def get_frame(self) -> Optional[np.ndarray]:
        sl = self.sl
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return None

        img = sl.Mat()
        self.zed.retrieve_image(img, sl.VIEW.LEFT)
        frame = img.get_data()

        # ZED returns BGRA; convert to BGR for OpenCV
        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        else:
            frame = frame.copy()

        return frame

    def close(self):
        if self.zed is not None:
            self.zed.close()
        cv2.destroyAllWindows()


# ═════════════════════════════════════════════════════════════════════════════
# ARUCO DETECTOR
# ═════════════════════════════════════════════════════════════════════════════

class ArucoDetector:
    """Detects ArUco markers and estimates their 3-D poses via solvePnP."""

    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs:   np.ndarray,
        marker_size_m: float,
        dictionary_name: int = aruco.DICT_6X6_1000,
    ):
        self.camera_matrix = camera_matrix
        self.dist_coeffs   = dist_coeffs
        self.marker_size_m = marker_size_m

        aruco_dict = aruco.getPredefinedDictionary(dictionary_name)

        if hasattr(aruco, "ArucoDetector"):
            params          = aruco.DetectorParameters()
            # improve sub-pixel corner accuracy
            params.cornerRefinementMethod      = aruco.CORNER_REFINE_SUBPIX
            params.adaptiveThreshWinSizeMin    = 3
            params.adaptiveThreshWinSizeMax    = 23
            params.adaptiveThreshWinSizeStep   = 10
            params.polygonalApproxAccuracyRate = 0.05
            self._detector     = aruco.ArucoDetector(aruco_dict, params)
            self._use_new_api  = True
        else:
            self._aruco_dict   = aruco_dict
            params             = aruco.DetectorParameters_create()
            self._params       = params
            self._use_new_api  = False

    def detect(self, frame: np.ndarray) -> Dict[int, MarkerPose]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self._use_new_api:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._params)

        poses: Dict[int, MarkerPose] = {}
        if ids is None or len(ids) == 0:
            return poses

        for i, mid in enumerate(ids.flatten()):
            rvec, tvec = self._solve_pose(corners[i])
            if rvec is None:
                continue
            center = np.mean(corners[i][0], axis=0).astype(int)
            poses[int(mid)] = MarkerPose(
                marker_id=int(mid),
                rvec=rvec,
                tvec=tvec.reshape(3),
                center_px=(int(center[0]), int(center[1])),
                corners=corners[i][0],
            )

        return poses

    def _solve_pose(self, corner: np.ndarray):
        h = self.marker_size_m / 2.0
        obj_pts = np.array(
            [[-h,  h, 0.0],
             [ h,  h, 0.0],
             [ h, -h, 0.0],
             [-h, -h, 0.0]],
            dtype=np.float32,
        )
        img_pts = corner.reshape((4, 2)).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        return (rvec, tvec) if ok else (None, None)

    def draw(self, frame: np.ndarray, poses: Dict[int, MarkerPose]):
        if not poses:
            return
        corners_list = [p.corners.reshape(1, 4, 2).astype(np.float32)
                        for p in poses.values()]
        ids_arr = np.array([[p.marker_id] for p in poses.values()],
                           dtype=np.int32)
        aruco.drawDetectedMarkers(frame, corners_list, ids_arr)

        for p in poses.values():
            cv2.drawFrameAxes(
                frame, self.camera_matrix, self.dist_coeffs,
                p.rvec, p.tvec.reshape(3, 1),
                self.marker_size_m * 0.5,
            )
            x, y = p.center_px
            cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)
            cv2.putText(
                frame, f"ID {p.marker_id} ({x},{y})",
                (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 255, 0), 2, cv2.LINE_AA,
            )


# ═════════════════════════════════════════════════════════════════════════════
# MAVLINK HELPERS  (raw pymavlink — no dronekit)
# ═════════════════════════════════════════════════════════════════════════════

def fc_connect() -> mavutil.mavfile:
    log_event(f"Connecting to flight controller at {FC_CONNECTION_STRING} ...")
    if FC_CONNECTION_STRING.startswith(("udp:", "tcp:")):
        master = mavutil.mavlink_connection(FC_CONNECTION_STRING)
    else:
        master = mavutil.mavlink_connection(FC_CONNECTION_STRING,
                                            baud=FC_BAUD_RATE)
    hb = master.wait_heartbeat(timeout=10)
    if not hb:
        raise RuntimeError("No heartbeat — check flight controller connection.")
    log_event("Heartbeat received.")
    _request_streams(master)
    return master


def _request_streams(master):
    try:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    except Exception:
        pass

    for msg_id, hz in [
        (mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,    15),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,            5),
        (mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW,        10),
    ]:
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
        except Exception:
            pass


def fc_set_mode(master, mode_name: str):
    mapping = master.mode_mapping()
    if mode_name not in mapping:
        raise RuntimeError(
            f"Mode '{mode_name}' not in mapping. Available: {list(mapping)}")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode_name],
    )
    log_event(f"Mode set -> {mode_name}")
    time.sleep(0.8)


def fc_arm(master) -> bool:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    log_event("ARM sent — waiting for confirmation ...")
    deadline = time.time() + ARM_TIMEOUT_S
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log_event("Motors ARMED.")
            return True
    log_event("[WARN] Arm timed out.")
    return False


def fc_disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0,
    )
    log_event("Disarm command sent.")


def fc_takeoff(master, alt_m: float) -> bool:
    """Issue MAV_CMD_NAV_TAKEOFF and wait for ACK."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, float(alt_m),
    )
    log_event("Takeoff command sent — waiting for ACK ...")
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    if ack is None:
        log_event("[WARN] No ACK for takeoff.")
        return False
    if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        log_event(f"[WARN] Takeoff rejected (result={ack.result}).")
        return False
    log_event("Takeoff accepted.")
    return True


def send_velocity(master, vx: float, vy: float, vz: float):
    """
    Body-frame velocity in MAV_FRAME_BODY_NED.
    vx = forward, vy = right, vz = down (positive = descend).
    Uses SET_POSITION_TARGET_LOCAL_NED with velocity-only type mask.
    """
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,   # velocity only: ignore pos, accel, yaw
        0, 0, 0,              # position (ignored)
        float(vx), float(vy), float(vz),  # velocity
        0, 0, 0,              # acceleration (ignored)
        0, 0,                 # yaw, yaw_rate (ignored)
    )


def send_landing_target(master, x_b: float, y_b: float, z_b: float):
    dist = abs(z_b)
    master.mav.landing_target_send(
        int(time.time() * 1e6), 0,
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        0.0, 0.0,
        dist, 0.0, 0.0,
        x_b, y_b, dist,
        (1.0, 0.0, 0.0, 0.0), 0, 1,
    )


def get_rangefinder_alt(master) -> Optional[float]:
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    while msg:
        alt = msg.current_distance / 100.0
        if alt > 0.01:
            return alt
        msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    return None


def get_baro_alt(master) -> Optional[float]:
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    return msg.relative_alt / 1000.0 if msg else None


def get_altitude(master) -> Tuple[Optional[float], str]:
    rng = get_rangefinder_alt(master)
    if rng is not None:
        return rng, "lidar"
    baro = get_baro_alt(master)
    if baro is not None:
        return baro, "baro"
    return None, "none"


def check_rc_emergency(master) -> bool:
    msg = master.recv_match(type="RC_CHANNELS", blocking=False)
    if msg is not None:
        ch = getattr(msg, f"chan{EMERGENCY_LAND_CH}_raw", None)
        if ch is not None and int(ch) > EMERGENCY_LAND_PWM_THRESHOLD:
            return True
    return False


def land_safely(master):
    log_event("Commanding LAND mode ...")
    fc_set_mode(master, "LAND")

    log_event("Waiting for auto-disarm ...")
    deadline = time.time() + LAND_TIMEOUT_S
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
        if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log_event("Motors disarmed — touchdown confirmed.")
            return
        alt, src = get_altitude(master)
        if alt is not None:
            print(f"  Alt: {alt:.2f} m ({src})", end="\r", flush=True)
        time.sleep(0.25)

    log_event("[WARN] Land timeout — sending disarm fallback.")
    fc_disarm(master)
    time.sleep(2.0)


# ═════════════════════════════════════════════════════════════════════════════
# TAKEOFF PHASE
# ═════════════════════════════════════════════════════════════════════════════

def run_takeoff(master, target_alt_m: float) -> bool:
    """
    Set GUIDED mode, arm, issue takeoff, wait for target altitude.
    Returns True if airborne at (approximately) the target altitude.
    """
    fc_set_mode(master, "GUIDED")

    if not fc_arm(master):
        log_event("Arm failed — aborting takeoff.")
        return False

    if not fc_takeoff(master, target_alt_m):
        log_event("Takeoff command rejected — aborting.")
        return False

    log_event(f"Climbing to {target_alt_m:.2f} m ...")
    t0              = time.time()
    airborne        = False
    last_known_alt  = None

    while True:
        if check_rc_emergency(master):
            log_event("RC EMERGENCY — landing immediately.")
            land_safely(master)
            return False

        alt, src = get_altitude(master)
        if alt is not None:
            last_known_alt = alt
            print(f"  Alt: {alt:.2f} m ({src})  target: {target_alt_m:.2f} m",
                  end="\r", flush=True)
            if alt >= TAKEOFF_MIN_CLIMB_M:
                airborne = True

        elapsed = time.time() - t0
        if elapsed > TAKEOFF_TIMEOUT_S:
            if not airborne:
                log_event("Takeoff timeout — vehicle never left the ground.")
                land_safely(master)
                return False
            else:
                log_event("Takeoff timeout — proceeding at current altitude.")
                break

        if last_known_alt is not None and \
                last_known_alt >= (target_alt_m - ALT_TOLERANCE_M):
            print()
            log_event(f"Target altitude reached: {last_known_alt:.2f} m")
            break

        time.sleep(CLIMB_LOOP_DT)

    return True


# ═════════════════════════════════════════════════════════════════════════════
# HOVER PHASE
# ═════════════════════════════════════════════════════════════════════════════

def run_hover(master):
    """Hold position for HOVER_WAIT_S seconds using zero-velocity commands."""
    log_event(f"Hovering for {HOVER_WAIT_S:.0f} s to stabilise ...")
    t0 = time.time()
    while (time.time() - t0) < HOVER_WAIT_S:
        if check_rc_emergency(master):
            log_event("RC EMERGENCY — landing.")
            land_safely(master)
            return False
        send_velocity(master, 0.0, 0.0, 0.0)
        time.sleep(HOVER_LOOP_DT)
    print()
    log_event("Hover complete.")
    return True


# ═════════════════════════════════════════════════════════════════════════════
# MOVE-TO-CENTRE PHASE
# ═════════════════════════════════════════════════════════════════════════════

def run_move_to_center(
    master,
    cam: ZEDCamera,
    detector: ArucoDetector,
    ugv_marker_id: int,
    dest_marker_id: int,
    target_alt_m: float,
) -> bool:
    """
    Fly diagonally toward the centre of a 15×15 yd field while slowly
    climbing.  Keeps sending forward body-frame velocity commands via
    send_velocity().  Returns True once both markers are visible, False on
    failure.
    """
    log_event("moveToCenter — flying toward field centre while searching for markers ...")

    # Diagonal distance: corner to centre of 15×15 yd square in metres
    dist_to_center_m = (15.0 / math.sqrt(2.0)) * 0.9144   # ≈ 9.68 m
    total_time_s     = dist_to_center_m / CENTER_FORWARD_MPS
    steps            = max(1, int(total_time_s / CENTER_STEP_S))
    fail_count       = 0

    # --- Phase 1: move toward centre, climbing if needed ---
    for step in range(steps):
        if check_rc_emergency(master):
            log_event("RC EMERGENCY during moveToCenter.")
            land_safely(master)
            return False

        frame = cam.get_frame()
        if frame is None:
            fail_count += 1
            log_event(f"Camera read failed ({fail_count}).")
            if fail_count > 5:
                return False
            continue

        poses = detector.detect(frame)
        if _both_visible(poses, ugv_marker_id, dest_marker_id):
            log_event(f"Both markers visible at step {step}/{steps} — entering nav loop.")
            send_velocity(master, 0.0, 0.0, 0.0)
            return True

        # Climb slowly if below target altitude
        alt, _ = get_altitude(master)
        if alt is not None and alt < target_alt_m:
            # small upward correction (positive vz = descend in NED, so use negative)
            send_velocity(master, CENTER_FORWARD_MPS, 0.0,
                          -CENTER_CLIMB_STEP_M / CENTER_STEP_S)
        else:
            send_velocity(master, CENTER_FORWARD_MPS, 0.0, 0.0)

        time.sleep(CENTER_STEP_S)

    # --- Phase 2: reached centre but markers still not found — keep climbing ---
    log_event("Reached estimated centre — hovering and climbing until markers visible.")
    while True:
        if check_rc_emergency(master):
            log_event("RC EMERGENCY during post-centre climb.")
            land_safely(master)
            return False

        frame = cam.get_frame()
        if frame is None:
            fail_count += 1
            if fail_count > 5:
                return False
            continue

        poses = detector.detect(frame)
        if _both_visible(poses, ugv_marker_id, dest_marker_id):
            log_event("Both markers found after centre climb — entering nav loop.")
            send_velocity(master, 0.0, 0.0, 0.0)
            return True

        alt, _ = get_altitude(master)
        if alt is not None:
            if alt >= MAX_CLIMB_ALT_M:
                log_event("Max altitude reached — markers not found.")
                return False
            # inch upward
            send_velocity(master, 0.0, 0.0, -CENTER_CLIMB_STEP_M)

        time.sleep(1.0)


def _both_visible(
    poses: Dict[int, MarkerPose],
    id_a: int,
    id_b: int,
) -> bool:
    return id_a in poses and id_b in poses


# ═════════════════════════════════════════════════════════════════════════════
# UGV COMMANDER
# ═════════════════════════════════════════════════════════════════════════════

class UGVCommander:
    """
    Sends ArUco-derived navigation commands to the ground station via the
    ESP32 V2V bridge.
    """

    def __init__(self, bridge_port: str, bridge_baud: int = 115200):
        self.bridge = v2v_bridge.V2VBridge(bridge_port, baud=bridge_baud,
                                           name="UGV-Bridge")
        self.seq             = 1
        self.arm_sent        = False
        self.last_motion     = "idle"
        self.next_drive_time = 0.0
        self.last_seen_time  = 0.0
        self.last_status     = "init"

    def connect(self):
        self.bridge.connect()
        self.bridge.send_message("challenge2 ugv navigator online")

    def close(self):
        try:
            self._send_stop(force=True)
        except Exception:
            pass
        self.bridge.stop()

    def _seq(self) -> int:
        v = self.seq
        self.seq += 1
        return v

    def ensure_armed(self):
        if self.arm_sent:
            return
        log_event("[UGV] Sending ARM ...")
        self.bridge.send_command(self._seq(), v2v_bridge.CMD_ARM, 0)
        self.arm_sent    = True
        self.last_status = "arming sent"
        time.sleep(0.25)

    def send_turn_left(self):
        if self.last_motion == "turn_left":
            return
        log_event("[UGV] TURN LEFT")
        self.bridge.send_command(self._seq(), v2v_bridge.CMD_TURN_LEFT, 0)
        self.last_motion = "turn_left"
        self.last_status = "turning left"

    def send_turn_right(self):
        if self.last_motion == "turn_right":
            return
        log_event("[UGV] TURN RIGHT")
        self.bridge.send_command(self._seq(), v2v_bridge.CMD_TURN_RIGHT, 0)
        self.last_motion = "turn_right"
        self.last_status = "turning right"

    def _send_stop(self, force: bool = False):
        if not force and self.last_motion == "stopped":
            return
        log_event("[UGV] STOP")
        self.bridge.send_command(self._seq(), v2v_bridge.CMD_STOP, 0)
        self.last_motion = "stopped"
        self.last_status = "stopped"

    def send_forward_step(self, step_m: float):
        now = time.time()
        if now < self.next_drive_time:
            return
        step_m   = max(STEP_MIN_M, min(step_m, STEP_MAX_M))
        duration = max(step_m / max(UGV_DRIVE_SPEED_MPS, 1e-6), 0.15)
        log_event(f"[UGV] FORWARD STEP {step_m:.3f} m  (GOTO message)")
        # Send as a GOTO message so ground_station.py uses velocity drive
        self.bridge.send_message(f"GOTO:{step_m:.3f},0")
        self.last_motion     = "forward"
        self.last_status     = f"forward {step_m:.2f} m"
        self.next_drive_time = now + duration + 0.25

    def handle_marker_loss(self):
        now = time.time()
        if self.last_seen_time <= 0.0:
            return
        if (now - self.last_seen_time) > MARKER_TIMEOUT_S:
            self._send_stop()
            self.last_status = "marker lost -> stopped"


# ═════════════════════════════════════════════════════════════════════════════
# UGV GUIDANCE LOOP  (overhead ArUco nav)
# ═════════════════════════════════════════════════════════════════════════════

FORWARD_AXIS_MAP = {
    "+x": np.array([1.0, 0.0, 0.0], dtype=np.float64),
    "-x": np.array([-1.0, 0.0, 0.0], dtype=np.float64),
    "+y": np.array([0.0, 1.0, 0.0], dtype=np.float64),
    "-y": np.array([0.0, -1.0, 0.0], dtype=np.float64),
}


def _marker_forward_px(
    pose: MarkerPose,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_size_m: float,
    axis_name: str,
) -> Optional[np.ndarray]:
    axis      = FORWARD_AXIS_MAP[axis_name]
    tip_local = axis * (marker_size_m * 0.75)
    pts3d     = np.array([[0.0, 0.0, 0.0], tip_local], dtype=np.float32)
    img_pts, _ = cv2.projectPoints(
        pts3d, pose.rvec, pose.tvec.reshape(3, 1),
        camera_matrix, dist_coeffs,
    )
    img_pts = img_pts.reshape(-1, 2)
    direction = img_pts[1] - img_pts[0]
    return direction if np.linalg.norm(direction) >= 1e-6 else None


def _signed_angle_deg(va: np.ndarray, vb: np.ndarray) -> float:
    a = np.asarray(va, dtype=np.float64).reshape(2)
    b = np.asarray(vb, dtype=np.float64).reshape(2)
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    a /= na
    b /= nb
    dot   = float(np.clip(np.dot(a, b), -1.0, 1.0))
    cross = float(a[0] * b[1] - a[1] * b[0])
    return math.degrees(math.atan2(cross, dot))


def _draw_crosshair(frame: np.ndarray):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)
    cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)


def _draw_distance_overlay(frame, pa: MarkerPose, pb: MarkerPose):
    diff   = pb.tvec.reshape(3) - pa.tvec.reshape(3)
    dist_m = np.linalg.norm(diff)
    cv2.line(frame, pa.center_px, pb.center_px, (0, 255, 0), 3)
    mid = ((pa.center_px[0] + pb.center_px[0]) // 2,
           (pa.center_px[1] + pb.center_px[1]) // 2)
    cv2.putText(frame, f"{dist_m * 1000:.0f} mm",
                (mid[0] + 10, mid[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)


def _draw_nav_overlay(
    frame, ugv: MarkerPose, dst: MarkerPose,
    heading_err: float, status: str,
    stop_dist: float,
    cam_mat: np.ndarray, dist_coeffs: np.ndarray,
    marker_size: float, axis_name: str,
):
    uc = np.array(ugv.center_px, dtype=np.int32)
    dc = np.array(dst.center_px, dtype=np.int32)
    cv2.arrowedLine(frame, tuple(uc), tuple(dc), (0, 255, 255), 2, tipLength=0.08)

    fwd = _marker_forward_px(ugv, cam_mat, dist_coeffs, marker_size, axis_name)
    if fwd is not None:
        tip = uc + np.round(fwd).astype(np.int32)
        cv2.arrowedLine(frame, tuple(uc), tuple(tip), (255, 0, 255), 2,
                        tipLength=0.18)

    dist_m = float(np.linalg.norm(dst.tvec.reshape(3) - ugv.tvec.reshape(3)))
    lines  = [
        f"UGV: {ugv.marker_id}",
        f"DEST: {dst.marker_id}",
        f"HdgErr: {heading_err:+.1f} deg",
        f"Dist: {dist_m:.3f} m",
        f"Stop: {stop_dist:.3f} m",
        f"State: {status}",
    ]

    x0, y0  = 20, 20
    pad     = 10
    line_h  = 28
    font    = cv2.FONT_HERSHEY_SIMPLEX
    max_w   = max((cv2.getTextSize(t, font, 0.7, 2)[0][0] for t in lines),
                  default=100)
    bw      = max_w + 2 * pad
    bh      = len(lines) * line_h + 2 * pad
    ovl     = frame.copy()
    cv2.rectangle(ovl, (x0, y0), (x0 + bw, y0 + bh), (255, 255, 255), -1)
    frame[:] = cv2.addWeighted(ovl, 0.82, frame, 0.18, 0)
    y = y0 + pad + 20
    for t in lines:
        cv2.putText(frame, t, (x0 + pad, y), font, 0.7, (0, 0, 0), 2,
                    cv2.LINE_AA)
        y += line_h


def run_guide_ugv(
    cam: ZEDCamera,
    detector: ArucoDetector,
    ugv_commander: UGVCommander,
    ugv_marker_id: int,
    dest_marker_id: int,
    marker_size_m: float,
    stop_distance_m: float = STOP_DISTANCE_M,
    turn_threshold_deg: float = TURN_THRESHOLD_DEG,
    step_min_m: float = STEP_MIN_M,
    step_max_m: float = STEP_MAX_M,
    forward_axis: str = "+y",
):
    """
    Main overhead ArUco guidance loop.  Runs until the UGV reaches the
    destination marker or the user presses 'q'.
    """
    log_event(f"guideUGV — tracking UGV marker {ugv_marker_id} "
        f"toward destination {dest_marker_id}.")

    while True:
        frame = cam.get_frame()
        if frame is None:
            ugv_commander.handle_marker_loss()
            continue

        poses   = detector.detect(frame)
        display = frame.copy()
        _draw_crosshair(display)
        detector.draw(display, poses)

        ugv_p  = poses.get(ugv_marker_id)
        dest_p = poses.get(dest_marker_id)

        if ugv_p is not None and dest_p is not None:
            ugv_commander.last_seen_time = time.time()
            ugv_commander.ensure_armed()

            _draw_distance_overlay(display, ugv_p, dest_p)

            fwd_dir = _marker_forward_px(
                ugv_p, cam.camera_matrix, cam.dist_coeffs,
                marker_size_m, forward_axis,
            )
            tgt_dir = np.array(
                [dest_p.center_px[0] - ugv_p.center_px[0],
                 dest_p.center_px[1] - ugv_p.center_px[1]],
                dtype=np.float64,
            )
            h_err = _signed_angle_deg(
                fwd_dir if fwd_dir is not None else tgt_dir, tgt_dir)
            dist_m = float(np.linalg.norm(
                dest_p.tvec.reshape(3) - ugv_p.tvec.reshape(3)))

            if dist_m <= stop_distance_m:
                ugv_commander._send_stop()
                ugv_commander.last_status = "destination reached"
                log_event("UGV reached destination!")
                # Show final frame then break
                _draw_nav_overlay(display, ugv_p, dest_p, h_err,
                                  ugv_commander.last_status, stop_distance_m,
                                  cam.camera_matrix, cam.dist_coeffs,
                                  marker_size_m, forward_axis)
                cv2.imshow("Challenge 2 — ArUco UGV Nav", display)
                cv2.waitKey(1)
                break

            elif abs(h_err) > turn_threshold_deg:
                if h_err > 0.0:
                    ugv_commander.send_turn_right()
                else:
                    ugv_commander.send_turn_left()
            else:
                if ugv_commander.last_motion in ("turn_left", "turn_right"):
                    ugv_commander._send_stop()
                    time.sleep(0.15)
                remaining = max(0.0, dist_m - stop_distance_m)
                step_m    = max(step_min_m, min(remaining * 0.5, step_max_m))
                ugv_commander.send_forward_step(step_m)

            _draw_nav_overlay(display, ugv_p, dest_p, h_err,
                              ugv_commander.last_status, stop_distance_m,
                              cam.camera_matrix, cam.dist_coeffs,
                              marker_size_m, forward_axis)
        else:
            ugv_commander.handle_marker_loss()
            cv2.putText(
                display,
                f"Waiting for markers {ugv_marker_id} & {dest_marker_id}",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255),
                2, cv2.LINE_AA,
            )
            cv2.putText(
                display, f"State: {ugv_commander.last_status}",
                (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255),
                2, cv2.LINE_AA,
            )

        cv2.imshow("Challenge 2 — ArUco UGV Nav", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            log_event("User pressed 'q' — exiting nav loop.")
            break
        elif key == ord("s"):
            fname = "challenge2_screenshot.png"
            cv2.imwrite(fname, display)
            log_event(f"Screenshot saved: {fname}")


# ═════════════════════════════════════════════════════════════════════════════
# ARGUMENT PARSING
# ═════════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description="Challenge 2 — ZED X camera, GUIDED takeoff, "
                    "moveToCenter, then ArUco UGV navigation."
    )
    p.add_argument("--ugv-marker-id",  type=int,   default=5,
                   help="ArUco ID on the ground vehicle.")
    p.add_argument("--dest-marker-id", type=int,   default=0,
                   help="Destination ArUco ID.")
    p.add_argument("--marker-size",    type=float, default=MARKER_SIZE_M,
                   help="Marker side length in metres.")
    p.add_argument("--target-alt",     type=float, default=0.30,
                   help="Takeoff target altitude in metres.")
    p.add_argument("--bridge-port",    type=str,   default="/dev/ttyUSB0",
                   help="Serial port for ESP32 V2V bridge.")
    p.add_argument("--bridge-baud",    type=int,   default=115200,
                   help="Baud rate for ESP32 V2V bridge.")
    p.add_argument("--fps",            type=int,   default=30,
                   help="ZED X camera FPS.")
    p.add_argument("--dict",           type=str,   default="DICT_6X6_1000",
                   help="ArUco dictionary name.")
    p.add_argument("--stop-distance-m", type=float, default=STOP_DISTANCE_M,
                   help="UGV stop radius in metres.")
    p.add_argument("--turn-threshold-deg", type=float,
                   default=TURN_THRESHOLD_DEG,
                   help="Heading error before issuing a turn command (deg).")
    p.add_argument("--step-min-m",     type=float, default=STEP_MIN_M,
                   help="Minimum forward UGV step in metres.")
    p.add_argument("--step-max-m",     type=float, default=STEP_MAX_M,
                   help="Maximum forward UGV step in metres.")
    p.add_argument("--ugv-forward-axis",
                   choices=["+x", "-x", "+y", "-y"], default="+y",
                   help="Which marker axis points forward on the UGV.")
    p.add_argument("--testing",        action="store_true",
                   help="Dry-run mode: skip arm/takeoff (camera + nav only).")
    return p.parse_args()


def _get_dict(name: str) -> int:
    if not hasattr(aruco, name):
        valid = [x for x in dir(aruco) if x.startswith("DICT_")]
        raise ValueError(f"Unknown dictionary '{name}'. Examples: {valid[:8]}")
    return getattr(aruco, name)


# ═════════════════════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════════════════════

def main():
    args = parse_args()

    # ── Open ZED X camera (always — no fallback) ──────────────────────────
    log_event("Opening ZED X camera ...")
    try:
        cam = ZEDCamera(fps=args.fps)
    except Exception as exc:
        log_event(f"Failed to open ZED X: {exc}")
        return

    detector = ArucoDetector(
        cam.camera_matrix,
        cam.dist_coeffs,
        marker_size_m=args.marker_size,
        dictionary_name=_get_dict(args.dict),
    )

    # ── Connect UGV bridge ────────────────────────────────────────────────
    ugv_cmd = UGVCommander(args.bridge_port, args.bridge_baud)
    ugv_cmd.connect()

    # ── Connect to flight controller ──────────────────────────────────────
    master = None
    if not args.testing:
        try:
            master = fc_connect()
        except Exception as exc:
            log_event(f"Flight controller connection failed: {exc}")
            cam.close()
            ugv_cmd.close()
            return

    try:
        if not args.testing:
            # ── PHASE 1: Arm + takeoff (GUIDED) ───────────────────────────
            if not run_takeoff(master, args.target_alt):
                log_event("Takeoff failed — aborting mission.")
                return

            # ── PHASE 2: Hover briefly to stabilise ───────────────────────
            if not run_hover(master):
                return

            # ── PHASE 3: Fly to centre, climb until both markers visible ──
            found = run_move_to_center(
                master, cam, detector,
                args.ugv_marker_id, args.dest_marker_id,
                args.target_alt,
            )
            if not found:
                log_event("Could not find both markers — landing.")
                land_safely(master)
                return

        else:
            log_event("*** TESTING MODE — skipping arm / takeoff / moveToCenter ***")
            log_event("Entering UGV guidance loop directly ...")

        # ── PHASE 4: Overhead ArUco UGV guidance ──────────────────────────
        run_guide_ugv(
            cam, detector, ugv_cmd,
            ugv_marker_id=args.ugv_marker_id,
            dest_marker_id=args.dest_marker_id,
            marker_size_m=args.marker_size,
            stop_distance_m=args.stop_distance_m,
            turn_threshold_deg=args.turn_threshold_deg,
            step_min_m=args.step_min_m,
            step_max_m=args.step_max_m,
            forward_axis=args.ugv_forward_axis,
        )

        # ── PHASE 5: Land ─────────────────────────────────────────────────
        if not args.testing and master is not None:
            log_event("Mission complete — landing.")
            land_safely(master)

    except KeyboardInterrupt:
        log_event("Keyboard interrupt — commanding land.")
        if not args.testing and master is not None:
            try:
                send_velocity(master, 0.0, 0.0, 0.0)
                land_safely(master)
            except Exception:
                pass

    finally:
        ugv_cmd.close()
        cam.close()
        log_event("Script finished.")


if __name__ == "__main__":
    main()

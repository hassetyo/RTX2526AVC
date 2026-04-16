"""
Challenge 1 - GPS Takeoff + ArUco Search + Follow + Precision Land + V2V UGV Command
======================================================================================
Mission Flow:
  1. TAKEOFF     - Arm and climb to TARGET_HEIGHT_M in GUIDED mode.
  2. SEARCH      - Fly a GPS lawnmower grid until ArUco marker ID 5 is detected.
  3. FOLLOW      - Track marker ID 5 with velocity commands, hovering above it.
  4. PREC_LOITER - Once stably centred, tighten the alignment loop.
  5. LANDING     - Descend at variable speed while locked on the moving marker.
  6. LANDED      - Disarm confirmed. Send CMD_MOVE_FORWARD to UGV via V2V bridge.

Hardware expected:
  - Flight controller on MAVLINK_CONN (pymavlink).
  - Camera (ZED or USB webcam) with calibration YAML on disk.
  - V2V ESP32 radio on V2V_PORT for UGV comms.

Set DESK_TESTING_NO_PROPELLERS = True to run the vision + state machine
on your desk without spinning propellers.
"""

import time
import math
import os
import sys

import cv2
import cv2.aruco as aruco
import numpy as np
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from pymavlink import mavutil

# ── Insert v2v_bridge sibling folder onto path ──────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_CAMERA_CODE_DIR = os.path.join(_HERE, "Camera Code")
if _CAMERA_CODE_DIR not in sys.path:
    sys.path.insert(0, _CAMERA_CODE_DIR)

from v2v_bridge import (
    V2VBridge,
    CMD_MOVE_FORWARD,
)

# ══════════════════════════════════════════════════════════════════════════════════
#  USER CONFIGURATION — edit these before your flight
# ══════════════════════════════════════════════════════════════════════════════════

# --- MAVLink ---
MAVLINK_CONN = "/dev/ttyACM0"      # e.g. udp:127.0.0.1:14550 for SITL
BAUD_RATE    = 921600

# --- V2V radio (ESP32 bridge to UGV) ---
V2V_PORT = "/dev/ttyUSB0"
V2V_BAUD = 115200

# --- Camera ---
USE_ZED        = False            # True = ZED SDK, False = USB webcam
CAMERA_INDEX   = 0
FRAME_W        = 1280
FRAME_H        = 720
CAM_FPS        = 30
CALIBRATION_YAML = os.path.join(_HERE, "calibration_chessboard.yaml")

# --- ArUco target ---
TARGET_MARKER_ID   = 5
MARKER_SIZE_M      = 0.3048         # 12 inches in metres (adjust to physical marker)

# --- Desk / bench testing (no props) ---
DESK_TESTING_NO_PROPELLERS = True   # <<<< SET False BEFORE FLYING OUTSIDE!

# --- Flight parameters ---
TAKEOFF_ALT_M          = 3.0        # metres AGL
TAKEOFF_ALT_TOLERANCE  = 0.30       # start vision once within this band
TAKEOFF_TIMEOUT_S      = 25.0
SEND_HZ                = 15.0       # MAVLink send rate

# --- Velocity controller gains ---
FOLLOW_KP         = 0.6             # proportional gain while following / loitering
FOLLOW_MAX_SPEED  = 0.8             # m/s horizontal clamp (walking pace — safe!)
LAND_DESCENT_RATE = 0.35            # m/s downward baseline during landing
LAND_LIDAR_KP     = 0.30            # scale lidar distance into vz

# --- State-machine thresholds ---
STABLE_FRAMES_NEEDED = 15           # consecutive frames before state transitions
LAND_LATERAL_ERR_M   = 0.20         # horizontal error to allow descent

# --- Search hover ---
# The drone climbs to TAKEOFF_ALT_M and holds position until marker ID 5
# appears in frame.  No GPS grid is flown.

# ══════════════════════════════════════════════════════════════════════════════════
#  CAMERA INTERFACE  (lifted from precision_land.py by friend)
# ══════════════════════════════════════════════════════════════════════════════════

@dataclass
class MarkerPose:
    marker_id:  int
    rvec:       np.ndarray
    tvec:       np.ndarray
    center_px:  Tuple[int, int]
    corners:    np.ndarray


class CameraInterface:
    def __init__(self, use_zed: bool = False, camera_index: int = 0,
                 width: int = 1280, height: int = 720, fps: int = 30):
        self.use_zed      = use_zed
        self.camera_index = camera_index
        self.width        = width
        self.height       = height
        self.fps          = fps
        self.cap         = None
        self.zed         = None
        self.sl          = None
        self.camera_matrix = None
        self.dist_coeffs   = None
        if self.use_zed:
            self._open_zed()
        else:
            self._open_standard()

    def _open_zed(self):
        import pyzed.sl as sl
        self.sl  = sl
        self.zed = sl.Camera()
        params   = sl.InitParameters()
        params.camera_resolution = sl.RESOLUTION.HD1080
        params.camera_fps        = self.fps
        params.depth_mode        = sl.DEPTH_MODE.NONE
        err = self.zed.open(params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED open failed: {err}")
        info  = self.zed.get_camera_information()
        calib = info.camera_configuration.calibration_parameters.left_cam
        self.camera_matrix = np.array(
            [[calib.fx, 0.0, calib.cx], [0.0, calib.fy, calib.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64)
        dist = np.array(calib.disto, dtype=np.float64).flatten()
        self.dist_coeffs = dist[:5].reshape(-1, 1) if dist.size >= 5 else np.zeros((5, 1))

    def _open_standard(self):
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {self.camera_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS,          self.fps)

    def load_calibration(self, yaml_path: Optional[str]):
        if yaml_path and os.path.exists(yaml_path):
            fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
            K  = fs.getNode("K").mat()
            D  = fs.getNode("D").mat()
            fs.release()
            if K is not None and D is not None:
                self.camera_matrix = np.array(K, dtype=np.float64)
                self.dist_coeffs   = np.array(D, dtype=np.float64)
                print(f"[CAM] Loaded calibration from {yaml_path}")
            else:
                print("[CAM] WARN: Calibration file found but K/D nodes are empty.")
        else:
            print("[CAM] WARN: No calibration YAML found — using default estimate.")
            # Provide a rough default so pose estimation does not crash
            cx = self.width / 2.0
            cy = self.height / 2.0
            f  = 800.0
            self.camera_matrix = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float64)
            self.dist_coeffs   = np.zeros((5, 1), dtype=np.float64)

    def get_frame(self) -> Optional[np.ndarray]:
        if self.use_zed:
            if self.zed.grab() != self.sl.ERROR_CODE.SUCCESS:
                return None
            img = self.sl.Mat()
            self.zed.retrieve_image(img, self.sl.VIEW.LEFT)
            frame = img.get_data()
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) if frame.shape[-1] == 4 else frame.copy()
        else:
            ret, frame = self.cap.read()
            return frame if ret else None

    def close(self):
        if self.use_zed and self.zed is not None:
            self.zed.close()
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


class ArucoEstimator:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
                 dict_name: int = aruco.DICT_6X6_1000):
        self.camera_matrix = camera_matrix
        self.dist_coeffs   = dist_coeffs
        self.aruco_dict    = aruco.getPredefinedDictionary(dict_name)
        if hasattr(aruco, "ArucoDetector"):
            self.detector = aruco.ArucoDetector(self.aruco_dict, aruco.DetectorParameters())
            self._new_api = True
        else:
            self.params   = aruco.DetectorParameters_create()
            self._new_api = False

    def detect(self, frame: np.ndarray, marker_size_m: float) -> Dict[int, MarkerPose]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        poses: Dict[int, MarkerPose] = {}
        if ids is None or len(ids) == 0:
            return poses
        half = marker_size_m / 2.0
        obj_pts = np.array(
            [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
            dtype=np.float32)
        for i, mid in enumerate(ids.flatten()):
            img_pts = corners[i].reshape((4, 2)).astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            center = np.mean(corners[i][0], axis=0).astype(int)
            poses[int(mid)] = MarkerPose(
                marker_id=int(mid),
                rvec=rvec, tvec=tvec.reshape(3),
                center_px=(int(center[0]), int(center[1])),
                corners=corners[i][0])
        return poses


# ══════════════════════════════════════════════════════════════════════════════════
#  MAVLINK HELPERS
# ══════════════════════════════════════════════════════════════════════════════════

def _make_master():
    mav = mavutil.mavlink_connection(MAVLINK_CONN, baud=BAUD_RATE)
    print("[MAV] Waiting for heartbeat…")
    mav.wait_heartbeat()
    print(f"[MAV] Heartbeat received (sys={mav.target_system} comp={mav.target_component})")
    return mav


def change_mode(mav, mode_name: str) -> bool:
    mode_id = mav.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"[MAV] ERROR: Mode {mode_name!r} not found in vehicle mode map.")
        return False
    mav.set_mode(mode_id)
    return True


def wait_for_mode(mav, mode_name: str, timeout: float = 10.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if mav.flightmode == mode_name:
            return True
    return False


def get_relative_alt_m(mav) -> Optional[float]:
    """Non-blocking best-effort altitude read from autopilot."""
    msg = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg is not None and hasattr(msg, "relative_alt"):
        return float(msg.relative_alt) / 1000.0
    msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=False)
    if msg is not None and hasattr(msg, "z"):
        return float(-msg.z)
    msg = mav.recv_match(type="VFR_HUD", blocking=False)
    if msg is not None and hasattr(msg, "alt"):
        return float(msg.alt)
    return None


def get_lidar_m(mav) -> Optional[float]:
    msg = mav.recv_match(type="DISTANCE_SENSOR", blocking=False)
    if msg:
        return float(msg.current_distance) / 100.0
    return None



def send_rc_override(mav, roll=1500, pitch=1500, throttle=1500, yaw=1500):
    print("[MAV] Switching to GUIDED…")
    if not change_mode(mav, "GUIDED"):
        raise RuntimeError("GUIDED mode failed.")
    if not wait_for_mode(mav, "GUIDED", timeout=10.0):
        raise RuntimeError("Timed out waiting for GUIDED mode.")
    print("[MAV] Arming motors…")
    mav.arducopter_arm()
    mav.motors_armed_wait()
    print(f"[MAV] Armed! Sending takeoff to {alt_m:.1f} m…")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt_m)


def send_velocity_body(mav, vx: float, vy: float, vz: float):
    """Send velocity setpoint in BODY_NED frame (FRD)."""
    mav.mav.set_position_target_local_ned_send(
        0,
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,   # use vx, vy, vz only
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0)


def send_landing_target(mav, x_b: float, y_b: float, z_b: float):
    mav.mav.landing_target_send(
        int(time.time() * 1e6), 0,
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        0.0, 0.0, abs(z_b), 0.0, 0.0,
        x_b, y_b, abs(z_b),
        (1.0, 0.0, 0.0, 0.0), 0, 1)


def send_rc_override(mav, roll=1500, pitch=1500, throttle=1500, yaw=1500):
    rc = [65535] * 18
    rc[0], rc[1], rc[2], rc[3] = int(roll), int(pitch), int(throttle), int(yaw)
    mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, *rc)


def release_rc(mav):
    send_rc_override(mav, 0, 0, 0, 0)


# ══════════════════════════════════════════════════════════════════════════════════
#  MAIN MISSION
# ══════════════════════════════════════════════════════════════════════════════════

def main():
    # ── MAVLink connection ────────────────────────────────────────────────────────
    mav = _make_master()

    # ── Camera setup ─────────────────────────────────────────────────────────────
    print("[CAM] Initialising camera…")
    try:
        cam = CameraInterface(use_zed=USE_ZED, camera_index=CAMERA_INDEX,
                              width=FRAME_W, height=FRAME_H, fps=CAM_FPS)
        cam.load_calibration(CALIBRATION_YAML)
    except Exception as exc:
        print(f"[CAM] FATAL: {exc}")
        return

    estimator = ArucoEstimator(cam.camera_matrix, cam.dist_coeffs, aruco.DICT_6X6_1000)

    # ── V2V bridge (non-fatal — log warning if unavailable) ──────────────────────
    bridge: Optional[V2VBridge] = None
    try:
        bridge = V2VBridge(V2V_PORT, V2V_BAUD, name="UAV")
        bridge.connect()
        print(f"[V2V] Connected to UGV bridge on {V2V_PORT}")
    except Exception as exc:
        print(f"[V2V] WARNING: Could not open V2V bridge ({exc}). "
              "UGV command will be skipped.")

    # ── State-machine initialisation ─────────────────────────────────────────────
    # States: TAKEOFF → SEARCH → FOLLOW → PREC_LOITER → LANDING → LANDED
    state            = "SEARCH" if DESK_TESTING_NO_PROPELLERS else "TAKEOFF"
    stable_count     = 0
    last_send_t      = 0.0
    cmd_seq          = 1            # incrementing V2V command sequence number
    ugv_cmd_sent     = False        # only send once per mission
    ugv_move_sent    = False        # True once CMD_MOVE_FORWARD is sent at first detection
    takeoff_started  = DESK_TESTING_NO_PROPELLERS
    takeoff_start_t  = None

    # ── Arm + Takeoff ─────────────────────────────────────────────────────────────
    print("=" * 70)
    if DESK_TESTING_NO_PROPELLERS:
        print(">>> [DESK TESTING] No propellers — vision + state machine only.")
    else:
        print(">>> [REAL FLIGHT] Arming and taking off NOW. Stand clear!")
        arm_and_takeoff(mav, TAKEOFF_ALT_M)
        takeoff_start_t = time.time()
    print("=" * 70)

    try:
        while True:
            # ── Grab frame ───────────────────────────────────────────────────────
            frame = cam.get_frame()
            if frame is None:
                continue

            now = time.time()

            # ─────────────────────────────────────────────────────────────────────
            #  STATE: TAKEOFF
            # ─────────────────────────────────────────────────────────────────────
            if state == "TAKEOFF":
                alt = get_relative_alt_m(mav)
                label = f"TAKEOFF  ALT: {alt:.2f} m" if alt is not None else "TAKEOFF  ALT: …"
                cv2.putText(frame, label, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                if alt is not None and alt >= TAKEOFF_ALT_M * 0.15:
                    takeoff_started = True

                timeout_hit = (takeoff_start_t is not None and
                               (now - takeoff_start_t) > TAKEOFF_TIMEOUT_S)
                altitude_ok = (alt is not None and
                               alt >= TAKEOFF_ALT_M - TAKEOFF_ALT_TOLERANCE)

                if altitude_ok or timeout_hit:
                    reason = "altitude reached" if altitude_ok else "timeout"
                    print(f"[STATE] TAKEOFF done ({reason}). Hovering — waiting for ArUco ID {TARGET_MARKER_ID}. → SEARCH")
                    state = "SEARCH"

                cv2.putText(frame, f"STATE: {state}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                cv2.imshow("Challenge 1", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            # ─────────────────────────────────────────────────────────────────────
            #  Detect ArUco target on every non-TAKEOFF frame
            # ─────────────────────────────────────────────────────────────────────
            poses = estimator.detect(frame, MARKER_SIZE_M)
            target_pose: Optional[MarkerPose] = poses.get(TARGET_MARKER_ID)

            # Draw cross-hair
            cx_img = frame.shape[1] // 2
            cy_img = frame.shape[0] // 2
            cv2.line(frame, (0, cy_img), (frame.shape[1], cy_img), (0, 255, 0), 1)
            cv2.line(frame, (cx_img, 0), (cx_img, frame.shape[0]), (0, 255, 0), 1)

            # ─────────────────────────────────────────────────────────────────────
            #  STATE: SEARCH  —  hover at altitude and wait for marker ID 5
            # ─────────────────────────────────────────────────────────────────────
            if state == "SEARCH":
                if target_pose is not None:
                    stable_count += 1
                    cv2.putText(frame, f"FOUND ID {TARGET_MARKER_ID}! stable={stable_count}/{STABLE_FRAMES_NEEDED}",
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    # On the very first detection, immediately order the UGV to move forward
                    # so it starts driving as soon as the drone spots the marker.
                    if not ugv_move_sent:
                        ugv_move_sent = True
                        if bridge is not None:
                            bridge.send_command(cmd_seq, CMD_MOVE_FORWARD, estop=0)
                            cmd_seq += 1
                            print(f"[V2V] Marker detected — sent CMD_MOVE_FORWARD to UGV (seq={cmd_seq - 1}).")
                        else:
                            print("[V2V] Marker detected but bridge unavailable — UGV not notified.")

                    if stable_count >= STABLE_FRAMES_NEEDED:
                        print(f"[STATE] Marker {TARGET_MARKER_ID} confirmed ({stable_count} frames). → FOLLOW")
                        state = "FOLLOW"
                        stable_count = 0
                else:
                    stable_count = 0
                    # Hover in place — send a zero-velocity keep-alive so ArduPilot
                    # does not time out the guided-velocity controller.
                    if not DESK_TESTING_NO_PROPELLERS and (now - last_send_t) >= 1.0 / SEND_HZ:
                        send_velocity_body(mav, 0.0, 0.0, 0.0)
                        last_send_t = now
                    cv2.putText(frame, f"SEARCHING — hovering at {TAKEOFF_ALT_M:.1f} m, waiting for ID {TARGET_MARKER_ID}…",
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)

            # ─────────────────────────────────────────────────────────────────────
            #  STATES: FOLLOW / PREC_LOITER / LANDING  —  vision velocity control
            # ─────────────────────────────────────────────────────────────────────
            elif state in ("FOLLOW", "PREC_LOITER", "LANDING"):
                target_eb: Optional[Tuple[float, float, float]] = None

                if target_pose is not None:
                    # Convert camera XYZ to ArduPilot FRD body-frame
                    x_cam = float(target_pose.tvec[0])
                    y_cam = float(target_pose.tvec[1])
                    z_cam = float(target_pose.tvec[2])
                    x_b =  -y_cam   # forward (+X cam → -Y cam)
                    y_b =   x_cam   # right
                    z_b =   z_cam   # down

                    target_eb = (x_b, y_b, z_b)
                    stable_count += 1

                    # ── Visual feedback ───────────────────────────────────────────
                    cv2.line(frame,
                             (cx_img, cy_img),
                             target_pose.center_px,
                             (0, 0, 255), 3)
                    corners_arr = [target_pose.corners.reshape(1, 4, 2).astype(np.float32)]
                    aruco.drawDetectedMarkers(frame, corners_arr,
                                             np.array([[TARGET_MARKER_ID]]))
                    cv2.drawFrameAxes(frame, cam.camera_matrix, cam.dist_coeffs,
                                      target_pose.rvec, target_pose.tvec.reshape(3, 1),
                                      MARKER_SIZE_M * 0.5)
                    cv2.putText(frame, f"ID {TARGET_MARKER_ID} | FRD [{x_b:.2f}, {y_b:.2f}, {z_b:.2f}] m",
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

                else:
                    stable_count = 0
                    cv2.putText(frame, "TARGET LOST — HOVERING",
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # ── Send velocity commands ────────────────────────────────────────
                if now - last_send_t >= 1.0 / SEND_HZ:
                    if DESK_TESTING_NO_PROPELLERS:
                        # Desk test: spoof RC instead of real velocity commands
                        if target_eb is not None:
                            req_roll  = max(1300, min(1700, 1500 + int(target_eb[1] * 200.0)))
                            req_pitch = max(1300, min(1700, 1500 - int(target_eb[0] * 200.0)))
                            send_rc_override(mav, roll=req_roll, pitch=req_pitch, throttle=1550)
                        else:
                            send_rc_override(mav)  # centred
                    else:
                        if target_eb is not None:
                            vx = max(-FOLLOW_MAX_SPEED, min(FOLLOW_MAX_SPEED, target_eb[0] * FOLLOW_KP))
                            vy = max(-FOLLOW_MAX_SPEED, min(FOLLOW_MAX_SPEED, target_eb[1] * FOLLOW_KP))
                            vz = 0.0

                            if state == "LANDING":
                                lidar = get_lidar_m(mav)
                                if lidar is not None:
                                    vz = max(0.10, min(LAND_DESCENT_RATE, lidar * LAND_LIDAR_KP))
                                    cv2.putText(frame, f"LIDAR {lidar:.2f} m",
                                                (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                                else:
                                    vz = LAND_DESCENT_RATE
                                    cv2.putText(frame, "LIDAR N/A — BLIND DESCENT",
                                                (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                            send_velocity_body(mav, vx, vy, vz)
                            send_landing_target(mav, target_eb[0], target_eb[1], target_eb[2])
                        else:
                            send_velocity_body(mav, 0.0, 0.0, 0.0)  # hover / brake
                    last_send_t = now

                # ── State transitions ─────────────────────────────────────────────
                if state == "FOLLOW":
                    err_label = ""
                    if stable_count >= STABLE_FRAMES_NEEDED:
                        print("[STATE] Marker stably centred. Switching to GUIDED → PREC_LOITER")
                        if not DESK_TESTING_NO_PROPELLERS:
                            change_mode(mav, "GUIDED")
                        state = "PREC_LOITER"
                        stable_count = 0
                    cv2.putText(frame, f"FOLLOW stable={stable_count}/{STABLE_FRAMES_NEEDED}",
                                (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

                elif state == "PREC_LOITER":
                    if stable_count == 0:
                        print("[STATE] Lost marker in PREC_LOITER. → FOLLOW")
                        state = "FOLLOW"
                    elif target_eb is not None:
                        err_m = math.sqrt(target_eb[0] ** 2 + target_eb[1] ** 2)
                        cv2.putText(frame, f"ALIGN ERR {err_m:.2f} m (need <{LAND_LATERAL_ERR_M:.2f})",
                                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                        if err_m < LAND_LATERAL_ERR_M:
                            print("[STATE] Aligned. → LANDING (velocity descent)")
                            state = "LANDING"

                elif state == "LANDING":
                    if not DESK_TESTING_NO_PROPELLERS and not mav.motors_armed():
                        print("[STATE] Motors disarmed — touchdown confirmed. → LANDED")
                        state = "LANDED"

            # ─────────────────────────────────────────────────────────────────────
            #  STATE: LANDED  —  send V2V command then exit
            # ─────────────────────────────────────────────────────────────────────
            elif state == "LANDED":
                cv2.putText(frame, "LANDED — sending UGV command",
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("Challenge 1", frame)
                cv2.waitKey(1)

                if not ugv_cmd_sent:
                    ugv_cmd_sent = True
                    if bridge is not None:
                        bridge.send_command(cmd_seq, CMD_MOVE_FORWARD, estop=0)
                        cmd_seq += 1
                        print(f"[V2V] Sent CMD_MOVE_FORWARD (seq={cmd_seq - 1}) to UGV.")
                    else:
                        print("[V2V] Bridge unavailable — UGV command not sent.")
                break

            # ─────────────────────────────────────────────────────────────────────
            #  HUD overlay and display
            # ─────────────────────────────────────────────────────────────────────
            cv2.putText(frame, f"STATE: {state}", (20, frame.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.imshow("Challenge 1", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[EXIT] User pressed Q.")
                break

    # ─────────────────────────────────────────────────────────────────────────────
    finally:
        print("[CLEANUP] Shutting down…")
        if DESK_TESTING_NO_PROPELLERS:
            release_rc(mav)
        else:
            send_velocity_body(mav, 0.0, 0.0, 0.0)
        if bridge is not None:
            bridge.stop()
        cam.close()
        print("[CLEANUP] Done.")


if __name__ == "__main__":
    main()

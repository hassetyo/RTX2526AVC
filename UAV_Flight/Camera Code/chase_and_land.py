"""
Chase & Land – UAV takes off, hovers until ArUco marker ID 5 is spotted on
a moving UGV, commands the UGV to move forward via the V2V radio bridge,
then chases and precision-lands on the marker while it is in motion.

Combines:
  - GPS takeoff + hover (pymavlink GUIDED)
  - ArUco detection & pose estimation (CameraInterface / ArucoDistanceEstimator)
  - Precision-loiter → velocity-descent landing (from precision_land.py)
  - V2V serial bridge commanding (from v2v_bridge.py)
"""

import time
import math
import os
import cv2
import cv2.aruco as aruco
import numpy as np
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from pymavlink import mavutil

# Import the V2V bridge for UGV communication
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from v2v_bridge import V2VBridge, CMD_MOVE_FORWARD

# ─── CONFIGURATION ──────────────────────────────────────────────────────────────
MAVLINK_CONN        = "/dev/ttyACM0"   # Pixhawk serial or udp:127.0.0.1:14550
BAUD_RATE           = 921600
V2V_PORT            = "/dev/ttyUSB0"   # ESP32 radio serial port
V2V_BAUD            = 115200

# Set to False for actual outdoor flight
DESK_TESTING_NO_PROPELLERS = True

# ArUco marker on the UGV
TARGET_MARKER_ID    = 5
TARGET_MARKER_SIZE  = 0.3048           # 12 inches in meters

# Flight parameters
TARGET_HEIGHT_M            = 2.0       # Takeoff / hover altitude
TAKEOFF_ALT_TOLERANCE_M    = 0.25
TAKEOFF_MIN_CLIMB_M        = 0.15
TAKEOFF_TIMEOUT_S          = 20.0

# Tracking / landing thresholds
SEND_HZ                    = 15.0
PLND_STABLE_FRAMES         = 15        # Frames of solid tracking → precision loiter
LAND_LATERAL_ERR_M         = 0.20      # Max lateral error before starting descent
CHASE_KP                   = 0.8       # Proportional gain for velocity tracking
MAX_CHASE_SPEED            = 0.8       # m/s clamp

# ─── CAMERA & ARUCO (from precision_land.py) ────────────────────────────────────
@dataclass
class MarkerPose:
    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    center_px: Tuple[int, int]
    corners: np.ndarray


class CameraInterface:
    def __init__(self, use_zed: bool = False, camera_index: int = 0,
                 width: int = 1280, height: int = 720, fps: int = 30):
        self.use_zed = use_zed
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.zed = None
        self.sl = None
        self.camera_matrix = None
        self.dist_coeffs = None
        if self.use_zed:
            self._open_zed()
        else:
            self._open_standard()

    def _open_zed(self):
        import pyzed.sl as sl
        self.sl = sl
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = self.fps
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {err}")
        cam_info = self.zed.get_camera_information()
        calib = cam_info.camera_configuration.calibration_parameters.left_cam
        self.camera_matrix = np.array(
            [[calib.fx, 0.0, calib.cx],
             [0.0, calib.fy, calib.cy],
             [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array(calib.disto, dtype=np.float64).flatten()
        if dist.size >= 5:
            self.dist_coeffs = dist[:5].reshape(-1, 1)
        elif dist.size > 0:
            self.dist_coeffs = dist.reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def _open_standard(self):
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {self.camera_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def load_standard_calibration(self, yaml_path: Optional[str]):
        if yaml_path:
            fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
            K = fs.getNode("K").mat()
            D = fs.getNode("D").mat()
            fs.release()
            if K is not None and D is not None:
                self.camera_matrix = np.array(K, dtype=np.float64)
                self.dist_coeffs = np.array(D, dtype=np.float64)

    def get_frame(self) -> Optional[np.ndarray]:
        if self.use_zed:
            if self.zed.grab() != self.sl.ERROR_CODE.SUCCESS:
                return None
            image = self.sl.Mat()
            self.zed.retrieve_image(image, self.sl.VIEW.LEFT)
            frame = image.get_data()
            if frame.shape[-1] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            else:
                frame = frame.copy()
            return frame
        else:
            ret, frame = self.cap.read()
            return frame if ret else None

    def close(self):
        if self.use_zed and self.zed is not None:
            self.zed.close()
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


class ArucoDistanceEstimator:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
                 dictionary_name: int = aruco.DICT_6X6_1000):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary_name)
        if hasattr(aruco, "ArucoDetector"):
            self.detector_params = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            self.use_new_api = True
        else:
            self.detector_params = aruco.DetectorParameters_create()
            self.detector = None
            self.use_new_api = False

    def detect_markers(self, frame: np.ndarray, marker_size_m: float) -> Dict[int, MarkerPose]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict,
                                                  parameters=self.detector_params)
        poses: Dict[int, MarkerPose] = {}
        if ids is None or len(ids) == 0:
            return poses
        for i, mid in enumerate(ids.flatten()):
            rvec, tvec = self._estimate_pose(corners[i], marker_size_m)
            if rvec is None:
                continue
            center = np.mean(corners[i][0], axis=0).astype(int)
            poses[int(mid)] = MarkerPose(
                marker_id=int(mid), rvec=rvec, tvec=tvec.reshape(3),
                center_px=(int(center[0]), int(center[1])),
                corners=corners[i][0])
        return poses

    def _estimate_pose(self, corner, marker_size_m):
        half = marker_size_m / 2.0
        obj_pts = np.array([[-half, half, 0], [half, half, 0],
                            [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
        img_pts = corner.reshape((4, 2)).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.camera_matrix,
                                       self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        return (rvec, tvec) if ok else (None, None)


def draw_crosshair(frame: np.ndarray):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)
    cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)


# ─── MAVLINK HELPERS ─────────────────────────────────────────────────────────────
def connect_mavlink():
    conn = mavutil.mavlink_connection(MAVLINK_CONN, baud=BAUD_RATE)
    print("Waiting for heartbeat from UAV...")
    conn.wait_heartbeat()
    print("Heartbeat received!")
    return conn


def send_landing_target(conn, x_b, y_b, z_b):
    conn.mav.landing_target_send(
        int(time.time() * 1e6), 0, mavutil.mavlink.MAV_FRAME_BODY_FRD,
        0.0, 0.0, abs(z_b), 0.0, 0.0,
        x_b, y_b, abs(z_b), (1.0, 0.0, 0.0, 0.0), 0, 1)


def send_guided_velocity(conn, vx, vy, vz):
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)


def send_rc_override(conn, roll=1500, pitch=1500, throttle=1500, yaw=1500):
    rc = [65535] * 18
    rc[0] = int(roll)
    rc[1] = int(pitch)
    rc[2] = int(throttle)
    rc[3] = int(yaw)
    conn.mav.rc_channels_override_send(conn.target_system, conn.target_component, *rc)


def release_rc_override(conn):
    send_rc_override(conn, 0, 0, 0, 0)


def change_mode(conn, mode_name: str):
    mode_id = conn.mode_mapping().get(mode_name)
    if mode_id is None:
        return False
    conn.set_mode(mode_id)
    return True


def wait_for_mode(conn, mode_name: str, timeout: float = 8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if conn.flightmode == mode_name:
            return True
    return False


def get_relative_alt_m(conn):
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg is not None and hasattr(msg, 'relative_alt'):
        return float(msg.relative_alt) / 1000.0
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg is not None and hasattr(msg, 'z'):
        return float(-msg.z)
    msg = conn.recv_match(type='VFR_HUD', blocking=False)
    if msg is not None and hasattr(msg, 'alt'):
        return float(msg.alt)
    return None


def get_lidar_distance_m(conn):
    msg = conn.recv_match(type='DISTANCE_SENSOR', blocking=False)
    if msg:
        return float(msg.current_distance) / 100.0
    return None


def arm_and_takeoff(conn, alt):
    print("Switching to GUIDED...")
    if not change_mode(conn, "GUIDED"):
        raise RuntimeError("GUIDED mode not available")
    if not wait_for_mode(conn, "GUIDED", timeout=8.0):
        raise RuntimeError(f"Vehicle never entered GUIDED (current: {conn.flightmode})")

    print("Arming motors...")
    conn.arducopter_arm()
    conn.motors_armed_wait()
    print("Armed! Sending takeoff command...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, alt)


# ─── V2V BRIDGE HELPER ──────────────────────────────────────────────────────────
def connect_v2v():
    bridge = V2VBridge(V2V_PORT, V2V_BAUD, name="UAV-Bridge")
    bridge.connect()
    return bridge


# ─── MAIN ────────────────────────────────────────────────────────────────────────
def main():
    # --- Camera setup ---
    USE_ZED = True
    yaml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "calibration_chessboard.yaml")
    try:
        cam = CameraInterface(use_zed=USE_ZED, camera_index=0, fps=30)
        cam.load_standard_calibration(yaml_path)
    except Exception as e:
        print(f"Camera init failed: {e}")
        return

    estimator = ArucoDistanceEstimator(cam.camera_matrix, cam.dist_coeffs,
                                       aruco.DICT_6X6_1000)

    # --- MAVLink setup ---
    conn = connect_mavlink()

    # --- V2V bridge setup ---
    bridge = connect_v2v()

    # --- State machine ---
    #   TAKEOFF  → climb to TARGET_HEIGHT_M
    #   HOVER    → hold position, scan for marker ID 5
    #   CHASE    → fly toward marker, follow it as it moves
    #   PREC_LOITER → stable tracking, aligning for descent
    #   LANDING  → velocity descent onto the moving UGV
    #   LANDED   → done

    state = "HOVER" if DESK_TESTING_NO_PROPELLERS else "TAKEOFF"
    stable_count = 0
    last_send = 0.0
    takeoff_started = DESK_TESTING_NO_PROPELLERS
    takeoff_start_time = None
    latest_alt_m = None
    ugv_move_sent = False       # True once we have commanded the UGV forward
    cmd_seq = 1                 # V2V command sequence counter

    print("=================================================================")
    if DESK_TESTING_NO_PROPELLERS:
        print(">>> DESK TESTING MODE – no propellers, RC spoofing active")
    else:
        print(">>> REAL FLIGHT MODE – arming and taking off")
        arm_and_takeoff(conn, TARGET_HEIGHT_M)
        takeoff_start_time = time.time()
    print("=================================================================")

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                continue

            draw_crosshair(frame)

            # ── TAKEOFF state (real flight only) ───────────────────────────
            if state == "TAKEOFF":
                latest_alt_m = get_relative_alt_m(conn)
                if latest_alt_m is not None:
                    cv2.putText(frame,
                                f"TAKEOFF ALT: {latest_alt_m:.2f} / {TARGET_HEIGHT_M:.2f} m",
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    if latest_alt_m >= TAKEOFF_MIN_CLIMB_M:
                        takeoff_started = True
                else:
                    cv2.putText(frame, "TAKEOFF ALT: waiting for telemetry...",
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                if (takeoff_start_time and
                        (time.time() - takeoff_start_time) > TAKEOFF_TIMEOUT_S):
                    if not takeoff_started:
                        raise RuntimeError("Vehicle never started climbing")
                    print(">>> Takeoff timeout – switching to HOVER")
                    state = "HOVER"
                elif (latest_alt_m is not None and
                      latest_alt_m >= TARGET_HEIGHT_M - TAKEOFF_ALT_TOLERANCE_M):
                    print(f">>> Takeoff altitude reached ({latest_alt_m:.2f} m) – HOVER")
                    state = "HOVER"

                cv2.putText(frame, f"STATE: {state}", (20, 130),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                cv2.imshow("Chase & Land", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            # ── Detect ArUco markers ───────────────────────────────────────
            poses = estimator.detect_markers(frame, TARGET_MARKER_SIZE)

            found = False
            target_eb = None

            if TARGET_MARKER_ID in poses:
                pose = poses[TARGET_MARKER_ID]
                # Camera XYZ → ArduPilot body-FRD
                x_cam = float(pose.tvec[0])
                y_cam = float(pose.tvec[1])
                z_cam = float(pose.tvec[2])
                x_b = -y_cam   # forward
                y_b =  x_cam   # right
                z_b =  z_cam   # down
                target_eb = (x_b, y_b, z_b)
                found = True
                stable_count += 1

                # ── First detection: command UGV to move forward ──────────
                if not ugv_move_sent:
                    print(">>> Marker ID 5 detected! Commanding UGV to MOVE FORWARD")
                    bridge.send_command(cmd_seq, CMD_MOVE_FORWARD, 0)
                    cmd_seq += 1
                    ugv_move_sent = True

                # ── Visualisation ─────────────────────────────────────────
                cam_cx, cam_cy = frame.shape[1] // 2, frame.shape[0] // 2
                mx, my = pose.center_px
                cv2.line(frame, (cam_cx, cam_cy), (mx, my), (0, 0, 255), 4)
                corners_arr = [pose.corners.reshape(1, 4, 2).astype(np.float32)]
                aruco.drawDetectedMarkers(frame, corners_arr,
                                          np.array([[TARGET_MARKER_ID]]))
                cv2.drawFrameAxes(frame, cam.camera_matrix, cam.dist_coeffs,
                                  pose.rvec, pose.tvec.reshape(3, 1),
                                  TARGET_MARKER_SIZE * 0.5)
                cv2.putText(frame, f"TRACKING ID {TARGET_MARKER_ID}",
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame,
                            f"FRD: [{x_b:.2f}, {y_b:.2f}, {z_b:.2f}]",
                            (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # ── Send velocity / RC commands at SEND_HZ ───────────────
                now = time.time()
                if now - last_send >= (1.0 / SEND_HZ):
                    if DESK_TESTING_NO_PROPELLERS:
                        req_roll  = max(1300, min(1700, 1500 + int(y_b * 200.0)))
                        req_pitch = max(1300, min(1700, 1500 - int(x_b * 200.0)))
                        send_rc_override(conn, roll=req_roll, pitch=req_pitch,
                                         throttle=1550)
                    else:
                        if state in ("CHASE", "PREC_LOITER", "LANDING"):
                            vx = max(-MAX_CHASE_SPEED,
                                     min(MAX_CHASE_SPEED, x_b * CHASE_KP))
                            vy = max(-MAX_CHASE_SPEED,
                                     min(MAX_CHASE_SPEED, y_b * CHASE_KP))
                            vz = 0.0

                            if state == "LANDING":
                                lidar_m = get_lidar_distance_m(conn)
                                if lidar_m is not None:
                                    vz = max(0.15, min(0.4, lidar_m * 0.3))
                                    cv2.putText(frame,
                                                f"LIDAR Z: {lidar_m:.2f}m",
                                                (20, 120),
                                                cv2.FONT_HERSHEY_SIMPLEX,
                                                0.7, (0, 255, 0), 2)
                                else:
                                    vz = 0.4
                                    cv2.putText(frame,
                                                "LIDAR N/A - BLIND DESCENT",
                                                (20, 120),
                                                cv2.FONT_HERSHEY_SIMPLEX,
                                                0.7, (0, 0, 255), 2)

                            send_guided_velocity(conn, vx, vy, vz)
                            send_landing_target(conn, x_b, y_b, z_b)
                    last_send = now

            else:
                # No marker visible
                stable_count = 0
                now = time.time()
                if now - last_send >= (1.0 / SEND_HZ):
                    if DESK_TESTING_NO_PROPELLERS:
                        cv2.putText(frame, "TARGET LOST - STICKS CENTERED",
                                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 0, 255), 2)
                        send_rc_override(conn, 1500, 1500, 1500, 1500)
                    else:
                        cv2.putText(frame, "TARGET LOST - BRAKING TO HOVER",
                                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 0, 255), 2)
                        send_guided_velocity(conn, 0.0, 0.0, 0.0)
                    last_send = now

            # ── STATE TRANSITIONS ─────────────────────────────────────────
            if state == "HOVER":
                if found:
                    print(">>> Marker acquired – switching to CHASE")
                    change_mode(conn, "GUIDED")
                    state = "CHASE"

            elif state == "CHASE":
                if stable_count == 0:
                    # Lost the marker – go back to hover
                    state = "HOVER"
                elif stable_count >= PLND_STABLE_FRAMES:
                    print(">>> Stable lock – engaging PRECISION LOITER")
                    state = "PREC_LOITER"

            elif state == "PREC_LOITER":
                if stable_count == 0:
                    state = "CHASE"
                elif found and target_eb is not None:
                    err_m = math.sqrt(target_eb[0] ** 2 + target_eb[1] ** 2)
                    cv2.putText(frame, f"Align Err: {err_m:.2f}m", (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    if err_m < LAND_LATERAL_ERR_M:
                        print(">>> Aligned – beginning VELOCITY DESCENT")
                        state = "LANDING"

            elif state == "LANDING":
                if not conn.motors_armed():
                    print(">>> TOUCHDOWN CONFIRMED – motors disarmed")
                    state = "LANDED"
                    break

            cv2.putText(frame, f"STATE: {state}", (20, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            if ugv_move_sent:
                cv2.putText(frame, "UGV: MOVE FWD SENT", (20, 160),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
            cv2.imshow("Chase & Land", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        print("Cleaning up...")
        if DESK_TESTING_NO_PROPELLERS:
            release_rc_override(conn)
        else:
            send_guided_velocity(conn, 0.0, 0.0, 0.0)
        cam.close()
        bridge.stop()
        print("Done.")


if __name__ == "__main__":
    main()

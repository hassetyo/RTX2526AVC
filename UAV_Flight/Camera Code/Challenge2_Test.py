"""
Unified Autonomous Mission
==========================
1. Takeoff & Climb (GUIDED mode, MAV_CMD_NAV_TAKEOFF)
2. UGV Navigation (UAV hovers while commanding UGV to destination via v2v_bridge)
3. Precision Loiter & Landing (UAV visually aligns and lands on UGV marker)

Hardware: ArduPilot UAV, ESP32 V2V Bridge, ZED 2i Camera.
No RC Overrides. Pure GUIDED velocity commands.
"""

import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from pymavlink import mavutil
import v2v_bridge

# ─── CONFIGURATION ────────────────────────────────────────────────────────────
# Connections
MAVLINK_CONN      = "/dev/ttyACM0"
BAUD_RATE         = 921600
BRIDGE_PORT       = "/dev/ttyUSB0"
BRIDGE_BAUD       = 115200

# Flight Params
TARGET_HEIGHT_M         = 2.0
TAKEOFF_ALT_TOLERANCE_M = 0.25
TAKEOFF_MIN_CLIMB_M     = 0.15
TAKEOFF_TIMEOUT_S       = 20.0
PLND_STABLE_FRAMES      = 15
LAND_LATERAL_ERR_M      = 0.20
SEND_HZ                 = 15.0

# ArUco Configuration
UGV_MARKER_ID     = 5
DEST_MARKER_ID    = 0
MARKER_SIZE_M     = 0.254  # Update to match your actual marker size

# UGV Navigation Tuning
TURN_THRESHOLD_DEG = 12.0
STOP_DISTANCE_M    = 0.35
STEP_MIN_M         = 0.20
STEP_MAX_M         = 0.60
DRIVE_SPEED_MPS    = 1.5

# ─── DATA STRUCTURES & CAMERA ─────────────────────────────────────────────────
@dataclass
class MarkerPose:
    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    center_px: Tuple[int, int]
    corners: np.ndarray

class CameraInterface:
    def __init__(self, fps: int = 30):
        import pyzed.sl as sl
        self.sl = sl
        self.zed = sl.Camera()
        
        # Initialization tailored for ZED 2i performance 
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = fps
        init_params.depth_mode = sl.DEPTH_MODE.NONE # Depth disabled for pure ArUco speed
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED 2i camera: {err}")
            
        cam_info = self.zed.get_camera_information()
        calib = cam_info.camera_configuration.calibration_parameters.left_cam
        self.camera_matrix = np.array(
            [[calib.fx, 0.0, calib.cx],
             [0.0, calib.fy, calib.cy],
             [0.0, 0.0, 1.0]], dtype=np.float64)
             
        dist = np.array(calib.disto, dtype=np.float64).flatten()
        self.dist_coeffs = dist[:5].reshape(-1, 1) if dist.size >= 5 else np.zeros((5, 1), dtype=np.float64)

    def get_frame(self) -> Optional[np.ndarray]:
        if self.zed.grab() != self.sl.ERROR_CODE.SUCCESS:
            return None
        image = self.sl.Mat()
        self.zed.retrieve_image(image, self.sl.VIEW.LEFT)
        frame = image.get_data()
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR) if frame.shape[-1] == 4 else frame.copy()

    def close(self):
        if self.zed is not None:
            self.zed.close()

# ─── ARUCO ESTIMATOR ──────────────────────────────────────────────────────────
class ArucoDistanceEstimator:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, dictionary_name: int = aruco.DICT_6X6_1000):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary_name)
        
        if hasattr(aruco, "ArucoDetector"):
            self.detector_params = aruco.DetectorParameters()
            self.detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            self.use_new_api = True
        else:
            self.detector_params = aruco.DetectorParameters_create()
            self.detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            self.detector = None
            self.use_new_api = False

    def detect_markers(self, frame: np.ndarray, marker_size_m: float) -> Dict[int, MarkerPose]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
            
        poses: Dict[int, MarkerPose] = {}
        if ids is None or len(ids) == 0:
            return poses
            
        half = marker_size_m / 2.0
        object_points = np.array([[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]], dtype=np.float32)

        for i, marker_id in enumerate(ids.flatten()):
            image_points = corners[i][0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
            if success:
                center = np.mean(corners[i][0], axis=0).astype(int)
                poses[int(marker_id)] = MarkerPose(int(marker_id), rvec, tvec.reshape(3), (int(center[0]), int(center[1])), corners[i][0])
        return poses

# ─── UGV COMMANDER ────────────────────────────────────────────────────────────
class UGVCommander:
    def __init__(self, bridge_port, bridge_baud):
        self.bridge = v2v_bridge.V2VBridge(bridge_port, baud=bridge_baud, name="UGV-Bridge")
        self.seq = 1
        self.next_drive_time = 0.0

    def connect(self):
        self.bridge.connect()

    def _next_seq(self):
        val = self.seq; self.seq += 1; return val

    def send_turn_left(self):
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_TURN_LEFT, 0)

    def send_turn_right(self):
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_TURN_RIGHT, 0)

    def send_stop(self):
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_STOP, 0)

    def send_forward_step(self, step_m: float):
        now = time.time()
        if now < self.next_drive_time: return
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_MOVE_2FT, 0)
        self.next_drive_time = now + max(step_m / max(DRIVE_SPEED_MPS, 1e-6), 0.15) + 0.25

    def close(self):
        try: self.send_stop()
        except: pass
        self.bridge.stop()

# ─── MATH & HELPERS ───────────────────────────────────────────────────────────
def signed_angle_deg(vec_a: np.ndarray, vec_b: np.ndarray) -> float:
    a, b = np.asarray(vec_a, dtype=np.float64).reshape(2), np.asarray(vec_b, dtype=np.float64).reshape(2)
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9: return 0.0
    a, b = a / na, b / nb
    return math.degrees(math.atan2(float(a[0]*b[1] - a[1]*b[0]), float(np.clip(np.dot(a, b), -1.0, 1.0))))

def get_marker_fwd_dir(pose: MarkerPose, cam_matrix, dist_coeffs, marker_size) -> Optional[np.ndarray]:
    tip_local = np.array([0.0, 1.0, 0.0]) * (marker_size * 0.75) # Assuming +Y is forward
    pts3d = np.array([[0.0, 0.0, 0.0], tip_local], dtype=np.float32)
    img_pts, _ = cv2.projectPoints(pts3d, pose.rvec, pose.tvec.reshape(3, 1), cam_matrix, dist_coeffs)
    direction = img_pts.reshape(-1, 2)[1] - img_pts.reshape(-1, 2)[0]
    return direction if np.linalg.norm(direction) >= 1e-6 else None

# ─── MAVLINK CONTROL ──────────────────────────────────────────────────────────
def wait_for_mode(master, mode_name: str, timeout: float = 8.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if master.flightmode == mode_name: return True
    return False

def get_relative_alt_m(master) -> Optional[float]:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg: return float(msg.relative_alt) / 1000.0
    return None

def send_guided_velocity(master, vx: float, vy: float, vz: float):
    """
    Sends velocity commands in GUIDED mode using MAV_FRAME_BODY_NED.
    Bitmask 3527 (0x0DC7) ignores everything except velocities.
    """
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        3527,  # Mask allowing only Velocity
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0
    )

def send_landing_target(master, x_b, y_b, z_b):
    dist = abs(z_b)
    master.mav.landing_target_send(
        int(time.time() * 1e6), 0,
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        0.0, 0.0, dist, 0.0, 0.0, x_b, y_b, dist,
        (1.0, 0.0, 0.0, 0.0), 0, 1
    )

# ─── MAIN MISSION ENGINE ──────────────────────────────────────────────────────
def main():
    print("--- INITIATING SYSTEM ---")
    
    # 1. Connect UAV
    master = mavutil.mavlink_connection(MAVLINK_CONN, baud=BAUD_RATE)
    master.wait_heartbeat()
    print("UAV Heartbeat Received.")
    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # 2. Connect Camera & Systems
    cam = CameraInterface()
    estimator = ArucoDistanceEstimator(cam.camera_matrix, cam.dist_coeffs)
    ugv = UGVCommander(BRIDGE_PORT, BRIDGE_BAUD)
    ugv.connect()
    
    state = "TAKEOFF"
    last_known_alt = 0.0
    takeoff_start_time = None
    stable_count = 0
    last_mav_send = 0.0

    try:
        # ─── EXECUTE TAKEOFF SEQUENCE ───
        print("Switching to GUIDED...")
        master.set_mode(master.mode_mapping()["GUIDED"])
        if not wait_for_mode(master, "GUIDED"): raise RuntimeError("GUIDED failed.")
        
        print("Arming...")
        master.arducopter_arm()
        master.motors_armed_wait()
        
        print(f"Taking off to {TARGET_HEIGHT_M}m...")
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TARGET_HEIGHT_M)
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5.0)
        if not ack or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError("Takeoff Rejected")
            
        takeoff_start_time = time.time()

        # ─── MAIN CONTROL LOOP ───
        while True:
            frame = cam.get_frame()
            if frame is None: continue
            
            alt = get_relative_alt_m(master)
            if alt is not None: last_known_alt = alt

            poses = estimator.detect_markers(frame, MARKER_SIZE_M)
            now = time.time()
            
            # --- STATE: TAKEOFF WAITING ---
            if state == "TAKEOFF":
                if last_known_alt >= (TARGET_HEIGHT_M - TAKEOFF_ALT_TOLERANCE_M):
                    print(">>> Takeoff altitude reached. Transitioning to UGV NAV.")
                    state = "UGV_NAV"
                elif (now - takeoff_start_time) > TAKEOFF_TIMEOUT_S:
                    print(">>> Takeoff timeout! Proceeding anyway...")
                    state = "UGV_NAV"
                    
            # --- STATE: UGV NAVIGATION ---
            elif state == "UGV_NAV":
                # UAV Task: Hover in place strictly using velocity commands
                if now - last_mav_send > (1.0 / SEND_HZ):
                    send_guided_velocity(master, 0.0, 0.0, 0.0) 
                    last_mav_send = now

                # UGV Task: Visually navigate it to destination
                if UGV_MARKER_ID in poses and DEST_MARKER_ID in poses:
                    ugv_p, dest_p = poses[UGV_MARKER_ID], poses[DEST_MARKER_ID]
                    
                    fwd_dir = get_marker_fwd_dir(ugv_p, cam.camera_matrix, cam.dist_coeffs, MARKER_SIZE_M)
                    target_dir = np.array([dest_p.center_px[0] - ugv_p.center_px[0], dest_p.center_px[1] - ugv_p.center_px[1]], dtype=np.float64)
                    heading_err = signed_angle_deg(fwd_dir if fwd_dir is not None else target_dir, target_dir)
                    dist_m = float(np.linalg.norm(dest_p.tvec.reshape(3) - ugv_p.tvec.reshape(3)))

                    if dist_m <= STOP_DISTANCE_M:
                        ugv.send_stop()
                        print(">>> UGV arrived at destination. Commencing Precision Land Alignment.")
                        state = "ALIGN"
                    elif abs(heading_err) > TURN_THRESHOLD_DEG:
                        ugv.send_turn_right() if heading_err > 0 else ugv.send_turn_left()
                    else:
                        ugv.send_forward_step(max(STEP_MIN_M, min(dist_m * 0.5, STEP_MAX_M)))

            # --- STATES: ALIGN & LANDING ---
            elif state in ["ALIGN", "LANDING"]:
                if now - last_mav_send >= (1.0 / SEND_HZ):
                    if UGV_MARKER_ID in poses:
                        pose = poses[UGV_MARKER_ID]
                        
                        # Translate Camera coordinates to ArduPilot Body NED coordinates
                        x_b = -float(pose.tvec[1]) # Forward
                        y_b = float(pose.tvec[0])  # Right
                        z_b = float(pose.tvec[2])  # Down
                        
                        err_m = math.sqrt(x_b**2 + y_b**2)
                        
                        # Velocity P-controller
                        Kp = 0.8
                        vx = max(-0.8, min(0.8, x_b * Kp))
                        vy = max(-0.8, min(0.8, y_b * Kp))
                        vz = 0.0
                        
                        if state == "ALIGN":
                            if err_m < LAND_LATERAL_ERR_M:
                                stable_count += 1
                                if stable_count > PLND_STABLE_FRAMES:
                                    print(">>> Aligned over UGV. Initiating Descent.")
                                    state = "LANDING"
                            else:
                                stable_count = 0
                        
                        elif state == "LANDING":
                            # Combine alignment with slow descent 
                            vz = 0.35 
                            send_landing_target(master, x_b, y_b, z_b)
                            
                            # Check for touchdown (disarm)
                            if not master.motors_armed():
                                print(">>> TOUCHDOWN DETECTED. Mission Complete.")
                                state = "LANDED"
                                break

                        send_guided_velocity(master, vx, vy, vz)
                    else:
                        # Target lost: Brake to hover
                        send_guided_velocity(master, 0.0, 0.0, 0.0)
                        
                    last_mav_send = now

            # OpenCV visualization (optional but good for debugging)
            cv2.putText(frame, f"STATE: {state} | ALT: {last_known_alt:.2f}m", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Mission Commander", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("User aborted mission.")
    except Exception as e:
        print(f"Mission Failed: {e}")
    finally:
        print("Cleaning up system & braking to hover...")
        try:
            send_guided_velocity(master, 0.0, 0.0, 0.0)
        except: pass
        ugv.close()
        cam.close()

if __name__ == "__main__":
    main()

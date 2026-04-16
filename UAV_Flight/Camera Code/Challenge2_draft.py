"""
Challenge 1 - UAV Precision Landing on Moving UGV
=================================================
Automated takeoff + Precision Loiter + Precision Land using MAVLink LANDING_TARGET.
Uses the robust CameraInterface from the user's friend's code.
"""
import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np
import os
import argparse
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from pymavlink import mavutil
import v2v_bridge


# ─── USER CONFIGURATION ────────────────────────────────────────────────────────
MAVLINK_CONN = "/dev/ttyACM0"      # Or udp:127.0.0.1:14550
BAUD_RATE    = 921600

# 🔴 IMPORTANT: Set this to False when you go outside to actually fly!
# When True, it violently spoofs RC controllers to force motor sounds on your desk.
DESK_TESTING_NO_PROPELLERS = True

# Nested ArUco Marker Setup
LARGE_MARKER_ID   = 5
SMALL_MARKER_ID   = 6
LARGE_MARKER_SIZE = 0.3048   # 12 inches -> meters
SMALL_MARKER_SIZE = 0.1016   # 4 inches -> meters

USE_SMALL_BELOW_M = 1.0
SEND_HZ = 15.0

# Flight Logic
TARGET_HEIGHT_M = 2.3
TAKEOFF_ALT_TOLERANCE_M = 0.25   # Start vision movement once we are within this band of target alt
TAKEOFF_MIN_CLIMB_M = 0.15       # Consider the vehicle airborne once it has climbed at least this much
TAKEOFF_TIMEOUT_S = 20.0
PLND_STABLE_FRAMES = 15   # Frames of solid tracking before engaging Precision Loiter
LAND_LATERAL_ERR_M = 0.20 # Meters of allowed lateral error before switching to LAND

# Error Handling
FAIL_COUNT = 0
MAX_FAIL_COUNT = 5


# ─── FRIEND'S ROBUST CAMERA & ARUCO CODE ───────────────────────────────────────
@dataclass
class MarkerPose:
    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    center_px: Tuple[int, int]
    corners: np.ndarray

class CameraInterface:
    def __init__(self, use_zed: bool = False, camera_index: int = 0, width: int = 1280, height: int = 720, fps: int = 30):
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
        try:
            import pyzed.sl as sl
        except ImportError as e:
            raise RuntimeError("pyzed.sl is not installed. Install the ZED SDK Python API first.") from e
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
        self.camera_matrix = np.array([[calib.fx, 0.0, calib.cx], [0.0, calib.fy, calib.cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array(calib.disto, dtype=np.float64).flatten()
        if dist.size >= 5: self.dist_coeffs = dist[:5].reshape(-1, 1)
        elif dist.size > 0: self.dist_coeffs = dist.reshape(-1, 1)
        else: self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

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
                print(f"Loaded custom camera calibration from {yaml_path}")
            return

    def get_frame(self) -> Optional[np.ndarray]:
        if self.use_zed:
            if self.zed.grab() != self.sl.ERROR_CODE.SUCCESS: return None
            image = self.sl.Mat()
            self.zed.retrieve_image(image, self.sl.VIEW.LEFT)
            frame = image.get_data()
            if frame.shape[-1] == 4: frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            else: frame = frame.copy()
            return frame
        else:
            ret, frame = self.cap.read()
            if not ret: return None
            return frame

    def close(self):
        if self.use_zed and self.zed is not None: self.zed.close()
        if self.cap is not None: self.cap.release()
        cv2.destroyAllWindows()

class ArucoDistanceEstimator:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, dictionary_name: int = aruco.DICT_6X6_1000):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary_name)
        if hasattr(aruco, "ArucoDetector"):
            self.detector_params = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            self.use_new_detector_api = True
        else:
            self.detector_params = aruco.DetectorParameters_create()
            self.detector = None
            self.use_new_detector_api = False

    def detect_markers(self, frame: np.ndarray, marker_size_m: float) -> Dict[int, MarkerPose]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_new_detector_api: corners, ids, _ = self.detector.detectMarkers(gray)
        else: corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        poses: Dict[int, MarkerPose] = {}
        if ids is None or len(ids) == 0: return poses

        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec = self._estimate_pose(corners[i], marker_size_m)
            if rvec is None or tvec is None: continue
            center = np.mean(corners[i][0], axis=0).astype(int)
            poses[int(marker_id)] = MarkerPose(
                marker_id=int(marker_id), rvec=rvec, tvec=tvec.reshape(3),
                center_px=(int(center[0]), int(center[1])), corners=corners[i][0]
            )
        return poses

    def _estimate_pose(self, corner: np.ndarray, marker_size_m: float):
        half = marker_size_m / 2.0
        object_points = np.array([[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float32)
        image_points = corner.reshape((4, 2)).astype(np.float32)
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success: return None, None
        return rvec, tvec

def draw_crosshair(frame: np.ndarray):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)
    cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)

# ─── MAVLINK HELPERS ───────────────────────────────────────────────────────────
master = mavutil.mavlink_connection(MAVLINK_CONN, baud=BAUD_RATE)
print("Waiting for heartbeats from UAV...")
master.wait_heartbeat()
print("Heartbeat received!")

def send_landing_target(x_b, y_b, z_b):
    """ Secondary: Broadcasts absolute marker target relative to flow """
    master.mav.landing_target_send(
        int(time.time() * 1e6), 0, mavutil.mavlink.MAV_FRAME_BODY_FRD, 0.0, 0.0,
        abs(z_b), 0.0, 0.0, x_b, y_b, abs(z_b), (1.0, 0.0, 0.0, 0.0), 0, 1
    )

def send_guided_velocity(vx, vy, vz):
    """ Primary Flight: Smooth, safe, velocity vectors directly forcing the PIDs! """
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0
    )

def send_rc_override(roll=1500, pitch=1500, throttle=1500, yaw=1500):
    """ Forcefully bypass ArduPilot's 'Not Flying' safety state for desk testing """
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[0] = int(roll)
    rc_channel_values[1] = int(pitch)
    rc_channel_values[2] = int(throttle)
    rc_channel_values[3] = int(yaw)
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_channel_values)

def release_rc_override():
    send_rc_override(0, 0, 0, 0)

def change_mode(mode_name: str):
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        return False
    master.set_mode(mode_id)
    return True


def wait_for_mode(mode_name: str, timeout: float = 8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if master.flightmode == mode_name:
            return True
    return False


def get_relative_alt_m():
    """Best-effort relative altitude using autopilot telemetry."""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if msg is not None and hasattr(msg, 'relative_alt'):
        return float(msg.relative_alt) / 1000.0  # mm -> m

    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg is not None and hasattr(msg, 'z'):
        return float(-msg.z)

    msg = master.recv_match(type='VFR_HUD', blocking=False)
    if msg is not None and hasattr(msg, 'alt'):
        return float(msg.alt)

    return None


def get_lidar_distance_m():
    """ Actively pulls hardware pings from the Lidar Lite V3 """
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=False)
    if msg:
        return float(msg.current_distance) / 100.0  # cm to meters
    return None

def arm_with_timeout(master, timeout=10):
    master.arducopter_arm()
    start_time = time.time()
    while time.time() - start_time < timeout:
        if master.motors_armed():
            print("Motors Armed!")
            return True
        time.sleep(0.1)
    print("Failed to arm within timeout.")
    return False

def arm_and_takeoff(alt):
    print("Switching to GUIDED...")
    if not change_mode("GUIDED"):
        raise RuntimeError("GUIDED mode is not available on this vehicle")
    if not wait_for_mode("GUIDED", timeout=8.0):
        raise RuntimeError(f"Vehicle never entered GUIDED mode (current: {master.flightmode})")

    print("Arming motors...")
    isArmed = arm_with_timeout(master, timeout=10)
    if not isArmed:
        raise RuntimeError("Failed to arm motors, cannot takeoff")
    
    print("Armed! Sending takeoff command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, alt
    )


'''Controls the UGV navigation'''
class UGVCommander:
    def __init__(
        self,
        bridge_port: str,
        bridge_baud: int,
        turn_threshold_deg: float,
        stop_distance_m: float,
        step_min_m: float,
        step_max_m: float,
        drive_speed_mps: float,
        marker_timeout_sec: float,
    ):
        self.bridge = v2v_bridge.V2VBridge(bridge_port, baud=bridge_baud, name="UGV-Nav-Bridge")
        self.turn_threshold_deg = turn_threshold_deg
        self.stop_distance_m = stop_distance_m
        self.step_min_m = step_min_m
        self.step_max_m = step_max_m
        self.drive_speed_mps = drive_speed_mps
        self.marker_timeout_sec = marker_timeout_sec

        self.seq = 1
        self.arm_sent = False
        self.last_motion = "idle"
        self.next_drive_time = 0.0
        self.last_seen_time = 0.0
        self.last_status_text = "init"

    ''' Connect to the ESP32 bridge and send an initial message indicating that the UGV navigator is online.'''
    def connect(self):
        self.bridge.connect()
        self.bridge.send_message("aruco ugv navigator online")

    ''' Close the connection to the ESP32 bridge and send a stop command to ensure the UGV is not left moving.'''
    def close(self):
        try:
            self.send_stop(force=True)
        except Exception:
            pass
        self.bridge.stop()

    ''' Generate the next sequence number for commands sent to the bridge. Increments an internal counter and returns the current value.'''
    def _next_seq(self) -> int:
        val = self.seq
        self.seq += 1
        return val

    ''' Makes sure the UGV is armed before sending movement commands.'''
    def ensure_armed(self):
        if self.arm_sent:
            return
        print("[UGV] Sending ARM command over bridge...")
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_ARM, 0)
        self.arm_sent = True
        self.last_status_text = "arming sent"
        time.sleep(0.25)

    '''Tells the UGV to turn left if it is not already turning left.'''
    def send_turn_left(self):
        if self.last_motion == "turn_left":
            return
        print("[UGV] TURN LEFT")
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_TURN_LEFT, 0)
        self.last_motion = "turn_left"
        self.last_status_text = "turning left"

    '''Tells the UGV to turn right if it is not already turning right.'''
    def send_turn_right(self):
        if self.last_motion == "turn_right":
            return
        print("[UGV] TURN RIGHT")
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_TURN_RIGHT, 0)
        self.last_motion = "turn_right"
        self.last_status_text = "turning right"

    '''Tells the UGV to stop if its not already stopped or if the stop if forced.'''
    def send_stop(self, force: bool = False):
        if not force and self.last_motion == "stopped":
            return
        print("[UGV] STOP")
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_STOP, 0)
        self.last_motion = "stopped"
        self.last_status_text = "stopped"

    '''Tells the UGV to move forward a certain step in meters.'''
    def send_forward_step(self, step_m: float):
        now = time.time()
        if now < self.next_drive_time:
            return

        step_m = max(self.step_min_m, min(step_m, self.step_max_m))
        duration = max(step_m / max(self.drive_speed_mps, 1e-6), 0.15)
        margin = 0.25

        print(f"[UGV] FORWARD STEP {step_m:.3f} m")
        self.bridge.send_message(f"GOTO:{step_m:.3f},0")
        self.last_motion = "forward"
        self.last_status_text = f"forward {step_m:.2f} m"
        self.next_drive_time = now + duration + margin

    ''' If the markers are lost for longer than the specified timeout, send a stop command to the UGV and update the status text.'''
    def handle_marker_loss(self):
        now = time.time()
        if self.last_seen_time <= 0.0:
            return
        if (now - self.last_seen_time) > self.marker_timeout_sec:
            self.send_stop()
            self.last_status_text = "marker lost -> stopped"


FORWARD_AXIS_MAP = {
    "+x": np.array([1.0, 0.0, 0.0], dtype=np.float64),
    "-x": np.array([-1.0, 0.0, 0.0], dtype=np.float64),
    "+y": np.array([0.0, 1.0, 0.0], dtype=np.float64),
    "-y": np.array([0.0, -1.0, 0.0], dtype=np.float64),
}

''' Calculate the forward direction of a marker in pixel coordinates. 
Returns a 2D vector pointing in the forward direction of the marker or None if the direction cannot be determined.'''
def get_marker_forward_direction_px(
    pose: MarkerPose,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_size_m: float,
    forward_axis_name: str,
) -> Optional[np.ndarray]:
    axis = FORWARD_AXIS_MAP[forward_axis_name]
    tip_local = axis * (marker_size_m * 0.75)

    pts3d = np.array(
        [
            [0.0, 0.0, 0.0],
            tip_local,
        ],
        dtype=np.float32,
    )

    img_pts, _ = cv2.projectPoints(
        pts3d,
        pose.rvec,
        pose.tvec.reshape(3, 1),
        camera_matrix,
        dist_coeffs,
    )
    img_pts = img_pts.reshape(-1, 2)

    center = img_pts[0]
    tip = img_pts[1]
    direction = tip - center

    if np.linalg.norm(direction) < 1e-6:
        return None
    return direction

''' Calculate the signed angle in degrees between two 2D vectors. The angle is positive if vec_b is counterclockwise from vec_a, and negative if clockwise.'''
def signed_angle_deg(vec_a: np.ndarray, vec_b: np.ndarray) -> float:
    a = np.asarray(vec_a, dtype=np.float64).reshape(2)
    b = np.asarray(vec_b, dtype=np.float64).reshape(2)

    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return 0.0

    a /= na
    b /= nb

    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    cross = float(a[0] * b[1] - a[1] * b[0])
    return math.degrees(math.atan2(cross, dot))

''' Pick two markers from the detected poses. If specific marker IDs are provided, try to find those. 
    Otherwise, pick the two markers with the lowest IDs. 
    Returns a tuple of the two MarkerPose objects or None if not enough markers are detected.'''
def pick_two_markers(
    poses: Dict[int, MarkerPose],
    marker1_id: Optional[int],
    marker2_id: Optional[int],
) -> Optional[Tuple[MarkerPose, MarkerPose]]:
    if len(poses) < 2:
        return None

    if marker1_id is not None and marker2_id is not None:
        if marker1_id in poses and marker2_id in poses:
            return poses[marker1_id], poses[marker2_id]
        return None

    ordered_ids = sorted(poses.keys())
    return poses[ordered_ids[0]], poses[ordered_ids[1]]


''' Draw a navigation overlay on the frame showing the UGV and destination markers, the heading error, distance, and status text.'''
def draw_nav_overlay(
    frame: np.ndarray,
    ugv_pose: MarkerPose,
    dest_pose: MarkerPose,
    heading_error_deg: float,
    status_text: str,
    stop_distance_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_size_m: float,
    forward_axis_name: str,
):
    ugv_center = np.array(ugv_pose.center_px, dtype=np.int32)
    dst_center = np.array(dest_pose.center_px, dtype=np.int32)

    fwd_dir = get_marker_forward_direction_px(
        ugv_pose,
        camera_matrix,
        dist_coeffs,
        marker_size_m,
        forward_axis_name,
    )

    cv2.arrowedLine(
        frame,
        tuple(ugv_center),
        tuple(dst_center),
        (0, 255, 255),
        2,
        tipLength=0.08,
    )

    if fwd_dir is not None:
        fwd_tip = ugv_center + np.round(fwd_dir).astype(np.int32)
        cv2.arrowedLine(
            frame,
            tuple(ugv_center),
            tuple(fwd_tip),
            (255, 0, 255),
            2,
            tipLength=0.18,
        )

    diff = dest_pose.tvec.reshape(3) - ugv_pose.tvec.reshape(3)
    dist_m = float(np.linalg.norm(diff))

    lines = [
        f"UGV ID: {ugv_pose.marker_id}",
        f"DEST ID: {dest_pose.marker_id}",
        f"Heading error: {heading_error_deg:+.1f} deg",
        f"Distance: {dist_m:.3f} m",
        f"Stop dist: {stop_distance_m:.3f} m",
        f"State: {status_text}",
    ]

    x0, y0 = 20, 20
    padding = 10
    line_h = 28
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.7
    thickness = 2

    max_w = 0
    for text in lines:
        (tw, _), _ = cv2.getTextSize(text, font, scale, thickness)
        max_w = max(max_w, tw)

    box_w = max_w + 2 * padding
    box_h = len(lines) * line_h + 2 * padding

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (255, 255, 255), -1)
    frame[:] = cv2.addWeighted(overlay, 0.82, frame, 0.18, 0)

    y = y0 + padding + 20
    for text in lines:
        cv2.putText(
            frame,
            text,
            (x0 + padding, y),
            font,
            scale,
            (0, 0, 0),
            thickness,
            cv2.LINE_AA,
        )
        y += line_h

''' Parse command-line arguments for configuring the camera, ArUco detection, and UGV control parameters.'''
def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Measure ArUco marker distance and command the ground vehicle marker to drive toward the destination marker."
        )
    )
    parser.add_argument("--use-zed", action="store_true", help="Use Stereolabs ZED / ZED X camera.")
    parser.add_argument("--camera-index", type=int, default=0, help="Standard camera index for laptop/USB webcam.")
    parser.add_argument("--calibration", type=str, default=None, help="YAML calibration file for standard camera with nodes K and D.")
    parser.add_argument("--marker-size", type=float, default=0.254, help="Marker size in meters. Example: 0.254 for 10 inches.")
    parser.add_argument("--ugv-marker-id", type=int, default=5, help="ArUco ID attached to the ground vehicle.")
    parser.add_argument("--dest-marker-id", type=int, default=0, help="Destination ArUco ID.")
    parser.add_argument("--dict", type=str, default="DICT_6X6_1000", help="ArUco dictionary name.")
    parser.add_argument("--width", type=int, default=1280, help="Standard camera width.")
    parser.add_argument("--height", type=int, default=720, help="Standard camera height.")
    parser.add_argument("--fps", type=int, default=60, help="Camera FPS.")

    parser.add_argument("--bridge-port", type=str, default="/dev/ttyUSB0", help="Local ESP32 bridge serial port used to send commands.")
    parser.add_argument("--bridge-baud", type=int, default=115200, help="ESP32 bridge baud rate.")

    parser.add_argument("--turn-threshold-deg", type=float, default=12.0, help="Heading error magnitude required before issuing turn commands.")
    parser.add_argument("--stop-distance-m", type=float, default=0.28, help="Distance at which the UGV is considered to have reached the destination.")
    parser.add_argument("--step-min-m", type=float, default=0.20, help="Minimum forward body step to command.")
    parser.add_argument("--step-max-m", type=float, default=0.60, help="Maximum forward body step to command.")
    parser.add_argument("--drive-speed-mps", type=float, default=1.5, help="Expected ground-station forward speed used only for cooldown timing.")
    parser.add_argument("--marker-timeout-sec", type=float, default=0.75, help="Stop the UGV if markers disappear longer than this timeout.")
    parser.add_argument(
        "--ugv-forward-axis",
        choices=["+x", "-x", "+y", "-y"],
        default="+y",
        help="Which marker local axis points in the UGV forward direction.",
    )
    return parser.parse_args()

''' Get the ArUco dictionary constant from OpenCV by name. Raises a ValueError if the name is not valid.'''
def get_dictionary_by_name(name: str) -> int:
    if not hasattr(aruco, name):
        valid = [x for x in dir(aruco) if x.startswith("DICT_")]
        raise ValueError(f"Unknown dictionary '{name}'. Valid examples: {valid[:10]}")
    return getattr(aruco, name)

'''Initialize the camera, ArUco estimator, and UGV commander before entering the main functionality'''
def initialize_system(args):
    dictionary = get_dictionary_by_name(args.dict)

    cam = CameraInterface(
        use_zed=args.use_zed,
        camera_index=args.camera_index,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )

    if not args.use_zed:
        cam.load_standard_calibration(args.calibration)

    estimator = ArucoDistanceEstimator(
        camera_matrix=cam.camera_matrix,
        dist_coeffs=cam.dist_coeffs,
        marker_size_m=args.marker_size,
        dictionary_name=dictionary,
    )

    commander = UGVCommander(
        bridge_port=args.bridge_port,
        bridge_baud=args.bridge_baud,
        turn_threshold_deg=args.turn_threshold_deg,
        stop_distance_m=args.stop_distance_m,
        step_min_m=args.step_min_m,
        step_max_m=args.step_max_m,
        drive_speed_mps=args.drive_speed_mps,
        marker_timeout_sec=args.marker_timeout_sec,
    )
    commander.connect()

    return cam, estimator, commander

''' Draw a line between two markers and annotate the distance and relative position. 
    Also displays a status box with detailed information about the markers and their relative pose.'''
def draw_distance_overlay(
    frame: np.ndarray,
    pose_a: MarkerPose,
    pose_b: MarkerPose,
):
    p1 = pose_a.tvec.reshape(3)
    p2 = pose_b.tvec.reshape(3)

    diff = p2 - p1
    dist_m = np.linalg.norm(diff)

    x_diff_mm = diff[0] * 1000.0
    y_diff_mm = diff[1] * 1000.0
    z_diff_mm = diff[2] * 1000.0
    dist_mm = dist_m * 1000.0

    cv2.line(frame, pose_a.center_px, pose_b.center_px, (0, 255, 0), 3)

    mid_x = (pose_a.center_px[0] + pose_b.center_px[0]) // 2
    mid_y = (pose_a.center_px[1] + pose_b.center_px[1]) // 2

    cv2.putText(
        frame,
        f"{dist_mm:.1f} mm",
        (mid_x + 10, mid_y - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    lines = [
        f"Marker A ID: {pose_a.marker_id}",
        f"Marker B ID: {pose_b.marker_id}",
        f"Z Diff is : {z_diff_mm:.2f} [mm]",
        f"Y Diff is : {y_diff_mm:.2f} [mm]",
        f"X Diff is : {x_diff_mm:.2f} [mm]",
        f"Distance is : {dist_mm:.2f} [mm]",
    ]

    padding = 12
    line_h = 34
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.9
    thickness = 2

    max_w = 0
    for text in lines:
        (tw, _), _ = cv2.getTextSize(text, font, scale, thickness)
        max_w = max(max_w, tw)

    box_w = max_w + padding * 2
    box_h = line_h * len(lines) + padding * 2
    x0 = 20
    y0 = frame.shape[0] - box_h - 20

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (255, 255, 255), -1)
    frame[:] = cv2.addWeighted(overlay, 0.82, frame, 0.18, 0)

    y = y0 + padding + 24
    for text in lines:
        cv2.putText(
            frame,
            text,
            (x0 + padding, y),
            font,
            scale,
            (0, 0, 0),
            thickness,
            cv2.LINE_AA,
        )
        y += line_h


'''Main ArUco navigation function'''
def guideUGV(args, commander, estimator, cam):

    print(
        f"Tracking UGV marker {args.ugv_marker_id} toward destination marker {args.dest_marker_id}."
    )

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                print("Failed to read frame.")
                commander.handle_marker_loss()
                continue

            poses = estimator.detect_markers(frame)
            display = frame.copy()

            draw_crosshair(display)
            estimator.draw_markers(display, poses)

            pair = pick_two_markers(poses, args.ugv_marker_id, args.dest_marker_id)
            if pair is not None:
                ugv_pose, dest_pose = pair
                commander.last_seen_time = time.time()
                commander.ensure_armed()

                draw_distance_overlay(display, ugv_pose, dest_pose)

                forward_dir = get_marker_forward_direction_px(
                    ugv_pose,
                    cam.camera_matrix,
                    cam.dist_coeffs,
                    args.marker_size,
                    args.ugv_forward_axis,
                )
                target_dir = np.array(
                    [
                        dest_pose.center_px[0] - ugv_pose.center_px[0],
                        dest_pose.center_px[1] - ugv_pose.center_px[1],
                    ],
                    dtype=np.float64,
                )
                heading_error_deg = signed_angle_deg(forward_dir if forward_dir is not None else target_dir, target_dir)
                dist_m = float(np.linalg.norm(dest_pose.tvec.reshape(3) - ugv_pose.tvec.reshape(3)))

                if dist_m <= args.stop_distance_m:
                    commander.send_stop()
                    commander.last_status_text = "destination reached"
                elif abs(heading_error_deg) > args.turn_threshold_deg:
                    if heading_error_deg > 0.0:
                        commander.send_turn_right()
                    else:
                        commander.send_turn_left()
                else:
                    if commander.last_motion in ("turn_left", "turn_right"):
                        commander.send_stop()
                        time.sleep(0.15)

                    remaining = max(0.0, dist_m - args.stop_distance_m)
                    step_m = max(args.step_min_m, min(remaining * 0.5, args.step_max_m))
                    commander.send_forward_step(step_m)

                draw_nav_overlay(
                    display,
                    ugv_pose,
                    dest_pose,
                    heading_error_deg,
                    commander.last_status_text,
                    args.stop_distance_m,
                    cam.camera_matrix,
                    cam.dist_coeffs,
                    args.marker_size,
                    args.ugv_forward_axis,
                )
            else:
                commander.handle_marker_loss()
                cv2.putText(
                    display,
                    f"Need markers {args.ugv_marker_id} and {args.dest_marker_id}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    display,
                    f"State: {commander.last_status_text}",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow("ArUco Distance + UGV Navigation", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                cv2.imwrite("aruco_distance_ugv_nav_screenshot.png", display)
                print("Saved: aruco_distance_ugv_nav_screenshot.png")

    finally:
        commander.close()
        cam.close()

''' Tells the drone to move to the center of the grid and increase altitude until both markers are visible, then switch to the main navigation loop. This can help in cases where the markers are initially too close or partially out of frame.'''
def moveToCenter(master, UAVcommander, estimator, cam, args, bottomRight=True):
    global FAIL_COUNT

    print("Moving toward center while climbing and searching for markers...")

    try:
        FAIL_COUNT = 0

        # Distance from corner -> center of a 15x15 yard field
        distance_to_center_m = (15.0 / math.sqrt(2.0)) * 0.9144

        forward_speed_mps = 1.5
        total_time = distance_to_center_m / forward_speed_mps

        step_time = 0.5   # seconds per forward step
        steps = int(total_time / step_time)

        climb_step = 0.1  # meters per loop

        for step in range(steps):
            frame = cam.get_frame()

            if frame is None:
                print("Failed to read frame.")
                FAIL_COUNT += 1
                if FAIL_COUNT > 5:
                    print("Too many camera failures.")
                    return 0
                continue

            poses = estimator.detect_markers(frame)

            pair = pick_two_markers(poses, args.ugv_marker_id, args.dest_marker_id)

            # If both markers found -> hover immediately
            if pair is not None:
                ugv_pose, dest_pose = pair

                UAVcommander.log_event(f"Markers detected! ids = {list(poses.keys())}")
                print(f"Both markers found: UGV={ugv_pose.marker_id}, DEST={dest_pose.marker_id}")
                print("UAV entering hover mode now.")
                UAVcommander.log_event("Markers found -> UAV hovering")
                return 1

            # If not found, keep climbing slowly while moving toward center
            currentAlt = UAVcommander.print_altitude(master)

            if currentAlt < 5.0:
                if DESK_TESTING_NO_PROPELLERS:
                    print(f"TESTING MODE: would climb from {currentAlt:.2f} m to {currentAlt + climb_step:.2f} m")
                else:
                    UAVcommander.climb_to_target(master, target_alt=currentAlt + climb_step)

            UAVcommander.move_pitch(master, forward=True, seconds=step_time)

        print("Reached center but did not detect markers.")

        # After reaching center, keep climbing and searching
        while True:
            frame = cam.get_frame()

            if frame is None:
                print("Failed to read frame.")
                FAIL_COUNT += 1
                if FAIL_COUNT > 5:
                    print("Too many camera failures.")
                    return 0
                continue

            poses = estimator.detect_markers(frame)
            pair = pick_two_markers(poses, args.ugv_marker_id, args.dest_marker_id)

            if pair is not None:
                ugv_pose, dest_pose = pair
                print(f"Both markers found: UGV={ugv_pose.marker_id}, DEST={dest_pose.marker_id}")
                print("UAV entering hover mode now.")
                UAVcommander.log_event("Markers found after reaching center -> UAV hovering")
                return 1

            currentAlt = UAVcommander.print_altitude(master)

            if currentAlt >= 5.0:
                print("Max altitude reached, markers not found.")
                return 0

            if DESK_TESTING_NO_PROPELLERS:
                print(f"TESTING MODE: would climb from {currentAlt:.2f} m to {currentAlt + climb_step:.2f} m")
            else:
                UAVcommander.climb_to_target(master, target_alt=currentAlt + climb_step)

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("Keyboard interrupt received.")
        return -1

    finally:
        print("Finished moveToCenter sequence.")
        FAIL_COUNT = 0

def endCode(master, UAVcommander, UGVcommander, cam):
    print("Ending program and landing drone safely...")
    time.sleep(2.0)
    UAVcommander.land_safely(master)
    cam.close()
    UGVcommander.close()

def loiter_landing_sequence(master, UAVcommander, cam, args):

    '''
    notes: make the drone land using the code in main
    change the parameters if needed 
    if an error occurs force it go go into regular land mode
        no need to handle errors at first, 
        do it when we know the code is properly integrated
    - Sam
    '''
# ─── MAIN PROGRAM ──────────────────────────────────────────────────────────────
def main():
    # Attempt ZED first to prevent v4l2 OpenCV timeout crash
    USE_ZED = True  
    yaml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_chessboard.yaml")
    
    # Intentionally initialize camera flawlessly using Friend's script wrapper
    try:
        # Toggle USE_ZED=True if you prefer ZED natively
        cam = CameraInterface(use_zed=USE_ZED, camera_index=0, fps=30)
        cam.load_standard_calibration(yaml_path)
    except Exception as e:
        print(f"Failed to open camera: {e}")
        return

    estimator = ArucoDistanceEstimator(cam.camera_matrix, cam.dist_coeffs, aruco.DICT_6X6_1000)

    state = "APPROACH" if DESK_TESTING_NO_PROPELLERS else "TAKEOFF"
    stable_count = 0
    last_send = 0.0
    takeoff_started = DESK_TESTING_NO_PROPELLERS
    takeoff_start_time = None
    latest_alt_m = None

    print("=================================================================")
    if DESK_TESTING_NO_PROPELLERS:
        print(">>> [WARNING] DESK TESTING MODE ACTIVE (NO PROPS)")
        print(">>> Hijacking raw RC limits to force motors to spin on your desk.")
        print(">>> 1. Put drone in ALT_HOLD and forcefully push throttle up via your RC.")
        print(">>> 2. The script will rip the sticks from your hand virtually!")
    else:
        print(">>> [✓] REAL FLIGHT MODE ACTIVE (GPS-DENIED SAFE VELOCITY CONTROLLER)")
        print(">>> Drone taking off in GUIDED -> waiting for climb -> then ArUco centering -> LAND.")
        print(">>> Back away from the drone quickly!")
        arm_and_takeoff(TARGET_HEIGHT_M)
        takeoff_start_time = time.time()
    print("=================================================================")
    
    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                continue

            draw_crosshair(frame)

            if not DESK_TESTING_NO_PROPELLERS and state == "TAKEOFF":
                latest_alt_m = get_relative_alt_m()
                if latest_alt_m is not None:
                    cv2.putText(frame, f"TAKEOFF ALT: {latest_alt_m:.2f} / {TARGET_HEIGHT_M:.2f} m", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    if latest_alt_m >= TAKEOFF_MIN_CLIMB_M:
                        takeoff_started = True
                else:
                    cv2.putText(frame, "TAKEOFF ALT: waiting for telemetry...", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                if takeoff_start_time is not None and (time.time() - takeoff_start_time) > TAKEOFF_TIMEOUT_S:
                    if not takeoff_started:
                        raise RuntimeError("Takeoff command was sent but the vehicle never started climbing")
                    print(">>> Takeoff timeout reached. Starting vision controller with current altitude.")
                    state = "APPROACH"
                elif latest_alt_m is not None and latest_alt_m >= (TARGET_HEIGHT_M - TAKEOFF_ALT_TOLERANCE_M):
                    print(f">>> Takeoff altitude reached ({latest_alt_m:.2f} m). Starting ArUco centering.")
                    state = "APPROACH"
                
                '''
                notes: This is where my code should be put for centering and guidance of the UGV 
                once the takeoff is complete, the drone shoud move and scan like Julian coded it to
                once both markers are found, the drone should command the UGV to move towards the marker

                    Find a way to integrate this with the existing code, use moveToCenter2.py as a reference
                    - Sam
                '''

                cv2.putText(frame, f"MODE: {state}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                cv2.imshow("Precision Landing", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            # Detect all markers, using Large Marker size to start
            poses = estimator.detect_markers(frame, LARGE_MARKER_SIZE)
            
            found = False
            target_eb = None
            marker_id_to_use = None
            marker_size_m_to_use = None

            # Priority 1: Check if the Small Marker is visible
            if SMALL_MARKER_ID in poses:
                # Re-estimate properly with small marker size
                poses = estimator.detect_markers(frame, SMALL_MARKER_SIZE)
                if SMALL_MARKER_ID in poses:
                    marker_id_to_use = SMALL_MARKER_ID
                    marker_size_m_to_use = SMALL_MARKER_SIZE
            # Priority 2: Fallback to Large Marker
            elif LARGE_MARKER_ID in poses:
                marker_id_to_use = LARGE_MARKER_ID
                marker_size_m_to_use = LARGE_MARKER_SIZE

            if marker_id_to_use is not None and marker_id_to_use in poses:
                pose = poses[marker_id_to_use]
                # Convert ZED/Webcam standard frame XYZ to ArduPilot FRD body frame
                x_cam, y_cam, z_cam = float(pose.tvec[0]), float(pose.tvec[1]), float(pose.tvec[2])
                x_b = -y_cam  # Forward
                y_b = x_cam   # Right
                z_b = z_cam   # Down
                target_eb = (x_b, y_b, z_b)

                # 1. Visualization: Draw targeting line from camera center to marker center
                cam_cx, cam_cy = frame.shape[1] // 2, frame.shape[0] // 2
                marker_cx, marker_cy = pose.center_px
                cv2.line(frame, (cam_cx, cam_cy), (marker_cx, marker_cy), (0, 0, 255), 4)
                
                stable_count += 1
                found = True

                corners_arr = [pose.corners.reshape(1, 4, 2).astype(np.float32)]
                aruco.drawDetectedMarkers(frame, corners_arr, np.array([[marker_id_to_use]]))
                cv2.drawFrameAxes(frame, cam.camera_matrix, cam.dist_coeffs, pose.rvec, pose.tvec.reshape(3, 1), marker_size_m_to_use * 0.5)

                cv2.putText(frame, f"TRACKING ID: {marker_id_to_use}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"XYZ: [{x_b:.2f}, {y_b:.2f}, {z_b:.2f}]", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                now = time.time()
                if now - last_send >= (1.0 / SEND_HZ):
                    if DESK_TESTING_NO_PROPELLERS:
                        req_roll = max(1300, min(1700, 1500 + int(y_b * 200.0)))
                        req_pitch = max(1300, min(1700, 1500 - int(x_b * 200.0)))
                        send_rc_override(roll=req_roll, pitch=req_pitch, throttle=1550)
                    else:
                        # Flight mode safe controller! Extremely similar to your friend's 
                        # moveToCenter2.py script. Command velocity directly in GUIDED mode!
                        if state in ["APPROACH", "PREC_LOITER", "LANDING"]:
                            Kp = 0.8
                            vx = float(x_b) * Kp
                            vy = float(y_b) * Kp
                            
                            # Hard clamp safety speeds (0.8 m/sec is walking pace)
                            vx = max(-0.8, min(0.8, vx))
                            vy = max(-0.8, min(0.8, vy))
                            vz = 0.0
                            
                            if state == "LANDING":
                                # Utilize Lidar Lite V3 to dynamically adjust descent safely!
                                lidar_m = get_lidar_distance_m()
                                if lidar_m is not None:
                                    # Scale speed smoothly using laser altitude! 
                                    # Very slow (0.1m/s) if < 1m to gently kiss the platform.
                                    # Max speed (0.4m/s) if higher up.
                                    vz = max(0.15, min(0.4, lidar_m * 0.3))
                                    cv2.putText(frame, f"LIDAR Z: {lidar_m:.2f}m", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                else:
                                    vz = 0.4 # Fallback
                                    cv2.putText(frame, "LIDAR N/A - BLIND DESCENT", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            
                            send_guided_velocity(vx, vy, vz)
                            send_landing_target(x_b, y_b, z_b) # Supplement the math for EKF logging
                        
                    last_send = now

            else:
                stable_count = 0
                now = time.time()
                if now - last_send >= (1.0 / SEND_HZ):
                    if DESK_TESTING_NO_PROPELLERS:
                        cv2.putText(frame, "TARGET LOST - STICKS CENTERED", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        send_rc_override(1500, 1500, 1500, 1500)
                    else:
                        cv2.putText(frame, "TARGET LOST - BRAKING TO HOVER", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        send_guided_velocity(0.0, 0.0, 0.0) # Break and hover!
                    last_send = now
            # STATE MACHINE (Transitions)
            if state == "APPROACH":
                if stable_count >= PLND_STABLE_FRAMES:
                    print(">>> Target stably held. Engaging GUIDED Velocity Tracking!")
                    change_mode("GUIDED")
                    state = "PREC_LOITER"
            
            elif state == "PREC_LOITER":
                if stable_count == 0:
                    state = "APPROACH"  
                elif found and target_eb is not None:
                    err_m = math.sqrt(target_eb[0]**2 + target_eb[1]**2)
                    cv2.putText(frame, f"Align Err: {err_m:.2f}m", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    
                    if err_m < LAND_LATERAL_ERR_M:
                        print(">>> Aligned flawlessly! Engaging Direct Velocity Descent!")
                        # Notice we STAY in GUIDED mode!
                        state = "LANDING"
            
            elif state == "LANDING":
                # Wait for Touchdown
                if not master.motors_armed():
                    print(">>> TOUCHDOWN AUTO-DISARM CONFIRMED. LANDING COMPLETE.")
                    state = "LANDED"
                    break
            
            cv2.putText(frame, f"MODE: {state}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.imshow("Precision Landing", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        print("Cleaning up system...")
        if DESK_TESTING_NO_PROPELLERS:
            release_rc_override()
        else:
            send_guided_velocity(0.0, 0.0, 0.0) # Hover brake!
        cam.close()

if __name__ == "__main__":
    main()

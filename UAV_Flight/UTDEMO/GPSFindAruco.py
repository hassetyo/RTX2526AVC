import argparse
import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
from pymavlink import mavutil

import cv2
import cv2.aruco as aruco
import numpy as np

showDisplay = True
singleMarkerTest = True

#-------------- Drone Helpers ---------------------
# CONNECTION 
CONNECTION_STRING = "/dev/ttyACM0" #"udp:127.0.0.1:14551" 
BAUD_RATE = 57600

# FLIGHT PARAMS 
TARGET_ALT_M    = 5.0
HOVER_TIMEOUT_S = 10
LAND_TIMEOUT_S  = 90

# Waypoint acceptance radius (metres) and per-leg timeout (seconds)
WP_TOLERANCE_M  = 0.5
WP_TIMEOUT_S    = 30

# LOGGING 
def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")

# MAVLINK SETUP 
def connect():
    log(f"Connecting to {CONNECTION_STRING}")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    log("Heartbeat received")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,
        1
    )
    return master

# GPS CHECK 
def wait_for_gps(master, timeout=30):
    log("Waiting for strong GPS fix...")
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if msg and msg.fix_type >= 3 and msg.satellites_visible >= 8:
            log(f"GPS OK (fix={msg.fix_type}, sats={msg.satellites_visible})")
            return True
    log("GPS timeout!")
    return False

# POSITION CHECK 
def wait_for_position(master, timeout=30):
    log("Waiting for position estimate (EKF)...")
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            log("Position estimate acquired")
            return True
    log("Position timeout!")
    return False

# MODE CONTROL 
def set_mode(master, mode):
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise Exception(f"Mode {mode} not supported. Available: {list(mapping.keys())}")
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    log(f"Mode -> {mode}")
    deadline = time.time() + 5
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb:
            current_mode = mavutil.mode_string_v10(hb)
            if current_mode == mode:
                log(f"Confirmed mode: {mode}")
                return True
    log(f"Warning: Mode change to {mode} not confirmed")
    return False

# ARMING 
def arm(master):
    log("Arming...")
    master.arducopter_arm()
    deadline = time.time() + 15
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log("Armed successfully")
            return True
    log("Failed to arm (check pre-arm errors)")
    return False

# TAKEOFF 
def takeoff(master, alt):
    log(f"Taking off to {alt} m")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt
    )
    start = time.time()
    while time.time() - start < 25:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg:
            rel_alt = msg.relative_alt / 1000.0
            print(f"Altitude: {rel_alt:.2f} m", end="\r")
            if rel_alt >= alt * 0.95:
                print()
                log("Reached target altitude")
                return True
    print()
    log("Takeoff timeout!")
    return False

# LANDING 
def land(master):
    log("Landing...")
    set_mode(master, "LAND")
    deadline = time.time() + LAND_TIMEOUT_S
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb:
            armed = hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:
                log("Disarmed — landed")
                return
    log("Landing timeout — forcing disarm")
    master.arducopter_disarm()

# POSITION HELPERS 

def get_current_position(master):
    """
    Return the current (lat_deg, lon_deg, rel_alt_m) from GLOBAL_POSITION_INT.
    Blocks for up to 3 seconds.
    """
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
    if msg is None:
        raise RuntimeError("Could not get current position — no GLOBAL_POSITION_INT message.")
    return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0


def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    """
    Compute a new (lat, lon) that is north_m metres north and east_m metres east
    of the given coordinate.  Uses a flat-earth approximation valid up to ~1 km.

    Args:
        lat_deg: starting latitude in decimal degrees
        lon_deg: starting longitude in decimal degrees
        north_m: offset in metres (+north / -south)
        east_m:  offset in metres (+east  / -west)

    Returns:
        (new_lat_deg, new_lon_deg)
    """
    d_lat = north_m / 111_111.0
    d_lon = east_m  / (111_111.0 * math.cos(math.radians(lat_deg)))
    return lat_deg + d_lat, lon_deg + d_lon


def haversine_distance(lat1, lon1, lat2, lon2):
    """Return the great-circle distance in metres between two lat/lon points."""
    R = 6_371_000  # Earth radius in metres
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def send_waypoint(master, lat_deg, lon_deg, alt_m):
    """
    Command the drone to fly to an absolute GPS waypoint at the given altitude
    using SET_POSITION_TARGET_GLOBAL_INT (works in GUIDED mode).
    """
    master.mav.set_position_target_global_int_send(
        0,                                              # time_boot_ms (ignored)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000_1111_1111_1000,                          # type_mask: position only
        int(lat_deg * 1e7),
        int(lon_deg * 1e7),
        alt_m,
        0, 0, 0,                                        # vx, vy, vz (ignored)
        0, 0, 0,                                        # ax, ay, az (ignored)
        0, 0                                            # yaw, yaw_rate (ignored)
    )


def wait_until_reached(master, lat_deg, lon_deg,
                       tolerance_m=WP_TOLERANCE_M, timeout_s=WP_TIMEOUT_S):
    """
    Block until the drone is within tolerance_m metres of (lat_deg, lon_deg),
    or until timeout_s seconds elapse.

    Returns True if the waypoint was reached, False on timeout.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            cur_lat = msg.lat / 1e7
            cur_lon = msg.lon / 1e7
            dist = haversine_distance(cur_lat, cur_lon, lat_deg, lon_deg)
            print(f"  Distance to waypoint: {dist:.2f} m", end="\r")
            if dist <= tolerance_m:
                print()
                return True
    print()
    log(f"Timeout reaching waypoint ({lat_deg:.6f}, {lon_deg:.6f})")
    return False


def goto_offset(master, north_m, east_m, alt_m=None,
                tolerance_m=WP_TOLERANCE_M, timeout_s=WP_TIMEOUT_S):
    """
    Fly north_m metres north and east_m metres east of the current position.
    Altitude is maintained unless alt_m is explicitly provided.

    Returns True if the waypoint was reached.
    """
    cur_lat, cur_lon, cur_alt = get_current_position(master)
    target_lat, target_lon = offset_latlon(cur_lat, cur_lon, north_m, east_m)
    target_alt = alt_m if alt_m is not None else cur_alt

    log(f"Flying N={north_m:+.1f} m  E={east_m:+.1f} m  "
        f"-> ({target_lat:.6f}, {target_lon:.6f})  alt={target_alt:.1f} m")

    send_waypoint(master, target_lat, target_lon, target_alt)
    reached = wait_until_reached(master, target_lat, target_lon, tolerance_m, timeout_s)
    if reached:
        log("Waypoint reached.")
    return reached


#-------------- Camera Helpers ---------------------
@dataclass
class MarkerPose:
    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    center_px: Tuple[int, int]
    corners: np.ndarray

class CameraInterface:
    """
    Camera wrapper that supports:
      1) ZED / ZED X via pyzed.sl
      2) Standard laptop / USB camera via OpenCV

    For ZED, camera intrinsics are read from the SDK.
    For standard camera, you can load intrinsics from a YAML file.
    """

    def __init__(
        self,
        use_zed: bool = False,
        camera_index: int = 0,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
    ):
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
            raise RuntimeError(
                "pyzed.sl is not installed. Install the ZED SDK Python API first."
            ) from e

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
            [
                [calib.fx, 0.0, calib.cx],
                [0.0, calib.fy, calib.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

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
        if self.use_zed:
            return

        if yaml_path:
            fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
            k = fs.getNode("K").mat()
            d = fs.getNode("D").mat()
            fs.release()

            if k is None or d is None:
                raise ValueError(f"Invalid calibration file: {yaml_path}")

            self.camera_matrix = np.array(k, dtype=np.float64)
            self.dist_coeffs = np.array(d, dtype=np.float64)
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            w, h = self.width, self.height
        else:
            h, w = frame.shape[:2]

        fx = float(w)
        fy = float(w)
        cx = float(w) / 2.0
        cy = float(h) / 2.0

        self.camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def get_frame(self) -> Optional[np.ndarray]:
        if self.use_zed:
            return self._get_zed_frame()
        return self._get_standard_frame()

    def _get_zed_frame(self) -> Optional[np.ndarray]:
        sl = self.sl
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return None

        image = sl.Mat()
        self.zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()

        if frame.shape[-1] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        else:
            frame = frame.copy()

        return frame

    def _get_standard_frame(self) -> Optional[np.ndarray]:
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def close(self):
        if self.use_zed and self.zed is not None:
            self.zed.close()
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

#-------------- ArUco Detection Helpers ---------------------
class ArucoDistanceEstimator:
    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        marker_size_m: float,
        dictionary_name: int = aruco.DICT_6X6_1000,
    ):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_size_m = marker_size_m

        self.aruco_dict = aruco.getPredefinedDictionary(dictionary_name)

        if hasattr(aruco, "ArucoDetector"):
            self.detector_params = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            self.use_new_detector_api = True
        else:
            self.detector_params = aruco.DetectorParameters_create()
            self.detector = None
            self.use_new_detector_api = False

    def detect_markers(self, frame: np.ndarray) -> Dict[int, MarkerPose]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.use_new_detector_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params
            )

        poses: Dict[int, MarkerPose] = {}

        if ids is None or len(ids) == 0:
            return poses

        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec = self._estimate_pose(corners[i])
            if rvec is None or tvec is None:
                continue

            center = np.mean(corners[i][0], axis=0).astype(int)
            poses[int(marker_id)] = MarkerPose(
                marker_id=int(marker_id),
                rvec=rvec,
                tvec=tvec.reshape(3),
                center_px=(int(center[0]), int(center[1])),
                corners=corners[i][0],
            )

        return poses

    def _estimate_pose(self, corner: np.ndarray):
        half = self.marker_size_m / 2.0

        object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

        image_points = corner.reshape((4, 2)).astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success:
            return None, None

        return rvec, tvec

    def draw_markers(self, frame: np.ndarray, poses: Dict[int, MarkerPose]):
        if not poses:
            return frame

        corners = [pose.corners.reshape(1, 4, 2).astype(np.float32) for pose in poses.values()]
        ids = np.array([[pose.marker_id] for pose in poses.values()], dtype=np.int32)

        aruco.drawDetectedMarkers(frame, corners, ids)

        for pose in poses.values():
            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                pose.rvec,
                pose.tvec.reshape(3, 1),
                self.marker_size_m * 0.5,
            )

            x, y = pose.center_px
            cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)
            cv2.putText(
                frame,
                f"ID {pose.marker_id} ({x}, {y})",
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return frame

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


def draw_crosshair(frame: np.ndarray):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)
    cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)


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


FORWARD_AXIS_MAP = {
    "+x": np.array([1.0, 0.0, 0.0], dtype=np.float64),
    "-x": np.array([-1.0, 0.0, 0.0], dtype=np.float64),
    "+y": np.array([0.0, 1.0, 0.0], dtype=np.float64),
    "-y": np.array([0.0, -1.0, 0.0], dtype=np.float64),
}


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


def draw_nav_overlay(
    frame: np.ndarray,
    ugv_pose: MarkerPose,
    dest_pose: MarkerPose,
    heading_error_deg: float,
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

def get_dictionary_by_name(name: str) -> int:
    if not hasattr(aruco, name):
        valid = [x for x in dir(aruco) if x.startswith("DICT_")]
        raise ValueError(f"Unknown dictionary '{name}'. Valid examples: {valid[:10]}")
    return getattr(aruco, name)

def calculate_marker_gps(drone_lat, drone_lon, drone_yaw_deg, marker_pose: MarkerPose):
    # With a DOWNWARD camera:
    # x_m is still Right/Left
    # y_m is now Forward/Backward (relative to camera orientation)
    x_m = marker_pose.tvec[0]
    y_m = marker_pose.tvec[1] 

    yaw_rad = math.radians(drone_yaw_deg)
    
    # Rotate offsets based on drone heading
    # Note: We use y_m here instead of z_m because the camera is tilted 90 degrees down
    north_offset = -y_m * math.cos(yaw_rad) - x_m * math.sin(yaw_rad)
    east_offset = -y_m * math.sin(yaw_rad) + x_m * math.cos(yaw_rad)

    lat_offset = north_offset / 111111.0
    lon_offset = east_offset / (111111.0 * math.cos(math.radians(drone_lat)))

    return drone_lat + lat_offset, drone_lon + lon_offset

#-------------- Not sure if this is needed, will check later ---------------------
def parse_args():
    parser = argparse.ArgumentParser(description="ArUco Marker GPS Detection")
    parser.add_argument("--use-zed", action="store_true", help="Use ZED camera.")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--calibration", type=str, default=None)
    parser.add_argument("--marker-size", type=float, default=0.254)
    parser.add_argument("--dict", type=str, default="DICT_6X6_1000")
    # Only keep the destination ID if you're looking for one specific marker, for now i am not
    #parser.add_argument("--target-id", type=int, default=0, help="The ID you want to find")
    return parser.parse_args()

def request_message_streams(master):  # asks the flight controller for the messages we care about
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1,
        )
    except Exception:
        pass

    def set_interval(msg_id, hz):
        try:
            us = int(1e6 / hz)
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                us,
                0, 0, 0, 0, 0,
            )
        except Exception:
            pass

    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 15)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 5)

def connectDrone():
        print(f"Connecting to Drone: {CONNECTION_STRING}...")
        if CONNECTION_STRING.startswith("udp:") or CONNECTION_STRING.startswith("tcp:"):
            master = mavutil.mavlink_connection(CONNECTION_STRING)
        else:
            master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)

        hb = master.wait_heartbeat(timeout=10)
        if not hb:
            print("No heartbeat received from drone. Check connection and try again.")
            return None
        print("Drone Heartbeat OK.")

        request_message_streams(master)

        return master

#-------------- Main ---------------------
def main():
    print("Starting Stationary Camera Test...")
    
    master = connectDrone()
    #Initialize the camera settings
    args = parse_args()
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

    #Calibrate the ArUco detection

    estimator = ArucoDistanceEstimator(
        camera_matrix=cam.camera_matrix,
        dist_coeffs=cam.dist_coeffs,
        marker_size_m=args.marker_size,
        dictionary_name=dictionary,
    )

    try:
        #Make sure Drone precheck are good
        if not wait_for_gps(master):
            return

        if not wait_for_position(master):
            return

        if not set_mode(master, "GUIDED"):
            log("Mode change failed — aborting")
            return

        #Arm the drone
        if not arm(master):
            return
        
        time.sleep(2)  # stabilization delay
        
        #Begin camera looping
        print("Starting Camera Code")

        while True:
            frame = cam.get_frame()
            if frame is None:
                continue

            poses = estimator.detect_markers(frame)
            display = frame.copy()
            draw_crosshair(display)
            estimator.draw_markers(display, poses)

            # calculate GPS for every marker detected
            if len(poses) > 0:
                droneLat, droneLon, droneAlt = get_current_position(master)
                
                # Get Drone Heading (Yaw) from MAVLink
                msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
                droneYaw = math.degrees(msg.yaw) if msg else 0.0

                for m_id, m_pose in poses.items():
                    # Use the function we created earlier
                    markerLat, markerLon = calculate_marker_gps(droneLat, droneLon, droneYaw, m_pose)
                    
                    # Display the coordinates
                    gps_text = f"ID {m_id} GPS: {markerLat:.6f}, {markerLon:.6f}"
                    cv2.putText(display, gps_text, (20, 30 + (m_id * 30)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display, "Searching for markers...", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            if(showDisplay):
                cv2.imshow("Camera Test", display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    cv2.imwrite("Camera_Test_screenshot.png", display)
                    print("Saved: Camera_Test_screenshot.png")
    
    #Handle errors
    except Exception as e:
        print(f"Error: {e}")

    #Handle a Ctrl+C exit
    except KeyboardInterrupt:
        print("Exiting Camera Test") 
        cam.close()
        land(master)
        return 
    
    #Final clean up
    finally:
        print("Cleaning up...")
        cam.close()
        if 'master' in locals():
            land(master)




if __name__ == "__main__":
    main()

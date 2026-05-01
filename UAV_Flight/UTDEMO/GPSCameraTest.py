import argparse
import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np

showDisplay = True

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
    #parser.add_argument("--ugv-marker-id", type=int, default=5, help="ArUco ID attached to the ground vehicle.")
    parser.add_argument("--dest-marker-id", type=int, default=0, help="Destination ArUco ID.")
    parser.add_argument("--dict", type=str, default="DICT_6X6_1000", help="ArUco dictionary name.")
    parser.add_argument("--width", type=int, default=1280, help="Standard camera width.")
    parser.add_argument("--height", type=int, default=720, help="Standard camera height.")
    parser.add_argument("--fps", type=int, default=30, help="Camera FPS.")

    parser.add_argument("--bridge-port", type=str, default="/dev/ttyUSB0", help="Local ESP32 bridge serial port used to send commands.")
    parser.add_argument("--bridge-baud", type=int, default=115200, help="ESP32 bridge baud rate.")
    
    #Controlling the UGV
    '''
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
    '''
    return parser.parse_args()

def main():
    print("Starting Stationary Camera Test...")
    
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

    estimator = ArucoDistanceEstimator(
        camera_matrix=cam.camera_matrix,
        dist_coeffs=cam.dist_coeffs,
        marker_size_m=args.marker_size,
        dictionary_name=dictionary,
    )

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                print("Failed to read frame.")
                continue

            poses = estimator.detect_markers(frame)
            display = frame.copy()

            draw_crosshair(display)
            estimator.draw_markers(display, poses)

            pair = pick_two_markers(poses, args.ugv_marker_id, args.dest_marker_id)
            if pair is not None:
                ugv_pose, dest_pose = pair

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

                draw_nav_overlay(
                    display,
                    ugv_pose,
                    dest_pose,
                    heading_error_deg,
                    args.stop_distance_m,
                    cam.camera_matrix,
                    cam.dist_coeffs,
                    args.marker_size,
                    args.ugv_forward_axis,
                )
            else:
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

            if(showDisplay):
                cv2.imshow("Camera Test", display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    cv2.imwrite("Camera_Test_screenshot.png", display)
                    print("Saved: Camera_Test_screenshot.png")

    except KeyboardInterrupt:
        print("Exiting Camera Test") 
        cam.close()
        return 


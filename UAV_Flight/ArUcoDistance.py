# ArUcoDistance.py
import argparse
import math
import sys
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np


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
        init_params.depth_mode = sl.DEPTH_MODE.NONE  # not needed for ArUco pose here

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {err}")

        # Read left-camera intrinsics from ZED SDK
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

        # ZED distortion parameters are typically available in calib.disto
        dist = np.array(calib.disto, dtype=np.float64).flatten()
        if dist.size >= 5:
            self.dist_coeffs = dist[:5].reshape(-1, 1)
        elif dist.size > 0:
            self.dist_coeffs = dist.reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def _open_standard(self):
        # CAP_DSHOW helps on Windows; harmless fallback elsewhere.
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {self.camera_index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def load_standard_calibration(self, yaml_path: Optional[str]):
        """
        Load standard camera intrinsics from YAML.
        Expected nodes:
          K : camera matrix
          D : distortion coeffs

        If yaml_path is None, create a rough fallback intrinsic matrix so the
        program can still run for testing, but measurements will be less accurate.
        """
        if self.use_zed:
            return

        if yaml_path:
            fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
            K = fs.getNode("K").mat()
            D = fs.getNode("D").mat()
            fs.release()

            if K is None or D is None:
                raise ValueError(f"Invalid calibration file: {yaml_path}")

            self.camera_matrix = np.array(K, dtype=np.float64)
            self.dist_coeffs = np.array(D, dtype=np.float64)
            return

        # Rough fallback for quick testing only
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

        # ZED SDK returns BGRA in many Python examples
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

        # Support both newer and older OpenCV ArUco APIs
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
                [-half,  half, 0.0],
                [ half,  half, 0.0],
                [ half, -half, 0.0],
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
    # 3D position of each marker center in camera coordinates, meters
    p1 = pose_a.tvec.reshape(3)
    p2 = pose_b.tvec.reshape(3)

    diff = p2 - p1
    dist_m = np.linalg.norm(diff)

    x_diff_mm = diff[0] * 1000.0
    y_diff_mm = diff[1] * 1000.0
    z_diff_mm = diff[2] * 1000.0
    dist_mm = dist_m * 1000.0

    # Draw line between marker centers in image space
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
        (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
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


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure 3D distance between two ArUco markers using ZED X or a standard camera."
    )
    parser.add_argument(
        "--use-zed",
        action="store_true",
        help="Use Stereolabs ZED / ZED X camera.",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="Standard camera index for laptop/USB webcam.",
    )
    parser.add_argument(
        "--calibration",
        type=str,
        default=None,
        help="YAML calibration file for standard camera with nodes K and D.",
    )
    parser.add_argument(
        "--marker-size",
        type=float,
        default=0.254,
        help="Marker size in meters. Example: 0.10 for 10 cm.",
    )
    parser.add_argument(
        "--marker1-id",
        type=int,
        default=None,
        help="First marker ID. If omitted, script uses the first detected marker.",
    )
    parser.add_argument(
        "--marker2-id",
        type=int,
        default=None,
        help="Second marker ID. If omitted, script uses the second detected marker.",
    )
    parser.add_argument(
        "--dict",
        type=str,
        default="DICT_6X6_1000",
        help="ArUco dictionary name, e.g. DICT_6X6_1000, DICT_4X4_50, DICT_7X7_1000.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Standard camera width.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Standard camera height.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Camera FPS.",
    )
    return parser.parse_args()


def get_dictionary_by_name(name: str) -> int:
    if not hasattr(aruco, name):
        valid = [x for x in dir(aruco) if x.startswith("DICT_")]
        raise ValueError(f"Unknown dictionary '{name}'. Valid examples: {valid[:10]}")
    return getattr(aruco, name)


def main():
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

    print("Press q to quit.")
    print("Press s to save a screenshot.")
    print("If --marker1-id and --marker2-id are not provided, the first two visible markers are used.")

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

            pair = pick_two_markers(poses, args.marker1_id, args.marker2_id)
            if pair is not None:
                pose_a, pose_b = pair
                draw_distance_overlay(display, pose_a, pose_b)
            else:
                cv2.putText(
                    display,
                    "Need two visible markers",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow("ArUco Distance", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                cv2.imwrite("aruco_distance_screenshot.png", display)
                print("Saved: aruco_distance_screenshot.png")

    finally:
        cam.close()


if __name__ == "__main__":
    main()
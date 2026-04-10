"""
UAV Vision Module for ArUco Marker Detection
Detects markers and provides position data for UAV-UGV coordination
"""

import cv2
import cv2.aruco as aruco
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List


@dataclass
class MarkerPosition:
    """Data class for marker position information"""
    marker_id: int
    x: float  # Side distance (meters)
    y: float  # Forward distance (meters)
    z: float  # Height distance (meters)
    distance: float  # Total distance (meters)
    detected: bool = True


class CameraInterface:
    """Handles camera initialization and frame capture"""

    def __init__(self, use_zed: bool = False, camera_index: int = 0):
        self.use_zed = use_zed
        self.camera_index = camera_index
        self.cap = None
        self.zed = None
        print(f"Initializing {'ZED' if use_zed else 'Standard'} camera...")

        if self.use_zed:
            self._initialize_zed()
        else:
            self._initialize_standard()

        print("Camera initialized successfully")

    def _initialize_zed(self):
        """Initialize ZED camera"""
        import pyzed.sl as sl
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 1)

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED camera error: {status}")

    def _initialize_standard(self):
        """Initialize standard USB camera on Windows"""
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)

        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open standard camera at index {self.camera_index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def get_frame(self) -> Optional[np.ndarray]:
        """Capture a frame from the camera"""
        if self.use_zed:
            return self._get_zed_frame()
        else:
            return self._get_standard_frame()

    def _get_zed_frame(self) -> Optional[np.ndarray]:
        """Get frame from ZED camera"""
        import pyzed.sl as sl
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            print("Error grabbing ZED frame")
            return None

        image = sl.Mat()
        self.zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def _get_standard_frame(self) -> Optional[np.ndarray]:
        """Get frame from standard camera"""
        ret, frame = self.cap.read()
        if not ret:
            print("Error grabbing standard frame")
            return None
        return frame

    def close(self):
        """Release camera resources"""
        if self.use_zed and self.zed:
            self.zed.close()
        elif self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


class CenterZone:
    """
    Defines a rectangular zone around the frame center.
    A marker inside the zone is considered 'on target'.
    """

    def __init__(self, box_width: int = 200, box_height: int = 200):
        self.box_width = box_width
        self.box_height = box_height

    def resize(self, box_width: int, box_height: int):
        self.box_width = box_width
        self.box_height = box_height
        print(f"Center zone resized to {box_width}x{box_height}px")

    def bounds(self, frame_w: int, frame_h: int) -> Tuple[int, int, int, int]:
        cx, cy = frame_w / 2.0, frame_h * 0.8
        half_w = self.box_width / 2.0
        half_h = self.box_height / 2.0
        return (
            int(cx - half_w),
            int(cy - half_h),
            int(cx + half_w),
            int(cy + half_h),
        )

    def contains(self, dx: float, dy: float) -> bool:
        return (abs(dx) <= self.box_width / 2.0 and
                abs(dy) <= self.box_height / 2.0)

    def draw(self, frame: np.ndarray, in_zone: bool) -> np.ndarray:
        h, w = frame.shape[:2]
        x_min, y_min, x_max, y_max = self.bounds(w, h)
        color = (0, 255, 0) if in_zone else (0, 0, 255)
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)

        label = "IN ZONE" if in_zone else "OUT OF ZONE"
        cv2.putText(frame, label, (x_min, y_min - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        return frame


class ArucoDetector:
    """Handles ArUco marker detection and pose estimation"""

    def __init__(self, calibration_file: str, marker_size: float = 0.1,
                 dictionary=aruco.DICT_6X6_1000):
        self.marker_size = marker_size
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary)
        self.camera_matrix, self.dist_coeffs = self._load_calibration(calibration_file)

        self.detector_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        print(f"Loaded calibration from {calibration_file}")
        print(f"Marker size: {marker_size}m, Dictionary: DICT_6X6_1000")

    def _load_calibration(self, file_path: str) -> Tuple[np.ndarray, np.ndarray]:
        fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
        camera_matrix = fs.getNode("K").mat()
        dist_coeffs = fs.getNode("D").mat()
        fs.release()

        if camera_matrix is None or dist_coeffs is None:
            raise ValueError(f"Invalid calibration file: {file_path}")

        return camera_matrix, dist_coeffs

    def _estimate_single_marker_pose(self, corner: np.ndarray):
        """
        Estimate pose for one detected marker using solvePnP.
        Replaces old aruco.estimatePoseSingleMarkers() for newer OpenCV builds.
        """
        half_size = self.marker_size / 2.0

        object_points = np.array([
            [-half_size,  half_size, 0.0],
            [ half_size,  half_size, 0.0],
            [ half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ], dtype=np.float32)

        image_points = corner.reshape((4, 2)).astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not success:
            return None, None

        return rvec, tvec

    def detect(self, frame: np.ndarray) -> List[MarkerPosition]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        positions = []

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec = self._estimate_single_marker_pose(corners[i])

                if rvec is None or tvec is None:
                    continue

                tvec = tvec.flatten()
                x, y, z = tvec[0], tvec[1], tvec[2]
                distance = np.linalg.norm(tvec)

                position = MarkerPosition(
                    marker_id=int(marker_id),
                    x=float(x),
                    y=float(y),
                    z=float(z),
                    distance=float(distance)
                )
                positions.append(position)

        return positions

    def draw_detections(self, frame: np.ndarray, positions: List[MarkerPosition]) -> np.ndarray:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec = self._estimate_single_marker_pose(corners[i])

                if rvec is None or tvec is None:
                    continue

                rvec = rvec.flatten()
                tvec = tvec.flatten()

                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size * 0.5
                )

                position = next((p for p in positions if p.marker_id == marker_id), None)
                if position:
                    corner = corners[i][0]
                    center = np.mean(corner, axis=0).astype(int)

                    color = (0, 255, 0) if marker_id == 0 else (0, 0, 255)
                    label = f"ID:{marker_id} D:{position.distance:.2f}m"

                    cv2.putText(
                        frame,
                        label,
                        tuple(center),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2,
                        cv2.LINE_AA
                    )

        return frame


class UAVVision:
    """Main vision system for UAV"""

    def __init__(self, calibration_file: str = "calibration_chessboard.yaml",
                 marker_size: float = 0.1, use_zed: bool = False,
                 zone_box_width: int = 200, zone_box_height: int = 200,
                 camera_index: int = 0):
        self.camera = CameraInterface(use_zed=use_zed, camera_index=camera_index)
        self.detector = ArucoDetector(calibration_file, marker_size)
        self.target_marker_id = 0
        self.center_zone = CenterZone(zone_box_width, zone_box_height)

    def get_target_position(self) -> Optional[MarkerPosition]:
        frame = self.camera.get_frame()
        if frame is None:
            return None

        positions = self.detector.detect(frame)
        target = next((p for p in positions if p.marker_id == self.target_marker_id), None)
        return target

    def process_frame(self, display: bool = True) -> Tuple[Optional[List[MarkerPosition]], Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        frame = self.camera.get_frame()
        if frame is None:
            return None, None, None, None

        positions = self.detector.detect(frame)
        annotated_frame = self.detector.draw_detections(frame.copy(), positions)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detector.detectMarkers(gray)

        frame_h, frame_w = annotated_frame.shape[:2]
        frame_center_x = frame_w / 2.0
        frame_center_y = frame_h / 2.0

        target_in_zone = False
        if ids is not None:
            target_index = next(
                (i for i, mid in enumerate(ids.flatten())
                 if mid == self.target_marker_id), None
            )
            if target_index is not None:
                mc = np.mean(corners[target_index][0], axis=0)
                dx = mc[0] - frame_center_x
                dy = mc[1] - frame_center_y
                target_in_zone = self.center_zone.contains(dx, dy)

        if display:
            self.center_zone.draw(annotated_frame, target_in_zone)

            cx, cy = int(frame_center_x), int(frame_center_y)
            cv2.drawMarker(annotated_frame, (cx, cy), (255, 255, 255),
                           cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)

            for i, pos in enumerate(positions):
                info = (f"ID {pos.marker_id}: "
                        f"X:{pos.x:.2f} Y:{pos.y:.2f} Z:{pos.z:.2f} "
                        f"D:{pos.distance:.2f}m")
                cv2.putText(annotated_frame, info, (10, 30 + i * 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("UAV Vision", annotated_frame)

        return positions, annotated_frame, corners, ids

    def close(self):
        self.camera.close()


def main():
    print("UAV Vision System Test")
    print("Press 'q' to quit")
    print("Press '+' / '-' to grow or shrink the centre zone by 20px")

    ZONE_BOX_WIDTH = 200
    ZONE_BOX_HEIGHT = 200

    vision = UAVVision(
        calibration_file="calibration_chessboard.yaml",
        marker_size=0.1,
        use_zed=False,
        zone_box_width=ZONE_BOX_WIDTH,
        zone_box_height=ZONE_BOX_HEIGHT,
        camera_index=0,
    )

    try:
        while True:
            positions, frame, corners, ids = vision.process_frame(display=True)

            if positions and frame is not None and ids is not None:
                frame_h, frame_w = frame.shape[:2]
                frame_center_x = frame_w / 2.0
                frame_center_y = frame_h / 2.0

                for i, pos in enumerate(positions):
                    marker_corners = corners[i][0]
                    marker_center = np.mean(marker_corners, axis=0)
                    dx = marker_center[0] - frame_center_x
                    dy = marker_center[1] - frame_center_y
                    in_zone = vision.center_zone.contains(dx, dy)

                    if in_zone:
                        print(f"Marker {pos.marker_id}: INSIDE zone (dx={dx:.1f}px, dy={dy:.1f}px)")
                    else:
                        print(f"Marker {pos.marker_id}: OUTSIDE zone — offset dx={dx:.1f}px, dy={dy:.1f}px")

                target_index = next(
                    (i for i, p in enumerate(positions)
                     if p.marker_id == vision.target_marker_id), None
                )
                if target_index is not None:
                    mc = np.mean(corners[target_index][0], axis=0)
                    dx = mc[0] - frame_center_x
                    dy = mc[1] - frame_center_y
                    in_zone = vision.center_zone.contains(dx, dy)

                    if in_zone:
                        print(">>> TARGET FOUND: inside centre zone ✓")
                    else:
                        print(f">>> TARGET FOUND: outside centre zone — dx={dx:.1f}px, dy={dy:.1f}px")

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('+') or key == ord('='):
                w = vision.center_zone.box_width + 20
                h = vision.center_zone.box_height + 20
                vision.center_zone.resize(w, h)
            elif key == ord('-'):
                w = max(20, vision.center_zone.box_width - 20)
                h = max(20, vision.center_zone.box_height - 20)
                vision.center_zone.resize(w, h)

    finally:
        vision.close()
        print("Vision system closed")


if __name__ == "__main__":
    main()
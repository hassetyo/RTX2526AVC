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
        """Initialize standard USB camera"""
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_AVFOUNDATION)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open standard camera")
        
        # Set resolution
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


class ArucoDetector:
    """Handles ArUco marker detection and pose estimation"""
    
    def __init__(self, calibration_file: str, marker_size: float = 0.1,
                 dictionary=aruco.DICT_6X6_1000):
        """
        Initialize ArUco detector
        
        Args:
            calibration_file: Path to camera calibration YAML file
            marker_size: Physical size of markers in meters
            dictionary: ArUco dictionary to use
        """
        self.marker_size = marker_size
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary)
        self.camera_matrix, self.dist_coeffs = self._load_calibration(calibration_file)
        print(f"Loaded calibration from {calibration_file}")
        print(f"Marker size: {marker_size}m, Dictionary: DICT_6X6_1000")
    
    def _load_calibration(self, file_path: str) -> Tuple[np.ndarray, np.ndarray]:
        """Load camera calibration from YAML file"""
        fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
        camera_matrix = fs.getNode("K").mat()
        dist_coeffs = fs.getNode("D").mat()
        fs.release()
        
        if camera_matrix is None or dist_coeffs is None:
            raise ValueError(f"Invalid calibration file: {file_path}")
        
        return camera_matrix, dist_coeffs
    
    def detect(self, frame: np.ndarray) -> List[MarkerPosition]:
        """
        Detect ArUco markers and estimate their positions
        
        Args:
            frame: Input image frame
        
        Returns:
            List of MarkerPosition objects for detected markers
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        
        positions = []
        
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size, 
                    self.camera_matrix, self.dist_coeffs
                )
                
                tvec = tvecs[0].flatten()
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
        """
        Draw detected markers and their information on the frame
        
        Args:
            frame: Input frame
            positions: List of detected marker positions
        
        Returns:
            Annotated frame
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        
        if ids is not None:
            # Draw marker boundaries
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Draw axes and labels for each marker
            for i, marker_id in enumerate(ids.flatten()):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size,
                    self.camera_matrix, self.dist_coeffs
                )
                
                rvec = rvecs[0].flatten()
                tvec = tvecs[0].flatten()
                
                # Draw 3D axes
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                rvec, tvec, self.marker_size * 0.5)
                
                # Find matching position data
                position = next((p for p in positions if p.marker_id == marker_id), None)
                if position:
                    # Add label
                    corner = corners[i][0]
                    center = np.mean(corner, axis=0).astype(int)
                    
                    color = (0, 255, 0) if marker_id == 0 else (0, 0, 255)
                    label = f"ID:{marker_id} D:{position.distance:.2f}m"
                    
                    cv2.putText(frame, label, tuple(center), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        
        return frame


class UAVVision:
    """Main vision system for UAV"""
    
    def __init__(self, calibration_file: str = "calibration_chessboard.yaml",
                 marker_size: float = 0.1, use_zed: bool = False):
        """
        Initialize UAV vision system
        
        Args:
            calibration_file: Path to calibration file
            marker_size: Marker size in meters
            use_zed: Use ZED camera if True, standard camera if False
        """
        self.camera = CameraInterface(use_zed=use_zed)
        self.detector = ArucoDetector(calibration_file, marker_size)
        self.target_marker_id = 0  # Default target marker ID
    
    def get_target_position(self) -> Optional[MarkerPosition]:
        """
        Get position of the target marker (ID 0 by default)
        
        Returns:
            MarkerPosition if target found, None otherwise
        """
        frame = self.camera.get_frame()
        if frame is None:
            return None
        
        positions = self.detector.detect(frame)
        
        # Find target marker
        target = next((p for p in positions if p.marker_id == self.target_marker_id), None)
        return target
    
    def process_frame(self, display: bool = True) -> Tuple[Optional[List[MarkerPosition]], Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Process a single frame: detect markers and optionally display
        
        Args:
            display: If True, show annotated frame
        
        Returns:
            Tuple of (positions list, annotated frame, corners, ids)
        """
        frame = self.camera.get_frame()
        if frame is None:
            return None, None, None, None
        
        positions = self.detector.detect(frame)
        annotated_frame = self.detector.draw_detections(frame.copy(), positions)

        # Reuse same detection logic to get marker image centers for printing offsets
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.detector.aruco_dict)
        
        if display:
            # Add position info overlay
            for i, pos in enumerate(positions):
                info = f"ID {pos.marker_id}: X:{pos.x:.2f} Y:{pos.y:.2f} Z:{pos.z:.2f} D:{pos.distance:.2f}m"
                cv2.putText(annotated_frame, info, (10, 30 + i * 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            
            cv2.imshow("UAV Vision", annotated_frame)
        
        return positions, annotated_frame, corners, ids
    
    def close(self):
        """Cleanup resources"""
        self.camera.close()


def main():
    """Test the vision system"""
    print("UAV Vision System Test")
    print("Press 'q' to quit")
    
    vision = UAVVision(calibration_file="calibration_chessboard.yaml",
                      marker_size=0.1,
                      use_zed=False)
    
    try:
        while True:
            positions, frame, corners, ids = vision.process_frame(display=True)
            
            if positions and frame is not None and ids is not None:
                frame_h, frame_w = frame.shape[:2]
                frame_center_x = frame_w / 2.0
                frame_center_y = frame_h / 2.0

                # Print detected markers as offset from image center instead of pose location
                for i, pos in enumerate(positions):
                    marker_corners = corners[i][0]
                    marker_center = np.mean(marker_corners, axis=0)
                    dx = marker_center[0] - frame_center_x
                    dy = marker_center[1] - frame_center_y

                    print(f"Marker {pos.marker_id}: "
                          f"center offset dx={dx:.1f}px, dy={dy:.1f}px")
                
                # Check if target marker found
                target_index = next((i for i, p in enumerate(positions) if p.marker_id == 0), None)
                if target_index is not None:
                    marker_corners = corners[target_index][0]
                    marker_center = np.mean(marker_corners, axis=0)
                    dx = marker_center[0] - frame_center_x
                    dy = marker_center[1] - frame_center_y

                    print(f">>> TARGET FOUND offset from center: dx={dx:.1f}px, dy={dy:.1f}px")
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        vision.close()
        print("Vision system closed")


if __name__ == "__main__":
    main()
import cv2
import cv2.aruco as aruco
import numpy as np
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

@dataclass
class MarkerPose:
    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    center_px: Tuple[int, int]
    corners: np.ndarray

class ArucoDistanceEstimator:
    def __init__(self, camera_matrix, dist_coeffs, marker_size_m, dictionary_name=aruco.DICT_6X6_1000):
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
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)

        poses: Dict[int, MarkerPose] = {}
        if ids is None or len(ids) == 0:
            return poses

        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec = self._estimate_pose(corners[i])
            if rvec is None or tvec is None: continue
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
        obj_pts = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
        img_pts = corner.reshape((4, 2)).astype(np.float32)
        success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        return (rvec, tvec) if success else (None, None)

    def draw_markers(self, frame, poses):
        if not poses: return frame
        corners = [p.corners.reshape(1, 4, 2).astype(np.float32) for p in poses.values()]
        ids = np.array([[p.marker_id] for p in poses.values()], dtype=np.int32)
        aruco.drawDetectedMarkers(frame, corners, ids)
        for p in poses.values():
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, p.rvec, p.tvec.reshape(3, 1), self.marker_size_m * 0.5)
        return frame
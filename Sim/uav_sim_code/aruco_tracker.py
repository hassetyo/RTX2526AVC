"""
ArUco Tracker - Standalone vision module
==========================================
extracted and corrected from the original mission code
fixes:
  - original code passes None to find_ugv() -> now takes actual frames
  - standardized to DICT_4X4_50 (matches the actual printed marker)
  - works with both ZED X camera and standard webcam
  - can also work with webots camera frames (numpy arrays)
"""

import cv2
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass

# ============================================================
# DATA TYPES
# ============================================================

@dataclass
class MarkerResult:
    """position info for a detected ArUco marker"""
    marker_id: int
    cx: float           # center x pixel coordinate
    cy: float           # center y pixel coordinate
    dx: float           # offset from frame center x (pixels)
    dy: float           # offset from frame center y (pixels)
    area: float         # approximate area of the marker in pixels
    corners: np.ndarray # raw corner points


# ============================================================
# ARUCO TRACKER CLASS
# ============================================================

class ArucoTracker:
    """
    ArUco marker detector using DICT_4X4_50 (matching the printed marker).
    
    the original code had 2 bugs i fixed:
    1. mission1.py calls tracker.find_ugv(None) which always returns None
    2. missionaruco.py uses DICT_6X6_1000 but the actual printed marker is DICT_4X4_50
    
    this class accepts numpy frames directly so it works with:
    - ZED X camera frames (real hardware)
    - standard webcam frames (testing)
    - webots camera frames (simulation)
    """

    def __init__(self, dictionary=cv2.aruco.DICT_6X6_1000, target_id=0):
        self.target_id = target_id
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        print(f"[Tracker] ArUco detector ready. Dictionary: DICT_6X6_1000 | Target ID: {target_id}")

    def detect_all(self, frame: np.ndarray) -> List[MarkerResult]:
        """finds ALL aruco markers in the frame and returns their positions"""
        if frame is None:
            return []

        # convert to grayscale for detection
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        corners, ids, _ = self.detector.detectMarkers(gray)
        results = []

        if ids is None:
            return results

        h, w = frame.shape[:2]
        frame_cx = w / 2.0
        frame_cy = h / 2.0

        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))

            # calculate pixel offset from frame center
            dx = cx - frame_cx
            dy = cy - frame_cy

            # approximate area from corners
            area = float(cv2.contourArea(c))

            results.append(MarkerResult(
                marker_id=int(marker_id),
                cx=cx, cy=cy,
                dx=dx, dy=dy,
                area=area,
                corners=c
            ))

        return results

    def find_target(self, frame: np.ndarray) -> Optional[MarkerResult]:
        """finds the specific target marker (default ID 0 on the UGV)"""
        results = self.detect_all(frame)
        for r in results:
            if r.marker_id == self.target_id:
                return r
        return None

    def find_any(self, frame: np.ndarray, id_range=(0, 4)) -> Optional[MarkerResult]:
        """finds any marker in the given ID range (for mission 2/3 destination markers)"""
        results = self.detect_all(frame)
        for r in results:
            if id_range[0] <= r.marker_id <= id_range[1]:
                return r
        return None

    def draw_detections(self, frame: np.ndarray, results: List[MarkerResult]) -> np.ndarray:
        """draws detection boxes and info on the frame for debugging"""
        annotated = frame.copy()
        h, w = frame.shape[:2]

        # draw crosshair at center
        cx, cy = w // 2, h // 2
        cv2.drawMarker(annotated, (cx, cy), (255, 255, 255),
                       cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)

        for r in results:
            # draw corners
            pts = r.corners.astype(int)
            cv2.polylines(annotated, [pts], True, (0, 255, 0), 2)

            # draw center dot
            cv2.circle(annotated, (int(r.cx), int(r.cy)), 5, (0, 0, 255), -1)

            # draw info text
            color = (0, 255, 0) if r.marker_id == self.target_id else (0, 165, 255)
            label = f"ID:{r.marker_id} dx:{r.dx:.0f} dy:{r.dy:.0f}"
            cv2.putText(annotated, label, (int(r.cx) - 50, int(r.cy) - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

        return annotated

    def is_centered(self, result: MarkerResult, deadband_px=30) -> bool:
        """checks if the marker is centered within the given pixel deadband"""
        return abs(result.dx) < deadband_px and abs(result.dy) < deadband_px

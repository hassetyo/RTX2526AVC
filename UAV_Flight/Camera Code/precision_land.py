"""
Challenge 1 - UAV Precision Landing on Moving UGV
=================================================
Hardware: ZED X Camera, Cube Orange+, Holybro H-Flow, Lidar Lite V3
Logic: 
  - Stabilize using Optical Flow + Lidar
  - Detect nested ArUco markers (Large: ID 10, Small: ID 20)
  - Estimate 6D pose -> convert to Body Frame FRD
  - Stream LANDING_TARGET messages at 15Hz
  - Automate flight mode transitions: LOITER -> PRECISION LOITER -> LAND
"""
import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np
from pymavlink import mavutil

def draw_crosshair(frame: np.ndarray):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)
    cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)

# ─── USER CONFIGURATION ────────────────────────────────────────────────────────
MAVLINK_CONN = "/dev/ttyACM0"      # Or udp:127.0.0.1:14550
BAUD_RATE    = 921600

# Nested ArUco Marker Setup
LARGE_MARKER_ID   = 5
SMALL_MARKER_ID   = 6
LARGE_MARKER_SIZE = 0.3048 # 12 inches -> meters
SMALL_MARKER_SIZE = 0.1016 # 4 inches -> meters

# Switch to the small marker when altitude is below this (meters)
USE_SMALL_BELOW_M = 1.0

# Send rate for LANDING_TARGET
SEND_HZ = 15.0

# Camera Calibration File
import os
CALIB_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_chessboard.yaml")
CAM_MATRIX = None
DIST_COEFFS = None

# ─── MAVLINK HELPERS ───────────────────────────────────────────────────────────
master = mavutil.mavlink_connection(MAVLINK_CONN, baud=BAUD_RATE)
print("Waiting for heartbeats from UAV...")
master.wait_heartbeat()
print("Heartbeat received!")

def send_landing_target(x_b, y_b, z_b):
    """
    Send LANDING_TARGET to ArduPilot in Body Frame (FRD).
    x_b = Forward (meters)
    y_b = Right (meters)
    z_b = Down (meters)
    """
    master.mav.landing_target_send(
        int(time.time() * 1e6),              # time_usec
        0,                                   # target_num (0 = default)
        mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame
        0.0,                                 # angle_x (not used for body frame)
        0.0,                                 # angle_y (not used for body frame)
        abs(z_b),                            # distance 
        0.0, 0.0,                            # size_x, size_y (rad)
        x_b,                                 # x position
        y_b,                                 # y position
        abs(z_b),                            # z position
        (1.0, 0.0, 0.0, 0.0),                # q
        0,                                   # type
        1                                    # position_valid flag
    )

# Removed automated flight logic to allow desktop testing by hand!
# ─── CAMERA SETUP ──────────────────────────────────────────────────────────────
# Using standard OpenCV + your YAML calibration file
print("Loading camera calibration from YAML...")
fs = cv2.FileStorage(CALIB_YAML, cv2.FILE_STORAGE_READ)
if not fs.isOpened():
    print(f"CRITICAL ERROR: Could not open {CALIB_YAML}")
    print("Ensure the file exists on the Jetson if running there.")
    exit(1)
CAM_MATRIX = fs.getNode("K").mat()
DIST_COEFFS = fs.getNode("D").mat()
fs.release()

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open /dev/video0")
    exit(1)

# Set resolution explicitly
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
if hasattr(aruco, "ArucoDetector"):
    aruco_params = aruco.DetectorParameters()
    aruco_detector = aruco.ArucoDetector(aruco_dict, aruco_params)
else:
    aruco_params = aruco.DetectorParameters_create()
    aruco_detector = None # Older OpenCV fallback

# ─── MAIN LOOP ─────────────────────────────────────────────────────────────
# We act exclusively as a visual mathematical relay for Ardupilot.
print(">>> Desk Testing Overrides Mode!")
print(">>> Send TARGET data constantly. To test motors:")
print(">>> 1. Arm your drone manually on the desk. (Bypass prearm checks if needed)")
print(">>> 2. Flip your remote controller switch to LAND.")
print(">>> 3. Slide the marker under the camera. The motors will speed up as it tries to fix its orientation!")

last_send = 0.0

try:
    while True:
        master.recv_match(blocking=False) # Keep MAVLink buffers clear
        
        # Grab frame with OpenCV
        ret, frame = cap.read()
        if not ret: continue

        draw_crosshair(frame)

        # Detect markers
        if aruco_detector:
            corners, ids, _ = aruco_detector.detectMarkers(frame)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        # For desk testing, assume constant height if not provided to simulate altitude
        rel_alt = 1.0

        # Choose target size based on altitude
        marker_id = LARGE_MARKER_ID
        marker_size = LARGE_MARKER_SIZE
        if rel_alt < USE_SMALL_BELOW_M:
            marker_id = SMALL_MARKER_ID
            marker_size = SMALL_MARKER_SIZE

        found = False
        target_eb = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            if marker_id in ids_flat:
                idx = int(np.where(ids_flat == marker_id)[0][0])

                # 3D object points
                half_s = marker_size / 2.0
                obj_pts = np.array([
                    [-half_s,  half_s, 0],
                    [ half_s,  half_s, 0],
                    [ half_s, -half_s, 0],
                    [-half_s, -half_s, 0]
                ], dtype=np.float32)

                img_pts = corners[idx].reshape(4, 2).astype(np.float32)

                # Solve PnP
                ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, CAM_MATRIX, DIST_COEFFS, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                if ok:
                    # Convert to Body Frame (FRD)
                    # Downward camera: X_img(Right) -> Y_body, Y_img(Down_img/Back_veh) -> -X_body
                    x_cam, y_cam, z_cam = tvec[0][0], tvec[1][0], tvec[2][0]
                    x_b = -y_cam  # Forward
                    y_b = x_cam   # Right
                    z_b = z_cam   # Down (Depth)
                    
                    target_eb = (x_b, y_b, z_b)

                    # Send landing target at required frequency
                    now = time.time()
                    if now - last_send >= (1.0 / SEND_HZ):
                        send_landing_target(x_b, y_b, z_b)
                        last_send = now

                    stable_count += 1
                    found = True

                    # Visualization (Green Boxes and Axes)
                    aruco.drawDetectedMarkers(frame, [img_pts.reshape(1, 4, 2)], np.array([[marker_id]]))
                    cv2.drawFrameAxes(frame, CAM_MATRIX, DIST_COEFFS, rvec, tvec, marker_size * 0.5)

                    # Print Data Overlay
                    cv2.putText(frame, f"MARKER {marker_id} LOCKED", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"XYZ Err: [{x_b:.2f}, {y_b:.2f}, {z_b:.2f}] m", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Precision Landing", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    print("Cleaning up...")
    if 'cap' in locals() and cap is not None:
        cap.release()
    cv2.destroyAllWindows()

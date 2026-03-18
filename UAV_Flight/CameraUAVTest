"""
UAV Mission 5 - Vision Guided
Takeoff -> Fly 5m Forward -> Center on ArUco (60s) -> Re-center -> Land

Built on top of the confirmed-working UAVVision module.
All camera init, aruco detection, pose estimation and frame annotation
are handled by UAVVision / ArucoDetector - the same code that works in the standalone test.
"""

# ── standard imports ──────────────────────────────────────────────────────────
from pymavlink import mavutil
import time
import cv2
import numpy as np

# ── confirmed-working vision module (keep this 100% intact) ──────────────────
import cv2.aruco as aruco
from dataclasses import dataclass
from typing import Optional, Tuple, List


@dataclass
class MarkerPosition:
    """Data class for marker position information"""
    marker_id: int
    x: float        # Side distance in meters     (tvec[0])
    y: float        # Forward distance in meters   (tvec[1])
    z: float        # Height distance in meters    (tvec[2])
    distance: float # Total distance in meters
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
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_AVFOUNDATION)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open standard camera")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def get_frame(self) -> Optional[np.ndarray]:
        if self.use_zed:
            return self._get_zed_frame()
        else:
            return self._get_standard_frame()

    def _get_zed_frame(self) -> Optional[np.ndarray]:
        import pyzed.sl as sl
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            print("Error grabbing ZED frame")
            return None
        image = sl.Mat()
        self.zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def _get_standard_frame(self) -> Optional[np.ndarray]:
        ret, frame = self.cap.read()
        if not ret:
            print("Error grabbing standard frame")
            return None
        return frame

    def close(self):
        if self.use_zed and self.zed:
            self.zed.close()
        elif self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


class ArucoDetector:
    """Handles ArUco marker detection and pose estimation"""

    def __init__(self, calibration_file: str, marker_size: float = 0.1,
                 dictionary=aruco.DICT_6X6_1000):
        self.marker_size = marker_size
        self.aruco_dict  = aruco.getPredefinedDictionary(dictionary)
        self.camera_matrix, self.dist_coeffs = self._load_calibration(calibration_file)
        print(f"Loaded calibration from {calibration_file}")
        print(f"Marker size: {marker_size}m, Dictionary: DICT_6X6_1000")

    def _load_calibration(self, file_path: str) -> Tuple[np.ndarray, np.ndarray]:
        fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
        camera_matrix = fs.getNode("K").mat()
        dist_coeffs   = fs.getNode("D").mat()
        fs.release()
        if camera_matrix is None or dist_coeffs is None:
            raise ValueError(f"Invalid calibration file: {file_path}")
        return camera_matrix, dist_coeffs

    def detect(self, frame: np.ndarray) -> List[MarkerPosition]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        positions = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size,
                    self.camera_matrix, self.dist_coeffs
                )
                tvec     = tvecs[0].flatten()
                x, y, z  = tvec[0], tvec[1], tvec[2]
                distance = float(np.linalg.norm(tvec))
                positions.append(MarkerPosition(
                    marker_id=int(marker_id),
                    x=float(x), y=float(y), z=float(z),
                    distance=distance
                ))
        return positions

    def draw_detections(self, frame: np.ndarray,
                        positions: List[MarkerPosition]) -> np.ndarray:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id in enumerate(ids.flatten()):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size,
                    self.camera_matrix, self.dist_coeffs
                )
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                  rvecs[0].flatten(), tvecs[0].flatten(),
                                  self.marker_size * 0.5)
                position = next((p for p in positions if p.marker_id == marker_id), None)
                if position:
                    corner = corners[i][0]
                    center = np.mean(corner, axis=0).astype(int)
                    color  = (0, 255, 0) if marker_id == 0 else (0, 0, 255)
                    label  = f"ID:{marker_id} D:{position.distance:.2f}m"
                    cv2.putText(frame, label, tuple(center),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        return frame


class UAVVision:
    """Main vision system for UAV"""

    def __init__(self, calibration_file: str = "calibration_chessboard.yaml",
                 marker_size: float = 0.1, use_zed: bool = False):
        self.camera   = CameraInterface(use_zed=use_zed)
        self.detector = ArucoDetector(calibration_file, marker_size)
        self.target_marker_id = 0

    def get_target_position(self) -> Optional[MarkerPosition]:
        frame = self.camera.get_frame()
        if frame is None:
            return None
        positions = self.detector.detect(frame)
        return next((p for p in positions if p.marker_id == self.target_marker_id), None)

    def process_frame(self, display: bool = True) -> Tuple[Optional[List[MarkerPosition]],
                                                            Optional[np.ndarray]]:
        frame = self.camera.get_frame()
        if frame is None:
            return None, None
        positions       = self.detector.detect(frame)
        annotated_frame = self.detector.draw_detections(frame.copy(), positions)
        if display:
            for i, pos in enumerate(positions):
                info = (f"ID {pos.marker_id}: "
                        f"X:{pos.x:.2f} Y:{pos.y:.2f} Z:{pos.z:.2f} D:{pos.distance:.2f}m")
                cv2.putText(annotated_frame, info, (10, 30 + i * 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("UAV Vision", annotated_frame)
        return positions, annotated_frame

    def close(self):
        self.camera.close()


# ═════════════════════════════════════════════════════════════════════════════
# MISSION CONFIG
# ═════════════════════════════════════════════════════════════════════════════

CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE         = 57600
ESP32_PORT        = "/dev/ttyUSB0"

# same calibration path that works in the standalone vision test
CALIBRATION_FILE  = "../CameraCalibration/calibration_chessboard.yaml"
MARKER_SIZE       = 0.1   # meters - must match your printed marker

TARGET_ALT        = 1.3   # hover height in meters

THROTTLE_CLIMB    = 1650
THROTTLE_HOVER    = 1500

# forward flight to cover ~5 meters
# tune FORWARD_FLIGHT_TIME: time(s) * speed(m/s) = distance
# at FORWARD_PITCH_PWM=1580 expect ~0.6-0.8 m/s -> ~7s = ~5m
FORWARD_PITCH_PWM   = 1580
FORWARD_FLIGHT_TIME = 7.0

# centering PID - meter-based using tvec from pose estimation
# tvec x = side offset, tvec y = forward offset (both 0 = drone directly above marker)
# KP maps meters of error to pwm nudge: 0.3m * 300 = 90 pwm nudge
KP_ROLL       = 300    # gain for side (x) correction
KP_PITCH      = 300    # gain for forward (y) correction
MAX_NUDGE     = 150    # max pwm offset from RC_CENTER
METER_DEADBAND = 0.05  # 5cm - ignore error smaller than this

CENTER_HOLD_TIME = 60.0  # seconds to hold centered before land phase
CONFIRM_NEEDED   = 3     # consecutive detections needed to lock onto a marker

RC_CENTER  = 1500
FOLLOW_HZ  = 10

WINDOW_NAME = "UAV Mission 5 - Vision"

# ═════════════════════════════════════════════════════════════════════════════
# MAVLINK HELPERS  (confirmed mission4 pattern, unchanged)
# ═════════════════════════════════════════════════════════════════════════════

def change_mode(master, mode):
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"[FC] Unknown mode '{mode}'")
        return
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode]
    )
    print(f"[FC] Mode: {mode}")
    time.sleep(1)

def arm_drone(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("[FC] Arming...")
    time.sleep(2)

def set_rc_override(master, roll, pitch, throttle, yaw=RC_CENTER):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        roll, pitch, throttle, yaw, 0, 0, 0, 0
    )

def get_lidar_alt(master, blocking=False):
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=blocking, timeout=0.05)
    if msg:
        return msg.current_distance / 100.0
    return 0.0

def throttle_hold(alt):
    if alt < TARGET_ALT - 0.1:
        return THROTTLE_HOVER + 100
    elif alt > TARGET_ALT + 0.1:
        return THROTTLE_HOVER - 100
    return THROTTLE_HOVER

# ═════════════════════════════════════════════════════════════════════════════
# METER-BASED PID  (uses real tvec coords from pose estimation)
# ═════════════════════════════════════════════════════════════════════════════

def compute_correction(pos: MarkerPosition):
    """
    pos.x = side offset in meters   -> roll correction
    pos.y = forward offset in meters -> pitch correction
    Both are 0 when the drone is directly above the marker.
    """
    ex = 0.0 if abs(pos.x) < METER_DEADBAND else pos.x
    ey = 0.0 if abs(pos.y) < METER_DEADBAND else pos.y
    roll_pwm  = int(RC_CENTER + max(-MAX_NUDGE, min(MAX_NUDGE, KP_ROLL  * ex)))
    pitch_pwm = int(RC_CENTER + max(-MAX_NUDGE, min(MAX_NUDGE, KP_PITCH * ey)))
    return roll_pwm, pitch_pwm

def is_centered(pos: MarkerPosition, deadband=METER_DEADBAND) -> bool:
    return abs(pos.x) <= deadband and abs(pos.y) <= deadband

# ═════════════════════════════════════════════════════════════════════════════
# STATUS BAR  (drawn on top of UAVVision's own detection annotations)
# ═════════════════════════════════════════════════════════════════════════════

def add_status_bar(frame, text):
    if frame is None:
        return frame
    out = frame.copy()
    h, w = out.shape[:2]
    cv2.rectangle(out, (0, 0), (w, 34), (0, 0, 0), -1)
    cv2.putText(out, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    return out

# ═════════════════════════════════════════════════════════════════════════════
# MAIN MISSION
# ═════════════════════════════════════════════════════════════════════════════

def main():
    print("==========================================")
    print("   UAV MISSION 5 - VISION GUIDED")
    print("   Takeoff -> 5m Forward -> Center ArUco (60s) -> Land")
    print("==========================================")

    print(f"[FC] Connecting: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    print("[FC] Heartbeat OK.")

    import v2v_bridge
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        bridge.send_message("MISSION 5: START")
    except Exception:
        print("[Bridge] Radio bridge failed.")
        return

    # ── init vision using the EXACT same call as the working standalone test ──
    print("[Vision] Starting UAVVision...")
    try:
        vision = UAVVision(
            calibration_file=CALIBRATION_FILE,
            marker_size=MARKER_SIZE,
            use_zed=True   # ZED 2 on Jetson Nano
        )
    except Exception as e:
        print(f"[!] Vision init failed: {e}")
        bridge.stop()
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    loop_dt = 1.0 / FOLLOW_HZ

    try:
        ##################################################################
        # PHASE 1: TAKEOFF
        ##################################################################
        print("\n--- PHASE 1: TAKEOFF ---")
        change_mode(master, "STABILIZE")
        arm_drone(master)
        bridge.send_message("PHASE 1: TAKEOFF")

        while True:
            alt = get_lidar_alt(master, blocking=True)
            print(f" Alt: {alt:.2f}m", end='\r')

            # process_frame(display=False) - same call as standalone test,
            # just suppressing its own imshow so we control the window ourselves
            positions, annotated = vision.process_frame(display=False)
            if annotated is not None:
                cv2.imshow(WINDOW_NAME,
                           add_status_bar(annotated,
                                          f"TAKEOFF  Alt:{alt:.2f}m  Target:{TARGET_ALT}m"))
                cv2.waitKey(1)

            if alt >= TARGET_ALT:
                set_rc_override(master, RC_CENTER, RC_CENTER, THROTTLE_HOVER)
                print(f"\n[FC] Hover reached: {alt:.2f}m")
                break

            set_rc_override(master, RC_CENTER, RC_CENTER, THROTTLE_CLIMB)
            time.sleep(0.1)

        ##################################################################
        # PHASE 2: FLY FORWARD ~5 METERS
        ##################################################################
        print(f"\n--- PHASE 2: FORWARD FLIGHT ({FORWARD_FLIGHT_TIME}s) ---")
        bridge.send_message("PHASE 2: FLYING FORWARD")

        fwd_start = time.time()
        while (time.time() - fwd_start) < FORWARD_FLIGHT_TIME:
            elapsed   = time.time() - fwd_start
            remaining = FORWARD_FLIGHT_TIME - elapsed

            alt = get_lidar_alt(master)
            set_rc_override(master, RC_CENTER, FORWARD_PITCH_PWM, throttle_hold(alt))

            positions, annotated = vision.process_frame(display=False)
            if annotated is not None:
                cv2.imshow(WINDOW_NAME,
                           add_status_bar(annotated,
                                          f"FORWARD  {elapsed:.1f}s/{FORWARD_FLIGHT_TIME}s  "
                                          f"left:{remaining:.1f}s  Alt:{alt:.2f}m"))
                cv2.waitKey(1)

            print(f" Flying forward {elapsed:.1f}s  Alt:{alt:.2f}m", end='\r')
            time.sleep(0.1)

        alt = get_lidar_alt(master)
        set_rc_override(master, RC_CENTER, RC_CENTER, throttle_hold(alt))
        print(f"\n[FC] Forward flight complete. Hovering.")

        ##################################################################
        # PHASE 3: ACQUIRE MARKER + CENTER FOR 60 SECONDS
        ##################################################################
        print(f"\n--- PHASE 3: ACQUIRE + CENTER ({CENTER_HOLD_TIME}s) ---")
        bridge.send_message("PHASE 3: SEARCHING FOR MARKER")

        target_id     = None
        confirm_count = 0
        center_timer  = None

        while True:
            loop_start = time.time()

            alt = get_lidar_alt(master)
            thr = throttle_hold(alt)

            positions, annotated = vision.process_frame(display=False)

            if positions is None:
                set_rc_override(master, RC_CENTER, RC_CENTER, thr)
                time.sleep(0.05)
                continue

            if target_id is None:
                # acquisition mode
                if positions:
                    confirm_count += 1
                    candidate = positions[0].marker_id
                    if confirm_count >= CONFIRM_NEEDED:
                        target_id    = candidate
                        center_timer = time.time()
                        print(f"\n[ArUco] Locked ID:{target_id}. Centering for {CENTER_HOLD_TIME}s...")
                        bridge.send_message(f"MARKER LOCKED: ID {target_id}")
                    status = (f"CONFIRMING ID:{candidate}  "
                              f"{confirm_count}/{CONFIRM_NEEDED}  Alt:{alt:.2f}m")
                else:
                    confirm_count = 0
                    status = f"SEARCHING  Alt:{alt:.2f}m"
                set_rc_override(master, RC_CENTER, RC_CENTER, thr)

            else:
                # locked - find our target
                pos = next((p for p in positions if p.marker_id == target_id), None)

                if pos:
                    roll_pwm, pitch_pwm = compute_correction(pos)
                    set_rc_override(master, roll_pwm, pitch_pwm, thr)

                    time_held = time.time() - center_timer
                    time_left = CENTER_HOLD_TIME - time_held

                    print(
                        f" [Center] ID:{target_id}  x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                        f"Roll:{roll_pwm} Pitch:{pitch_pwm}  "
                        f"held:{time_held:.1f}s left:{time_left:.1f}s  Alt:{alt:.2f}m",
                        end='\r'
                    )
                    status = (f"CENTERING ID:{target_id}  x:{pos.x:.2f}m y:{pos.y:.2f}m  "
                              f"held:{time_held:.1f}s left:{time_left:.1f}s  Alt:{alt:.2f}m")

                    if time_held >= CENTER_HOLD_TIME:
                        print(f"\n[ArUco] {CENTER_HOLD_TIME}s complete. Moving to land phase.")
                        bridge.send_message("CENTERING COMPLETE: PREPARING LAND")
                        if annotated is not None:
                            cv2.imshow(WINDOW_NAME, add_status_bar(annotated, status))
                            cv2.waitKey(1)
                        break
                else:
                    set_rc_override(master, RC_CENTER, RC_CENTER, thr)
                    status = f"MARKER LOST - HOLDING  ID:{target_id}  Alt:{alt:.2f}m"

            if annotated is not None:
                cv2.imshow(WINDOW_NAME, add_status_bar(annotated, status))
                cv2.waitKey(1)

            sleep_t = loop_dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        ##################################################################
        # PHASE 4: RE-CENTER THEN LAND
        ##################################################################
        print(f"\n--- PHASE 4: FINAL CENTER BEFORE LAND ---")
        bridge.send_message("PHASE 4: FINAL CENTERING")

        LAND_DEADBAND = METER_DEADBAND * 1.5  # 7.5cm, slightly relaxed for land gate

        while True:
            loop_start = time.time()

            alt = get_lidar_alt(master)
            thr = throttle_hold(alt)

            positions, annotated = vision.process_frame(display=False)

            if positions is not None:
                pos = next((p for p in positions if p.marker_id == target_id), None)
                if pos:
                    centered = is_centered(pos, deadband=LAND_DEADBAND)
                    if centered:
                        set_rc_override(master, RC_CENTER, RC_CENTER, thr)
                        status = (f"CENTERED  x:{pos.x:.3f}m y:{pos.y:.3f}m -> LANDING")
                        if annotated is not None:
                            cv2.imshow(WINDOW_NAME, add_status_bar(annotated, status))
                            cv2.waitKey(500)
                        print(f"\n[ArUco] Centered. Landing!")
                        bridge.send_message("CENTERED: LANDING NOW")
                        break
                    else:
                        roll_pwm, pitch_pwm = compute_correction(pos)
                        set_rc_override(master, roll_pwm, pitch_pwm, thr)
                        status = (f"FINAL CENTER  x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                                  f"need<{LAND_DEADBAND:.2f}m  Alt:{alt:.2f}m")
                        print(f" [LandGate] x:{pos.x:.3f}m y:{pos.y:.3f}m "
                              f"need<{LAND_DEADBAND:.2f}m", end='\r')
                else:
                    set_rc_override(master, RC_CENTER, RC_CENTER, thr)
                    status = f"FINAL CENTER - TARGET LOST  Alt:{alt:.2f}m"
            else:
                set_rc_override(master, RC_CENTER, RC_CENTER, thr)
                status = f"FINAL CENTER - NO FRAME  Alt:{alt:.2f}m"

            if annotated is not None:
                cv2.imshow(WINDOW_NAME, add_status_bar(annotated, status))
                cv2.waitKey(1)

            sleep_t = loop_dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        ##################################################################
        # LAND
        ##################################################################
        print("\n[FC] Landing...")
        change_mode(master, "LAND")
        set_rc_override(master, RC_CENTER, RC_CENTER, 0, RC_CENTER)

        while True:
            alt = get_lidar_alt(master)
            print(f" Land Alt: {alt:.2f}m", end='\r')

            positions, annotated = vision.process_frame(display=False)
            if annotated is not None:
                cv2.imshow(WINDOW_NAME,
                           add_status_bar(annotated, f"LANDING  Alt:{alt:.2f}m"))
                cv2.waitKey(1)

            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("\n[FC] Touchdown confirmed. Motors stopped.")
                break
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[!] EMERGENCY LAND")
        change_mode(master, "LAND")
        set_rc_override(master, RC_CENTER, RC_CENTER, 0, RC_CENTER)
        time.sleep(1)

    finally:
        vision.close()   # same close() call as the standalone test
        bridge.stop()
        print("Mission 5 finalized.")


if __name__ == "__main__":
    main()

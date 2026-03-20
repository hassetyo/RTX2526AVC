"""
UAV Mission 5 - Vision Guided ArUco Follow merged with UGV Control
===================================================================
Flow:
  Phase 1 — Arm + climb to TARGET_ALT
  Phase 2 — Fly forward ~5 m (timed pitch override)
  Phase 3 — Scan for ArUco marker, then centre on it for 20 seconds
             Corrections use real tvec metres from pose estimation
             Additionally, commands the UGV via radio to steer if the marker drifts
  Phase 4 — Final re-centre tightened to LAND_DEADBAND, then switch to LAND mode
  Land    — Wait for autopilot to confirm disarm

Fully self-contained — no external vision module required.
Camera settings (HD1080, DEPTH_MODE.NONE, exposure=1) are defined here.
"""

# ── standard imports ──────────────────────────────────────────────────────────
from pymavlink import mavutil
import time
import cv2
import cv2.aruco as aruco
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List
import v2v_bridge


# =============================================================================
# TUNABLE MISSION CONFIG
# =============================================================================

# --- MAVLink ---
CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE         = 57600
ESP32_PORT        = "/dev/ttyUSB0"

# --- Calibration / vision ---
CALIBRATION_FILE = "calibration_chessboard.yaml"
MARKER_SIZE      = 0.1          # metres -- must match your printed marker

# --- Camera ---
ZED_RESOLUTION  = "HD1080"      # HD2K | HD1080 | HD720 | VGA
ZED_FPS         = 60             # 0 = SDK default for chosen resolution
ZED_EXPOSURE    = 50             # 1-100; low value = sharp fast-moving markers
ZONE_BOX_WIDTH  = 200           # pixels -- acceptance box width
ZONE_BOX_HEIGHT = 200           # pixels -- acceptance box height

# --- Flight ---
TARGET_ALT     = 0.0            # hover height in metres
THROTTLE_CLIMB = 1650           # PWM to climb
THROTTLE_HOVER = 1500           # PWM neutral hover
ALT_BAND       = 0.1            # +-m band before altitude correction fires
ALT_BOOST      = 100            # PWM delta applied when outside ALT_BAND

# --- Forward flight phase ---
FORWARD_PITCH_PWM   = 1580
FORWARD_FLIGHT_TIME = 7.0       # seconds

# --- Marker acquisition ---
CONFIRM_NEEDED = 3              # consecutive detections before locking onto a marker

# --- Centering (metre-based, uses tvec from pose estimation) ---
KP_ROLL        = 300            # gain:  0.3 m * 300 = 90 PWM nudge
KP_PITCH       = 300
MAX_NUDGE      = 150            # hard cap on PWM offset from RC_CENTER
METER_DEADBAND = 0.15           # 15 cm -- used as a rough fallback

# --- Cooldown between correction commands ---
CORRECTION_COOLDOWN = 0.8       # seconds

# --- Hold timer ---
CENTER_HOLD_TIME = 20.0         # seconds to stay centred before land phase (MODIFIED)
LAND_DEADBAND    = 0.075        # 7.5 cm -- tighter gate used just before landing

# --- Loop rate ---
FOLLOW_HZ = 10                  # Hz for the centering loop

# --- UGV Radio ---
UGV_COOLDOWN = 6.0              # seconds between UGV commands to prevent spam

# --- Display ---
WINDOW_NAME = "UAV Mission ArUco - Vision Guided"

# --- RC neutral ---
RC_CENTER = 1500

# --- Log file ---
LOG_FILE = "missionaruco.log"


# =============================================================================
# VISION -- data types
# =============================================================================

@dataclass
class MarkerPosition:
    """Position of a detected ArUco marker in camera-frame metres."""
    marker_id: int
    x: float        # side distance  (right = positive)
    y: float        # forward distance (forward = positive in camera frame)
    z: float        # height distance
    distance: float # total Euclidean distance
    detected: bool = True


# =============================================================================
# VISION -- CenterZone
# =============================================================================

class CenterZone:
    """Rectangular acceptance box around the frame centre."""

    def __init__(self, box_width: int = ZONE_BOX_WIDTH,
                 box_height: int = ZONE_BOX_HEIGHT):
        self.box_width  = box_width
        self.box_height = box_height

    def resize(self, box_width: int, box_height: int):
        self.box_width  = box_width
        self.box_height = box_height

    def bounds(self, frame_w: int, frame_h: int) -> Tuple[int, int, int, int]:
        cx, cy = frame_w / 2.0, frame_h / 2.0
        return (
            int(cx - self.box_width  / 2.0),
            int(cy - self.box_height / 2.0),
            int(cx + self.box_width  / 2.0),
            int(cy + self.box_height / 2.0),
        )

    def contains(self, dx: float, dy: float) -> bool:
        return (abs(dx) <= self.box_width  / 2.0 and
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


# =============================================================================
# VISION -- CameraInterface
# =============================================================================

class CameraInterface:
    """Handles camera initialisation and frame capture."""

    def __init__(self, use_zed: bool = False, camera_index: int = 0):
        self.use_zed      = use_zed
        self.camera_index = camera_index
        self.cap          = None
        self.zed          = None
        self._sl          = None
        print(f"Initializing {'ZED' if use_zed else 'Standard'} camera...")

        if use_zed:
            self._initialize_zed()
        else:
            self._initialize_standard()

        print("Camera initialized successfully.")

    def _initialize_zed(self):
        import pyzed.sl as sl
        self._sl = sl

        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        res_map = {
            "HD2K":   sl.RESOLUTION.HD2K,
            "HD1080": sl.RESOLUTION.HD1080,
            "HD720":  sl.RESOLUTION.HD720,
            "VGA":    sl.RESOLUTION.VGA,
        }
        init_params.camera_resolution = res_map.get(ZED_RESOLUTION,
                                                     sl.RESOLUTION.HD1080)
        init_params.camera_fps = ZED_FPS
        init_params.depth_mode = sl.DEPTH_MODE.NONE

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED camera error: {status}")

        # Exposure must be set AFTER open()
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, ZED_EXPOSURE)
        print(f"ZED opened -- {ZED_RESOLUTION} @ fps={ZED_FPS} exposure={ZED_EXPOSURE}")

    def _initialize_standard(self):
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open standard camera.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)

    def get_frame(self) -> Optional[np.ndarray]:
        return self._get_zed_frame() if self.use_zed else self._get_standard_frame()

    def _get_zed_frame(self) -> Optional[np.ndarray]:
        sl = self._sl
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return None
        image = sl.Mat()
        self.zed.retrieve_image(image, sl.VIEW.LEFT)
        return cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)

    def _get_standard_frame(self) -> Optional[np.ndarray]:
        ret, frame = self.cap.read()
        return frame if ret else None

    def close(self):
        if self.use_zed and self.zed:
            self.zed.close()
        elif self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


# =============================================================================
# VISION -- ArucoDetector
# =============================================================================

class ArucoDetector:
    """ArUco marker detection and pose estimation."""

    def __init__(self, calibration_file: str, marker_size: float = 0.1,
                 dictionary=aruco.DICT_6X6_1000):
        self.marker_size = marker_size
        self.aruco_dict  = aruco.getPredefinedDictionary(dictionary)

        fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("K").mat()
        self.dist_coeffs   = fs.getNode("D").mat()
        fs.release()

        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError(f"Invalid calibration file: {calibration_file}")

        print(f"Calibration loaded: {calibration_file} | "
              f"marker_size={marker_size}m | DICT_6X6_1000")

    def detect(self, frame: np.ndarray) -> List[MarkerPosition]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)

        positions = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size,
                    self.camera_matrix, self.dist_coeffs,
                )
                tvec = tvecs[0].flatten()
                positions.append(MarkerPosition(
                    marker_id=int(marker_id),
                    x=float(tvec[0]),
                    y=float(tvec[1]),
                    z=float(tvec[2]),
                    distance=float(np.linalg.norm(tvec)),
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
                    self.camera_matrix, self.dist_coeffs,
                )
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                  rvecs[0].flatten(), tvecs[0].flatten(),
                                  self.marker_size * 0.5)
                pos = next((p for p in positions
                            if p.marker_id == marker_id), None)
                if pos:
                    center = np.mean(corners[i][0], axis=0).astype(int)
                    color  = (0, 255, 0) if marker_id == 0 else (0, 0, 255)
                    cv2.putText(frame,
                                f"ID:{marker_id} D:{pos.distance:.2f}m",
                                tuple(center),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color,
                                2, cv2.LINE_AA)
        return frame


# =============================================================================
# VISION -- UAVVision
# =============================================================================

class UAVVision:
    """Main vision system -- self-contained, no external module needed."""

    def __init__(self, calibration_file: str = CALIBRATION_FILE,
                 marker_size: float = MARKER_SIZE,
                 use_zed: bool = True,
                 zone_box_width: int  = ZONE_BOX_WIDTH,
                 zone_box_height: int = ZONE_BOX_HEIGHT):
        self.camera           = CameraInterface(use_zed=use_zed)
        self.detector         = ArucoDetector(calibration_file, marker_size)
        self.target_marker_id = 0
        self.center_zone      = CenterZone(zone_box_width, zone_box_height)

    def process_frame(self, display: bool = False) \
            -> Tuple[Optional[List[MarkerPosition]],
                     Optional[np.ndarray],
                     Optional[np.ndarray],
                     Optional[np.ndarray]]:
        """
        Grab a frame, detect markers, annotate, and return results.

        Returns:
            (positions, annotated_frame, corners, ids)
            All fields are None if the frame grab failed.
        """
        frame = self.camera.get_frame()
        if frame is None:
            return None, None, None, None

        positions = self.detector.detect(frame)
        annotated = self.detector.draw_detections(frame.copy(), positions)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.detector.aruco_dict)

        frame_h, frame_w = frame.shape[:2]
        cx, cy           = frame_w / 2.0, frame_h / 2.0
        target_in_zone   = False

        if ids is not None:
            target_idx = next(
                (i for i, mid in enumerate(ids.flatten())
                 if mid == self.target_marker_id), None
            )
            if target_idx is not None:
                mc = np.mean(corners[target_idx][0], axis=0)
                target_in_zone = self.center_zone.contains(
                    mc[0] - cx, mc[1] - cy
                )

        if display:
            self.center_zone.draw(annotated, target_in_zone)
            cv2.drawMarker(annotated, (int(cx), int(cy)), (255, 255, 255),
                           cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
            for i, pos in enumerate(positions):
                cv2.putText(annotated,
                            f"ID {pos.marker_id}: X:{pos.x:.2f} Y:{pos.y:.2f} "
                            f"Z:{pos.z:.2f} D:{pos.distance:.2f}m",
                            (10, 30 + i * 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("UAV Vision", annotated)

        return positions, annotated, corners, ids

    def close(self):
        self.camera.close()


# =============================================================================
# LOGGING
# =============================================================================

def log(text: str):
    ts   = time.strftime("%H:%M:%S")
    line = f"[{ts}] {text}"
    print(line)
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(line + "\n")


# =============================================================================
# MAVLINK HELPERS
# =============================================================================

def change_mode(master, mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        log(f"[FC] Unknown mode '{mode}'")
        return
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode],
    )
    log(f"[FC] Mode: {mode}")
    time.sleep(1)


def arm_drone(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    log("[FC] Arming motors...")
    time.sleep(2)


def set_rc_override(master, roll=RC_CENTER, pitch=RC_CENTER,
                    throttle=THROTTLE_HOVER, yaw=RC_CENTER):
    """Send RC channel overrides CH1-4. All channels always sent explicitly."""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        roll, pitch, throttle, yaw,
        0, 0, 0, 0,
    )


def get_lidar_alt(master, blocking: bool = False) -> float:
    msg = master.recv_match(type="DISTANCE_SENSOR",
                            blocking=blocking, timeout=0.05)
    if msg:
        return msg.current_distance / 100.0
    return 0.0


def throttle_hold(alt: float) -> int:
    """Bang-bang altitude hold around TARGET_ALT."""
    if alt < TARGET_ALT - ALT_BAND:
        return THROTTLE_HOVER + ALT_BOOST
    if alt > TARGET_ALT + ALT_BAND:
        return THROTTLE_HOVER - ALT_BOOST
    return THROTTLE_HOVER


# =============================================================================
# CENTERING HELPERS
# =============================================================================

def compute_correction(pos: MarkerPosition) -> Tuple[int, int]:
    """
    Convert tvec metres into roll/pitch PWM values.
    """
    # Now that we use the visual pixel box for holding deadband, we can just purely rely on pos.x and pos.y
    roll_pwm  = int(RC_CENTER + max(-MAX_NUDGE, min(MAX_NUDGE,  KP_ROLL  * pos.x)))
    pitch_pwm = int(RC_CENTER + max(-MAX_NUDGE, min(MAX_NUDGE, -KP_PITCH * pos.y)))
    return roll_pwm, pitch_pwm


def is_centered(pos: MarkerPosition, deadband: float = METER_DEADBAND) -> bool:
    return abs(pos.x) <= deadband and abs(pos.y) <= deadband


# =============================================================================
# DISPLAY HELPER
# =============================================================================

def show(frame, status: str, vision: UAVVision, target_in_zone: bool):
    """Overlay status bar + centre-zone box and push to window."""
    if frame is None:
        return

    out = frame.copy()
    vision.center_zone.draw(out, target_in_zone)

    h, w = out.shape[:2]
    cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 255),
                   cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)

    cv2.rectangle(out, (0, 0), (w, 36), (0, 0, 0), -1)
    cv2.putText(out, status, (8, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow(WINDOW_NAME, out)
    cv2.waitKey(1)


# =============================================================================
# MAIN MISSION
# =============================================================================

def main():
    log("==========================================")
    log("   UAV MISSION ArUco - VISION GUIDED")
    log("   Takeoff -> Forward -> Centre (20s) -> Land")
    log("==========================================")

    # Connect to flight controller
    log(f"[FC] Connecting: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    log("[FC] Heartbeat OK.")

    # ── Connect to V2V Bridge for UGV Control ────────────────────────────────
    log(f"[Bridge] Starting V2V Bridge on {ESP32_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        bridge.send_message("MISSION ArUco: AWAITING UGV")
    except Exception as e:
        log(f"[!] Radio Bridge Fail: {e}")
        return

    # Init vision
    log("[Vision] Starting UAVVision with ZED camera...")
    try:
        vision = UAVVision(
            calibration_file=CALIBRATION_FILE,
            marker_size=MARKER_SIZE,
            use_zed=True,
            zone_box_width=ZONE_BOX_WIDTH,
            zone_box_height=ZONE_BOX_HEIGHT,
        )
    except Exception as e:
        log(f"[!] Vision init failed: {e}")
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    loop_dt = 1.0 / FOLLOW_HZ

    cmd_seq = 500

    try:
        ##################################################################
        # PHASE 1: ARM + CLIMB
        ##################################################################
        log("\n--- PHASE 1: TAKEOFF ---")
        change_mode(master, "STABILIZE")
        arm_drone(master)

        log("\n[Bridge] FORCE ARMING Ground Vehicle AND moving it immediately (Mission 1)!")
        bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_MISSION_1, estop=0)
        cmd_seq += 1

        while True:
            alt = get_lidar_alt(master, blocking=True)
            print(f"\r  Alt: {alt:.2f}m / {TARGET_ALT}m", end="", flush=True)

            positions, annotated, _, _ = vision.process_frame(display=False)
            show(annotated, f"TAKEOFF  Alt:{alt:.2f}m  Target:{TARGET_ALT}m",
                 vision, False)

            if alt >= TARGET_ALT:
                set_rc_override(master, throttle=THROTTLE_HOVER)
                log(f"\n[FC] Hover altitude reached: {alt:.2f}m")
                break

            set_rc_override(master, throttle=THROTTLE_CLIMB)
            time.sleep(0.1)

        ##################################################################
        # PHASE 2: FLY FORWARD ~5 METRES
        ##################################################################
        log(f"\n--- PHASE 2: FORWARD FLIGHT ({FORWARD_FLIGHT_TIME}s) ---")

        fwd_start = time.time()
        while (time.time() - fwd_start) < FORWARD_FLIGHT_TIME:
            elapsed   = time.time() - fwd_start
            remaining = FORWARD_FLIGHT_TIME - elapsed

            alt = get_lidar_alt(master)
            set_rc_override(master, pitch=FORWARD_PITCH_PWM,
                            throttle=throttle_hold(alt))

            positions, annotated, _, _ = vision.process_frame(display=False)
            show(annotated,
                 f"FORWARD  {elapsed:.1f}s / {FORWARD_FLIGHT_TIME}s  "
                 f"remaining:{remaining:.1f}s  Alt:{alt:.2f}m",
                 vision, False)

            print(f"\r  Flying {elapsed:.1f}s  Alt:{alt:.2f}m",
                  end="", flush=True)
            time.sleep(0.1)

        alt = get_lidar_alt(master)
        set_rc_override(master, throttle=throttle_hold(alt))
        log("\n[FC] Forward flight complete. Hovering.")

        ##################################################################
        # PHASE 3: ACQUIRE MARKER + CENTRE FOR 20 SECONDS
        ##################################################################
        log(f"\n--- PHASE 3: ACQUIRE + CENTRE ({CENTER_HOLD_TIME}s) ---")

        target_id            = None
        confirm_count        = 0
        center_timer         = None
        last_correction_time = 0.0
        last_ugv_cmd_time    = 0.0

        while True:
            loop_start = time.time()

            alt = get_lidar_alt(master)
            thr = throttle_hold(alt)

            positions, annotated, corners, ids = vision.process_frame(display=False)

            if positions is None:
                set_rc_override(master, throttle=thr)
                time.sleep(0.05)
                continue

            if target_id is None:
                # Acquisition mode
                if positions:
                    confirm_count += 1
                    candidate = positions[0].marker_id
                    if confirm_count >= CONFIRM_NEEDED:
                        target_id    = candidate
                        center_timer = time.time()
                        log(f"\n[ArUco] Locked on ID:{target_id}. "
                            f"Centering for {CENTER_HOLD_TIME}s...")
                    status = (f"CONFIRMING ID:{candidate}  "
                              f"{confirm_count}/{CONFIRM_NEEDED}  Alt:{alt:.2f}m")
                else:
                    confirm_count = 0
                    status = f"SEARCHING for marker...  Alt:{alt:.2f}m"

                set_rc_override(master, throttle=thr)
                show(annotated, status, vision, False)

            else:
                # Centring mode
                pos = next((p for p in positions
                            if p.marker_id == target_id), None)

                if pos:
                    target_idx = next((i for i, mid in enumerate(ids.flatten()) if mid == target_id), None)
                    in_zone = False
                    if target_idx is not None:
                        mc = np.mean(corners[target_idx][0], axis=0)
                        # Pixel-based zone check overrides math deadbands so the green box behaves flawlessly
                        in_zone = vision.center_zone.contains(mc[0] - (annotated.shape[1]/2.0), mc[1] - (annotated.shape[0]/2.0))

                    time_held = time.time() - center_timer
                    time_left = CENTER_HOLD_TIME - time_held

                    now = time.time()
                    if not in_zone and (now - last_correction_time) >= CORRECTION_COOLDOWN:
                        roll_pwm, pitch_pwm = compute_correction(pos)
                        set_rc_override(master, roll=roll_pwm,
                                        pitch=pitch_pwm, throttle=thr)
                        log(f"[Centre] x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                            f"roll:{roll_pwm} pitch:{pitch_pwm}  "
                            f"held:{time_held:.1f}s left:{time_left:.1f}s")
                        last_correction_time = now

                    # --- FUN COMMAND TO UGV ---
                    # Use a separate 6.0s cooldown so we don't spam the UGV while it's turning
                    if not in_zone and (now - last_ugv_cmd_time) >= UGV_COOLDOWN:
                        if pos.x < -0.05:
                            log(">>> Radioing UGV: TURN LEFT!")
                            bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_TURN_LEFT, estop=0)
                            cmd_seq += 1
                            last_ugv_cmd_time = now
                        elif pos.x > 0.05:
                            log(">>> Radioing UGV: TURN RIGHT!")
                            bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_TURN_RIGHT, estop=0)
                            cmd_seq += 1
                            last_ugv_cmd_time = now

                    if in_zone:
                        set_rc_override(master, throttle=thr)

                    status = (f"CENTRING ID:{target_id}  "
                              f"x:{pos.x:.2f}m y:{pos.y:.2f}m  "
                              f"held:{time_held:.1f}s  left:{time_left:.1f}s  "
                              f"Alt:{alt:.2f}m")
                    show(annotated, status, vision, in_zone)
                    print(f"\r  {status}", end="", flush=True)

                    if time_held >= CENTER_HOLD_TIME:
                        log(f"\n[ArUco] {CENTER_HOLD_TIME}s centring complete.")
                        break

                else:
                    set_rc_override(master, throttle=thr)
                    status = f"MARKER LOST - holding  ID:{target_id}  Alt:{alt:.2f}m"
                    show(annotated, status, vision, False)

            sleep_t = loop_dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        ##################################################################
        # PHASE 4: FINAL RE-CENTRE BEFORE LAND
        ##################################################################
        log(f"\n--- PHASE 4: FINAL CENTRE (deadband={LAND_DEADBAND*100:.1f}cm) ---")

        last_correction_time = 0.0

        while True:
            loop_start = time.time()

            alt = get_lidar_alt(master)
            thr = throttle_hold(alt)

            positions, annotated, _, _ = vision.process_frame(display=False)

            if positions is not None:
                pos = next((p for p in positions
                            if p.marker_id == target_id), None)
                if pos:
                    target_idx = next((i for i, mid in enumerate(ids.flatten()) if mid == target_id), None)
                    centered = False
                    if target_idx is not None:
                        mc = np.mean(corners[target_idx][0], axis=0)
                        # Re-use pixel-based zone check to land beautifully
                        centered = vision.center_zone.contains(mc[0] - (annotated.shape[1]/2.0), mc[1] - (annotated.shape[0]/2.0))

                    if centered:
                        set_rc_override(master, throttle=thr)
                        show(annotated,
                             f"CENTRED  x:{pos.x:.3f}m y:{pos.y:.3f}m  -> LANDING",
                             vision, True)
                        log("[ArUco] Centred within land gate. Initiating land.")
                        break
                    else:
                        now = time.time()
                        if (now - last_correction_time) >= CORRECTION_COOLDOWN:
                            roll_pwm, pitch_pwm = compute_correction(pos)
                            set_rc_override(master, roll=roll_pwm,
                                            pitch=pitch_pwm, throttle=thr)
                            log(f"[LandGate] x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                                f"need<{LAND_DEADBAND:.3f}m  "
                                f"roll:{roll_pwm} pitch:{pitch_pwm}")
                            last_correction_time = now
                        else:
                            set_rc_override(master, throttle=thr)

                        show(annotated,
                             f"FINAL CENTRE  x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                             f"need<{LAND_DEADBAND*100:.1f}cm  Alt:{alt:.2f}m",
                             vision, False)
                else:
                    set_rc_override(master, throttle=thr)
                    show(annotated,
                         f"FINAL CENTRE - target lost  Alt:{alt:.2f}m",
                         vision, False)
            else:
                set_rc_override(master, throttle=thr)
                show(None, f"FINAL CENTRE - no frame  Alt:{alt:.2f}m",
                     vision, False)

            sleep_t = loop_dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        ##################################################################
        # LAND
        ##################################################################
        log("\n[FC] Switching to LAND mode...")
        change_mode(master, "LAND")
        # PWM 0 = release override so autopilot handles landing cleanly
        set_rc_override(master, roll=RC_CENTER, pitch=RC_CENTER,
                        throttle=0, yaw=RC_CENTER)

        log("[FC] Waiting for touchdown...")
        while True:
            alt = get_lidar_alt(master)
            print(f"\r  Land alt: {alt:.2f}m", end="", flush=True)

            positions, annotated, _, _ = vision.process_frame(display=False)
            show(annotated, f"LANDING  Alt:{alt:.2f}m", vision, False)

            msg = master.recv_match(type="HEARTBEAT", blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                log("\n[FC] Touchdown confirmed. Motors stopped.")
                break
            time.sleep(0.5)

    except KeyboardInterrupt:
        log("\n[!] Emergency land triggered by user.")
        change_mode(master, "LAND")
        set_rc_override(master, roll=RC_CENTER, pitch=RC_CENTER,
                        throttle=0, yaw=RC_CENTER)
        time.sleep(1)

    finally:
        vision.close()
        bridge.stop()
        log("Mission 5 finalized.")


if __name__ == "__main__":
    main()

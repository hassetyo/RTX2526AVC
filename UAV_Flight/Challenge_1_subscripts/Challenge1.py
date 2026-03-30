"""
UAV Challenge 1 - Autonomous Launch, Track, and Land on Moving UGV
===================================================================
Rules (Raytheon AVC '2526 Operation Touchdown - Challenge 1):
  - UAV autonomously takes off from the UGV.
  - After launch, UGV begins moving forward at >= 0.2 mph in a straight line.
  - UAV must fly >= 5 seconds at >= 4 feet altitude (1.22 m).
  - UAV must autonomously find and land on the moving UGV (ArUco marker) within 7 minutes.
  - Once landed, the UGV continues traveling for 30 seconds before stopping.
  - Time stops once the system has completed the 30-second post-landing travel.

Flow:
  Phase 1 - Arm + climb to TARGET_ALT (must be >= 4 ft / 1.22 m)
  Phase 2 - Enforce minimum 5-second flight time at altitude while scanning
  Phase 3 - Search for ArUco marker on top of the moving UGV
  Phase 4 - Center above marker using rc roll/pitch corrections
  Phase 5 - Final tight re-center, then descend and land
  Phase 6 - Wait 30 seconds after touchdown while UGV continues, then stop UGV

Hardware:
  - Jetson Nano (UAV)
  - ZED 2 stereo camera (downward-facing for ArUco detection)
  - ArduPilot flight controller via MAVLink (/dev/ttyACM0)
  - ESP32 V2V radio bridge (/dev/ttyUSB0) for UGV commands

Notes:
  - Calibration file must be present and valid (calibration_chessboard.yaml)
  - ArUco marker on UGV must be DICT_6X6_1000, ID 0-4, 1ft x 1ft (0.3048 m)
  - Do NOT remove comments or change existing functionality patterns
"""

# ── standard imports ───────────────────────────────────────────────────────────
from pymavlink import mavutil  # confirmed mavlink pattern for flight controller
import time                    # for timing and sleeps
import cv2                     # for aruco detection
import cv2.aruco as aruco      # aruco sub-module
import numpy as np             # for frame math
from dataclasses import dataclass  # clean data containers
from typing import Optional, Tuple, List  # type hints
import v2v_bridge              # our custom radio bridge for ugv commands


# =============================================================================
# TUNABLE MISSION CONFIG
# =============================================================================

# --- MAVLink connection ---
CONNECTION_STRING = "/dev/ttyACM0"   # flight controller USB wire
BAUD_RATE         = 57600            # confirmed baud rate

# --- ESP32 V2V radio ---
ESP32_PORT        = "/dev/ttyUSB0"   # radio bridge USB wire

# --- Calibration / vision ---
CALIBRATION_FILE = "calibration_chessboard.yaml"  # camera cal file for pose estimation
MARKER_SIZE      = 0.3048            # metres -- 1 ft marker as per rules spec

# --- Camera (ZED 2) ---
ZED_RESOLUTION  = "HD720"            # HD720 is easier on the Jetson Nano
ZED_FPS         = 30                 # 30 fps
ZED_EXPOSURE    = 50                 # 1-100; moderate exposure for moving marker
ZONE_BOX_WIDTH  = 180                # pixels -- acceptance box width for centering
ZONE_BOX_HEIGHT = 180                # pixels -- acceptance box height for centering

# --- Flight altitudes ---
TARGET_ALT      = 1.4                # hover height in metres (~4.6 ft, safely above 4 ft rule min)
MIN_ALT_RULE    = 1.22               # minimum flight altitude per rules (4 feet)
ALT_BAND        = 0.10               # +- metres before altitude correction fires
ALT_BOOST       = 80                 # PWM delta when outside the altitude band

# --- Throttle settings (tuned values from working scripts) ---
THROTTLE_MIN    = 1000               # motors off
THROTTLE_IDLE   = 1150               # props spinning but not lifting
THROTTLE_CLIMB  = 1650               # power to lift off
THROTTLE_HOVER  = 1500               # neutral hover midpoint

# --- RC neutral ---
RC_CENTER       = 1500               # neutral PWM for roll/pitch/yaw

# --- Minimum flight time before centering/landing allowed (rules: >= 5s) ---
MIN_FLIGHT_TIME = 6.0                # seconds -- 1s buffer above the 5s rule minimum

# --- Maximum time allowed to land on UGV after UGV starts moving (rules: 7 min) ---
MISSION_TIMEOUT = 420.0              # seconds (7 minutes)

# --- Marker centering gains and limits ---
KP_ROLL        = 280                 # proportional gain for lateral (x) correction
KP_PITCH       = 280                 # proportional gain for forward (y) correction
MAX_NUDGE      = 120                 # hard cap on PWM offset from RC_CENTER
METER_DEADBAND = 0.15                # 15 cm fallback deadband
CORRECTION_COOLDOWN = 0.7           # seconds between rc correction commands

# --- Centering hold and land gate ---
CENTER_HOLD_TIME = 4.0               # seconds to stay centered before attempting land
LAND_DEADBAND    = 0.08              # 8 cm -- tight gate used just before initiating LAND mode

# --- Marker acquisition confirmation ---
CONFIRM_NEEDED   = 3                 # consecutive detections before locking on

# --- Loop rate ---
FOLLOW_HZ = 10                       # Hz for centering/scan loop

# --- Post-landing UGV travel time (rules: 30 seconds after landing) ---
POST_LAND_TRAVEL_TIME = 30.0         # seconds UGV continues after UAV touchdown

# --- UGV speed command (rules: >= 0.2 mph = 0.089 m/s; we use 0.2 m/s for margin) ---
UGV_FORWARD_SPEED = 0.2              # m/s -- passed as speed in circle/drive calls

# --- Log file ---
LOG_FILE = "challenge1.log"

# --- Display window ---
WINDOW_NAME = "UAV Challenge 1 - Land on Moving UGV"


# =============================================================================
# VISION -- data types
# =============================================================================

@dataclass
class MarkerPosition:
    """Position of a detected ArUco marker in camera-frame metres (from tvec)."""
    marker_id: int
    x: float        # lateral (positive = right of camera center)
    y: float        # forward/vertical in camera frame (positive = below center for downward cam)
    z: float        # depth (distance along camera optical axis)
    distance: float # total Euclidean distance from camera
    detected: bool = True


# =============================================================================
# VISION -- CenterZone
# =============================================================================

class CenterZone:
    """Pixel-space rectangular acceptance box around the frame center."""

    def __init__(self, box_width: int = ZONE_BOX_WIDTH,
                 box_height: int = ZONE_BOX_HEIGHT):
        self.box_width  = box_width   # total width of the acceptance box in pixels
        self.box_height = box_height  # total height of the acceptance box in pixels

    def bounds(self, frame_w: int, frame_h: int) -> Tuple[int, int, int, int]:
        # returns (x_min, y_min, x_max, y_max) of the center box in pixel coords
        cx, cy = frame_w / 2.0, frame_h / 2.0
        return (
            int(cx - self.box_width  / 2.0),
            int(cy - self.box_height / 2.0),
            int(cx + self.box_width  / 2.0),
            int(cy + self.box_height / 2.0),
        )

    def contains(self, dx: float, dy: float) -> bool:
        # dx, dy are pixel offsets from the frame center (not metres)
        return (abs(dx) <= self.box_width  / 2.0 and
                abs(dy) <= self.box_height / 2.0)

    def draw(self, frame: np.ndarray, in_zone: bool) -> np.ndarray:
        # overlays the acceptance box on the frame with color feedback
        h, w = frame.shape[:2]
        x_min, y_min, x_max, y_max = self.bounds(w, h)
        color = (0, 255, 0) if in_zone else (0, 0, 255)  # green if centered, red if not
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)
        label = "IN ZONE" if in_zone else "OUT OF ZONE"
        cv2.putText(frame, label, (x_min, y_min - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        return frame


# =============================================================================
# VISION -- CameraInterface (ZED 2 only, with standard cam fallback)
# =============================================================================

class CameraInterface:
    """Wraps ZED 2 SDK camera init and frame capture for the Jetson Nano."""

    def __init__(self, use_zed: bool = True, camera_index: int = 0):
        self.use_zed      = use_zed      # flag: use zed or standard opencv cam
        self.camera_index = camera_index # opencv camera index if not using zed
        self.cap          = None         # opencv VideoCapture handle
        self.zed          = None         # zed camera object
        self._sl          = None         # pyzed.sl module reference

        print(f"[Camera] Initializing {'ZED 2' if use_zed else 'Standard'} camera...")
        if use_zed:
            self._initialize_zed()
        else:
            self._initialize_standard()
        print("[Camera] Camera ready.")

    def _initialize_zed(self):
        import pyzed.sl as sl  # only import if we actually need it
        self._sl = sl          # store reference so other methods can use it

        self.zed = sl.Camera()
        init_params = sl.InitParameters()

        # map the resolution string to the SDK constant
        res_map = {
            "HD2K":   sl.RESOLUTION.HD2K,
            "HD1080": sl.RESOLUTION.HD1080,
            "HD720":  sl.RESOLUTION.HD720,
            "VGA":    sl.RESOLUTION.VGA,
        }
        init_params.camera_resolution = res_map.get(ZED_RESOLUTION, sl.RESOLUTION.HD720)
        init_params.camera_fps   = ZED_FPS
        init_params.depth_mode   = sl.DEPTH_MODE.NONE  # no depth needed, saves CPU on Nano

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"[Camera] ZED open failed: {status}")

        # set exposure after open() as the SDK requires
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, ZED_EXPOSURE)
        print(f"[Camera] ZED 2 opened -- {ZED_RESOLUTION} @ {ZED_FPS}fps "
              f"exposure={ZED_EXPOSURE}")

    def _initialize_standard(self):
        # fallback: standard USB/webcam via opencv
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("[Camera] Failed to open standard camera.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)

    def get_frame(self) -> Optional[np.ndarray]:
        # returns the latest BGR frame or None if grab failed
        if self.use_zed:
            return self._get_zed_frame()
        return self._get_standard_frame()

    def _get_zed_frame(self) -> Optional[np.ndarray]:
        sl = self._sl
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return None  # grab failed, skip this cycle
        image = sl.Mat()
        self.zed.retrieve_image(image, sl.VIEW.LEFT)  # left eye for aruco
        return cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)  # strip alpha

    def _get_standard_frame(self) -> Optional[np.ndarray]:
        ret, frame = self.cap.read()
        return frame if ret else None  # return None if read failed

    def close(self):
        # cleanly release camera resources
        if self.use_zed and self.zed:
            self.zed.close()
        elif self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


# =============================================================================
# VISION -- ArucoDetector
# =============================================================================

class ArucoDetector:
    """Detects ArUco markers and estimates 3D pose using camera calibration."""

    def __init__(self, calibration_file: str, marker_size: float = MARKER_SIZE,
                 dictionary=aruco.DICT_6X6_1000):
        self.marker_size = marker_size  # physical marker size in metres
        self.aruco_dict  = aruco.getPredefinedDictionary(dictionary)

        # load camera calibration from the yaml file
        fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("K").mat()  # intrinsic camera matrix
        self.dist_coeffs   = fs.getNode("D").mat()  # lens distortion coefficients
        fs.release()

        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError(f"[Vision] Invalid calibration file: {calibration_file}")

        print(f"[Vision] Calibration loaded: {calibration_file} | "
              f"marker_size={marker_size}m | DICT_6X6_1000")

    def detect(self, frame: np.ndarray) -> Tuple[List[MarkerPosition],
                                                  Optional[np.ndarray],
                                                  Optional[np.ndarray]]:
        """
        Detect all ArUco markers in the frame.
        Returns (positions, corners, ids).
        positions is a list of MarkerPosition with real tvec metre data.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)

        positions = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                # estimate 3D pose using the calibration
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size,
                    self.camera_matrix, self.dist_coeffs,
                )
                tvec = tvecs[0].flatten()  # translation vector: [x, y, z] in metres
                positions.append(MarkerPosition(
                    marker_id=int(marker_id),
                    x=float(tvec[0]),          # lateral offset
                    y=float(tvec[1]),          # forward/depth offset
                    z=float(tvec[2]),          # height offset
                    distance=float(np.linalg.norm(tvec)),  # total distance
                ))
        return positions, corners, ids

    def draw_detections(self, frame: np.ndarray,
                        positions: List[MarkerPosition],
                        corners, ids) -> np.ndarray:
        """Annotate the frame with detected marker outlines and pose data."""
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id in enumerate(ids.flatten()):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_size,
                    self.camera_matrix, self.dist_coeffs,
                )
                # draw the 3D pose axes on the marker
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                  rvecs[0].flatten(), tvecs[0].flatten(),
                                  self.marker_size * 0.5)
                pos = next((p for p in positions if p.marker_id == marker_id), None)
                if pos:
                    center = np.mean(corners[i][0], axis=0).astype(int)
                    color  = (0, 255, 0)  # green for UGV marker
                    cv2.putText(frame,
                                f"ID:{marker_id} X:{pos.x:.2f}m Y:{pos.y:.2f}m "
                                f"D:{pos.distance:.2f}m",
                                tuple(center),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color,
                                2, cv2.LINE_AA)
        return frame


# =============================================================================
# LOGGING
# =============================================================================

def log(text: str):
    """Write a timestamped log entry to console and the log file."""
    ts   = time.strftime("%H:%M:%S")
    line = f"[{ts}] {text}"
    print(line)
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(line + "\n")


# =============================================================================
# MAVLINK HELPERS (same confirmed patterns from Mission4_Updated.py)
# =============================================================================

def change_mode(master, mode: str):
    """Switch the flight controller to the named flight mode."""
    mapping = master.mode_mapping()
    if mode not in mapping:
        log(f"[FC] Unknown mode '{mode}'")
        return
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    log(f"[FC] Mode set: {mode}")
    time.sleep(1)  # let the mode settle


def arm_drone(master):
    """Arm the motors via MAVLink command."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    log("[FC] Arming motors...")
    time.sleep(2)  # wait for the ESCs to spin up


def set_rc_override(master, roll: int = RC_CENTER, pitch: int = RC_CENTER,
                    throttle: int = THROTTLE_HOVER, yaw: int = RC_CENTER):
    """Send RC channel overrides for CH1-4. All channels sent every call."""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        roll, pitch, throttle, yaw,
        0, 0, 0, 0,
    )


def get_lidar_alt(master, blocking: bool = False) -> float:
    """Read the current altitude from the LiDAR sensor via MAVLink."""
    msg = master.recv_match(type="DISTANCE_SENSOR",
                            blocking=blocking, timeout=0.05)
    if msg:
        return msg.current_distance / 100.0  # convert cm to metres
    return 0.0  # return zero if no reading (safe fallback)


def throttle_hold(alt: float) -> int:
    """
    Bang-bang altitude hold. Returns a throttle PWM value that nudges the drone
    back toward TARGET_ALT when it drifts outside the ALT_BAND.
    """
    if alt < TARGET_ALT - ALT_BAND:
        return THROTTLE_HOVER + ALT_BOOST  # sinking, add power
    if alt > TARGET_ALT + ALT_BAND:
        return THROTTLE_HOVER - ALT_BOOST  # rising too high, cut power
    return THROTTLE_HOVER                   # in the band, hold steady


# =============================================================================
# CENTERING HELPERS
# =============================================================================

def compute_correction(pos: MarkerPosition) -> Tuple[int, int]:
    """
    Convert tvec metre offsets into roll and pitch PWM corrections.
    Positive x (marker right of center) -> roll right (increase PWM).
    Positive y (camera frame forward) -> pitch forward (decrease PWM).
    """
    roll_pwm  = int(RC_CENTER + max(-MAX_NUDGE, min(MAX_NUDGE,  KP_ROLL  * pos.x)))
    pitch_pwm = int(RC_CENTER + max(-MAX_NUDGE, min(MAX_NUDGE, -KP_PITCH * pos.y)))
    return roll_pwm, pitch_pwm


def marker_pixel_offset(corners, ids, target_id: int,
                        frame_w: int, frame_h: int) -> Tuple[Optional[float], Optional[float]]:
    """
    Find the pixel offset of the target marker's center from the frame center.
    Returns (dx, dy) in pixels, or (None, None) if the target isn't in the frame.
    """
    if ids is None:
        return None, None
    target_idx = next((i for i, mid in enumerate(ids.flatten())
                       if mid == target_id), None)
    if target_idx is None:
        return None, None
    mc = np.mean(corners[target_idx][0], axis=0)  # pixel centroid of the marker
    dx = mc[0] - frame_w / 2.0  # horizontal offset from frame center
    dy = mc[1] - frame_h / 2.0  # vertical offset from frame center
    return dx, dy


# =============================================================================
# DISPLAY HELPER
# =============================================================================

def show(frame, status: str, center_zone: CenterZone, in_zone: bool):
    """Draw the status overlay and center zone box on the frame and show it."""
    if frame is None:
        return
    out = frame.copy()
    center_zone.draw(out, in_zone)

    h, w = out.shape[:2]
    # draw a crosshair at the frame center
    cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 255),
                   cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
    # dark status bar at the top
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
    log("   UAV CHALLENGE 1 - LAND ON MOVING UGV  ")
    log("   Rules: >= 4ft alt, >= 5s flight,       ")
    log("   land within 7 min, 30s post-landing    ")
    log("==========================================")

    # ── Connect to flight controller ─────────────────────────────────────────
    log(f"[FC] Connecting: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()  # block until the FC heartbeat comes in
    log("[FC] Heartbeat OK.")

    # ── Connect to V2V radio bridge for UGV commands ─────────────────────────
    log(f"[Bridge] Starting V2V Bridge on {ESP32_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        bridge.send_message("CHALLENGE 1: UAV ONLINE - AWAITING LAUNCH")
    except Exception as e:
        log(f"[!] Radio Bridge Fail: {e}")
        return  # cannot proceed without the radio link to the UGV

    # ── Initialize ZED 2 camera and ArUco detector ───────────────────────────
    log("[Vision] Initializing ZED 2 camera and ArUco detector...")
    try:
        camera   = CameraInterface(use_zed=True)
        detector = ArucoDetector(CALIBRATION_FILE, marker_size=MARKER_SIZE)
    except Exception as e:
        log(f"[!] Vision init failed: {e}")
        bridge.stop()
        return  # cannot proceed without vision

    center_zone = CenterZone(ZONE_BOX_WIDTH, ZONE_BOX_HEIGHT)  # acceptance box

    # open the display window
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    loop_dt    = 1.0 / FOLLOW_HZ  # target time per loop iteration
    cmd_seq    = 600               # command sequence counter for the bridge

    try:
        ######################################################################
        # PHASE 1: ARM + CLIMB TO TARGET_ALT
        ######################################################################
        log("\n--- PHASE 1: ARM AND TAKEOFF ---")
        change_mode(master, "STABILIZE")  # manual throttle mode required for rc override
        arm_drone(master)                 # spin up the props

        log(f"[FC] Climbing to {TARGET_ALT}m ({TARGET_ALT*100/2.54/12:.1f} ft)...")
        while True:
            alt = get_lidar_alt(master, blocking=True)  # wait for a lidar reading
            print(f"\r  Alt: {alt:.2f}m / {TARGET_ALT}m", end="", flush=True)

            # grab a camera frame for the display even during climb
            frame = camera.get_frame()
            if frame is not None:
                show(frame, f"TAKEOFF  Alt:{alt:.2f}m  Target:{TARGET_ALT}m",
                     center_zone, False)

            if alt >= TARGET_ALT:
                set_rc_override(master, throttle=THROTTLE_HOVER)  # cut climb power
                log(f"\n[FC] Hover altitude reached: {alt:.2f}m")
                break

            set_rc_override(master, throttle=THROTTLE_CLIMB)  # keep climbing
            time.sleep(0.1)

        # record the time we first hit target altitude (flight timer starts here)
        flight_start_time = time.time()
        log(f"[Timer] Flight timer started. Min flight time: {MIN_FLIGHT_TIME}s.")

        ######################################################################
        # PHASE 2: COMMAND UGV TO START MOVING FORWARD
        #          (rules: UGV starts after UAV has launched)
        #          We command the UGV right after achieving altitude.
        ######################################################################
        log("\n--- PHASE 2: COMMAND UGV TO MOVE FORWARD ---")
        # CMD_MOVE_FORWARD tells the ground station to start driving straight
        bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0)
        cmd_seq += 1
        bridge.send_message("CHALLENGE 1: UGV START MOVING FORWARD")
        log("[Bridge] UGV ordered to move forward at >= 0.2 mph.")

        # record when the UGV starts moving (mission timeout is from this point)
        ugv_start_time = time.time()
        log(f"[Timer] Mission timeout: {MISSION_TIMEOUT}s from now.")

        ######################################################################
        # PHASE 3: ENFORCE MINIMUM 5-SECOND FLIGHT TIME + SCAN FOR MARKER
        #          The drone hovers and scans while ensuring the 5s rule is met.
        ######################################################################
        log(f"\n--- PHASE 3: MIN FLIGHT TIME ({MIN_FLIGHT_TIME}s) + SCAN FOR ARUCO ---")

        target_id            = None   # the locked-on marker ID
        confirm_count        = 0      # consecutive detection counter for lock-on
        last_correction_time = 0.0   # tracks last RC correction timestamp

        while True:
            loop_start = time.time()
            now        = time.time()

            # check the 7-minute mission timeout
            elapsed_mission = now - ugv_start_time
            if elapsed_mission >= MISSION_TIMEOUT:
                log(f"[!] MISSION TIMEOUT ({MISSION_TIMEOUT}s). Aborting landing attempt.")
                break  # drop out to the finally block for cleanup

            alt = get_lidar_alt(master)
            thr = throttle_hold(alt)  # altitude hold throttle value

            # grab a frame and run aruco detection
            frame = camera.get_frame()
            if frame is None:
                set_rc_override(master, throttle=thr)
                time.sleep(loop_dt)
                continue  # skip if we couldn't get a frame

            positions, corners, ids = detector.detect(frame)
            annotated = detector.draw_detections(frame.copy(), positions, corners, ids)

            # check minimum flight time rule before allowing centering/landing
            flight_elapsed = now - flight_start_time
            min_time_ok    = flight_elapsed >= MIN_FLIGHT_TIME

            if not min_time_ok:
                # still enforcing minimum flight time -- just hover and scan
                remaining_min = MIN_FLIGHT_TIME - flight_elapsed
                status = (f"MIN FLIGHT TIME: {flight_elapsed:.1f}s / {MIN_FLIGHT_TIME}s  "
                          f"wait:{remaining_min:.1f}s  Alt:{alt:.2f}m")
                set_rc_override(master, throttle=thr)
                show(annotated, status, center_zone, False)
                sleep_t = loop_dt - (time.time() - loop_start)
                if sleep_t > 0:
                    time.sleep(sleep_t)
                continue  # loop back until min time is satisfied

            # minimum flight time satisfied -- now actively acquire the marker
            if target_id is None:
                # acquisition mode: count consecutive detections before locking on
                if positions:
                    confirm_count += 1
                    candidate = positions[0].marker_id  # grab first detected marker
                    if confirm_count >= CONFIRM_NEEDED:
                        # confirmed the marker -- lock on
                        target_id = candidate
                        log(f"\n[ArUco] Locked on ID:{target_id}. "
                            f"Beginning centering phase...")
                    status = (f"CONFIRMING ID:{candidate}  "
                              f"{confirm_count}/{CONFIRM_NEEDED}  "
                              f"Alt:{alt:.2f}m  "
                              f"Mission:{elapsed_mission:.0f}s/{MISSION_TIMEOUT:.0f}s")
                else:
                    confirm_count = 0  # reset if we lost it
                    status = (f"SEARCHING for UGV marker...  "
                              f"Alt:{alt:.2f}m  "
                              f"Mission:{elapsed_mission:.0f}s/{MISSION_TIMEOUT:.0f}s")

                set_rc_override(master, throttle=thr)
                show(annotated, status, center_zone, False)

            else:
                # centering mode: correct roll/pitch to align above the marker
                pos = next((p for p in positions
                            if p.marker_id == target_id), None)

                if pos:
                    # pixel-space check for the acceptance zone
                    frame_h, frame_w = annotated.shape[:2]
                    dx, dy = marker_pixel_offset(corners, ids, target_id,
                                                 frame_w, frame_h)
                    in_zone = (dx is not None and
                               center_zone.contains(dx, dy))

                    if not in_zone and (now - last_correction_time) >= CORRECTION_COOLDOWN:
                        # apply roll/pitch correction to drift toward the marker
                        roll_pwm, pitch_pwm = compute_correction(pos)
                        set_rc_override(master, roll=roll_pwm,
                                        pitch=pitch_pwm, throttle=thr)
                        log(f"[Centre] x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                            f"roll:{roll_pwm} pitch:{pitch_pwm}  "
                            f"Mission:{elapsed_mission:.1f}s")
                        last_correction_time = now
                    else:
                        set_rc_override(master, throttle=thr)  # hold steady if in zone

                    status = (f"CENTRING ID:{target_id}  "
                              f"x:{pos.x:.2f}m y:{pos.y:.2f}m  "
                              f"Alt:{alt:.2f}m  "
                              f"Mission:{elapsed_mission:.0f}s/{MISSION_TIMEOUT:.0f}s")
                    show(annotated, status, center_zone, in_zone)
                    print(f"\r  {status}", end="", flush=True)

                    # if we've been in the zone long enough, advance to final land gate
                    if in_zone:
                        log(f"\n[ArUco] In zone. Advancing to final land gate.")
                        break  # exit centering loop for Phase 4

                else:
                    # marker lost mid-centering -- hold position and keep scanning
                    set_rc_override(master, throttle=thr)
                    status = (f"MARKER LOST - holding  ID:{target_id}  "
                              f"Alt:{alt:.2f}m  "
                              f"Mission:{elapsed_mission:.0f}s/{MISSION_TIMEOUT:.0f}s")
                    show(annotated, status, center_zone, False)

            sleep_t = loop_dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        ######################################################################
        # PHASE 4: FINAL TIGHT RE-CENTER BEFORE LAND
        #          Tighten the deadband to LAND_DEADBAND to ensure accurate touch.
        ######################################################################
        log(f"\n--- PHASE 4: FINAL LAND GATE (deadband={LAND_DEADBAND*100:.1f}cm) ---")

        last_correction_time = 0.0  # reset correction timer for the tighter phase

        while True:
            loop_start = time.time()
            now        = time.time()

            # still enforce the mission timeout even during final centering
            elapsed_mission = now - ugv_start_time
            if elapsed_mission >= MISSION_TIMEOUT:
                log(f"[!] MISSION TIMEOUT during land gate. Forcing land.")
                break  # abort centering and go straight to land

            alt = get_lidar_alt(master)
            thr = throttle_hold(alt)

            frame = camera.get_frame()
            if frame is None:
                set_rc_override(master, throttle=thr)
                time.sleep(loop_dt)
                continue

            positions, corners, ids = detector.detect(frame)
            annotated = detector.draw_detections(frame.copy(), positions, corners, ids)

            pos = next((p for p in positions
                        if p.marker_id == target_id), None)

            if pos:
                frame_h, frame_w = annotated.shape[:2]
                dx, dy = marker_pixel_offset(corners, ids, target_id, frame_w, frame_h)
                centered = (dx is not None and center_zone.contains(dx, dy))

                if centered:
                    # tight enough -- commit to land
                    set_rc_override(master, throttle=thr)
                    show(annotated,
                         f"CENTRED  x:{pos.x:.3f}m y:{pos.y:.3f}m  -> LANDING",
                         center_zone, True)
                    log("[ArUco] Inside land gate. Initiating LAND mode.")
                    break  # exit to the landing sequence

                # apply a tighter correction using the metre-based deadband
                if (now - last_correction_time) >= CORRECTION_COOLDOWN:
                    if (abs(pos.x) > LAND_DEADBAND or abs(pos.y) > LAND_DEADBAND):
                        roll_pwm, pitch_pwm = compute_correction(pos)
                        set_rc_override(master, roll=roll_pwm,
                                        pitch=pitch_pwm, throttle=thr)
                        log(f"[LandGate] x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                            f"need<{LAND_DEADBAND:.3f}m  "
                            f"roll:{roll_pwm} pitch:{pitch_pwm}")
                        last_correction_time = now
                    else:
                        set_rc_override(master, throttle=thr)
                else:
                    set_rc_override(master, throttle=thr)

                show(annotated,
                     f"FINAL CENTRE  x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                     f"need<{LAND_DEADBAND*100:.1f}cm  Alt:{alt:.2f}m",
                     center_zone, False)
            else:
                # lost the target -- hover and wait for it to come back into view
                set_rc_override(master, throttle=thr)
                show(annotated,
                     f"FINAL CENTRE - target lost  Alt:{alt:.2f}m",
                     center_zone, False)

            sleep_t = loop_dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        ######################################################################
        # LAND: Switch to LAND mode and wait for autopilot to confirm disarm
        ######################################################################
        log("\n--- LAND: SWITCHING TO LAND MODE ---")
        change_mode(master, "LAND")  # autopilot handles the descent
        # release the RC override so the autopilot can control throttle cleanly
        set_rc_override(master, roll=RC_CENTER, pitch=RC_CENTER,
                        throttle=0, yaw=RC_CENTER)

        log("[FC] Descending... waiting for touchdown confirmation.")
        while True:
            alt = get_lidar_alt(master)
            print(f"\r  Land alt: {alt:.2f}m", end="", flush=True)

            # keep showing camera feed during descent
            frame = camera.get_frame()
            if frame is not None:
                show(frame, f"LANDING  Alt:{alt:.2f}m", center_zone, False)

            # the autopilot disarms itself after a confirmed landing
            msg = master.recv_match(type="HEARTBEAT", blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                log("\n[FC] Touchdown confirmed. Motors stopped.")
                break  # motors stopped, we're on the UGV

            time.sleep(0.5)  # slow loop during descent -- no need for high rate

        ######################################################################
        # PHASE 6: POST-LANDING -- UGV CONTINUES FOR 30 SECONDS
        #          Rules: "it must continue traveling with the UGV system for
        #          thirty seconds without separating"
        #          The UGV is already moving. We just wait the required 30s.
        #          Then we send the STOP command to the UGV.
        ######################################################################
        log(f"\n--- PHASE 6: POST-LANDING TRAVEL ({POST_LAND_TRAVEL_TIME}s) ---")
        log("[Bridge] UAV landed. UGV continues moving. Waiting 30 seconds...")

        post_land_start = time.time()
        while (time.time() - post_land_start) < POST_LAND_TRAVEL_TIME:
            # broadcast telemetry back to keep the radio link alive during the wait
            data = bridge.get_telemetry()
            elapsed_travel = time.time() - post_land_start
            remaining_travel = POST_LAND_TRAVEL_TIME - elapsed_travel
            if data:
                print(f"\r  Post-land travel: {elapsed_travel:.1f}s / "
                      f"{POST_LAND_TRAVEL_TIME}s  "
                      f"(remaining: {remaining_travel:.1f}s)", end="", flush=True)
            time.sleep(0.5)  # 2Hz check rate is plenty here

        log(f"\n[Bridge] 30 seconds elapsed. Sending STOP to UGV.")
        bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_STOP, estop=0)
        cmd_seq += 1
        bridge.send_message("CHALLENGE 1: MISSION COMPLETE - UGV STOP")
        log("[Challenge 1] MISSION COMPLETE. Time stops now.")

    except KeyboardInterrupt:
        # emergency abort -- user hit Ctrl+C during the mission
        log("\n[!] Emergency: User triggered abort. Switching to LAND mode.")
        change_mode(master, "LAND")
        set_rc_override(master, roll=RC_CENTER, pitch=RC_CENTER,
                        throttle=0, yaw=RC_CENTER)
        # also stop the UGV immediately on emergency
        try:
            bridge.send_command(cmdSeq=cmd_seq, cmd=v2v_bridge.CMD_STOP, estop=1)
            bridge.send_message("CHALLENGE 1: EMERGENCY ABORT")
        except Exception:
            pass  # best effort -- don't mask the original interrupt
        time.sleep(1)

    finally:
        # always clean up, no matter how we exited
        camera.close()    # close the ZED 2 and destroy opencv windows
        bridge.stop()     # close the V2V radio serial link
        log("Challenge 1 mission finalized.")


# =============================================================================
# UGV GROUND STATION ADDITIONS (ground_station.py extension)
# =============================================================================
# The ground_station.py needs a handler for CMD_MOVE_FORWARD that drives the
# UGV forward continuously at >= 0.2 mph (0.089 m/s) until CMD_STOP is received.
# The existing execute_drive() function drives for a fixed distance -- for
# Challenge 1, the UGV must drive indefinitely until stopped by the UAV.
#
# Add this function to ground_station.py and add the CMD handler in main():
#
#   def execute_forward_indefinite(bridge):
#       """Drive forward at UGV_FORWARD_SPEED until CMD_STOP is received."""
#       import math
#       from pymavlink import mavutil as mavu
#       speed = 0.2  # m/s -- satisfies >= 0.2 mph rule
#       print(f"[Ground] INDEFINITE FORWARD DRIVE at {speed} m/s until STOP cmd.")
#       msg = vehicle.message_factory.set_position_target_local_ned_encode(
#           0, 0, 0, mavu.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
#           0, 0, 0, speed, 0, 0, 0, 0, 0, 0, 0)
#       while True:
#           vehicle.send_mavlink(msg)
#           broadcast_status(bridge, 0)
#           cmd = bridge.get_command()
#           if cmd:
#               _, cmdVal, eStopFlag = cmd
#               if cmdVal == v2v_bridge.CMD_STOP or eStopFlag == 1:
#                   print("[Ground] STOP received. Halting UGV.")
#                   break
#           time.sleep(0.1)
#       # full stop
#       stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
#           0, 0, 0, mavu.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
#           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
#       vehicle.send_mavlink(stop_msg)
#
#   # In ground_station.py main() cmd handler, add:
#   elif cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
#       if not vehicle.armed: arm_and_move(bridge)
#       execute_forward_indefinite(bridge)


if __name__ == "__main__":
    main()
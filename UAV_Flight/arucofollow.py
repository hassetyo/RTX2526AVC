"""
UAV ArUco Marker Follow Mission
================================
Flow:
  1. Connect to flight controller via MAVLink
  2. Arm + climb to TARGET_ALT using throttle RC override (STABILIZE mode)
  3. Hold hover and scan for ArUco marker ID 0
  4. If the marker centre is OUTSIDE the acceptance box, nudge roll/pitch
     RC channels to re-centre — with a cooldown so we don't spam commands
  5. Ctrl-C triggers a clean LAND + disarm

RC channel mapping (ArduPilot default):
  CH1 = Roll       (1500 = centre, <1500 = left,  >1500 = right)
  CH2 = Pitch      (1500 = centre, <1500 = fwd,   >1500 = back)
  CH3 = Throttle   (raw PWM)
  CH4 = Yaw        (1500 = centre)

All channels we are NOT touching are sent as 0, which tells ArduPilot to
ignore them and use whatever the sticks say (or the last override value).
"""

from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, List

################################# TUNABLE CONFIG ##################################

# --- MAVLink ---
CONNECTION_STRING = "/dev/ttyACM0"   # serial port to Pixhawk
BAUD_RATE         = 57600            # confirmed working baud

# --- Flight params ---
TARGET_ALT      = 1.3   # hover height in meters
THROTTLE_MIN    = 1000  # motors off / floor
THROTTLE_IDLE   = 1150  # props spinning, no lift
THROTTLE_CLIMB  = 1650  # power to climb
THROTTLE_HOVER  = 1500  # neutral hover power
ALT_BAND        = 0.1   # ±metres before altitude correction kicks in
ALT_BOOST       = 100   # PWM added/removed for altitude correction

# --- Camera / ArUco ---
CAMERA_INDEX       = 0                     # USB camera index
CALIBRATION_FILE   = "calibration_chessboard.yaml"
MARKER_SIZE        = 0.1                   # physical marker size in metres
ARUCO_DICT         = aruco.DICT_6X6_1000   # must match printed markers
TARGET_MARKER_ID   = 0                     # the marker we chase

# --- Centre-zone acceptance box (pixels) ---
ZONE_BOX_WIDTH  = 200   # total width  — shrink for tighter following
ZONE_BOX_HEIGHT = 200   # total height — shrink for tighter following

# --- Correction nudge (PWM away from 1500 neutral) ---
# How hard each axis corrects when the marker is outside the box.
# Start small and tune upward — too large = oscillation.
ROLL_NUDGE    = 50   # PWM  (applied left/right to centre marker horizontally)
PITCH_NUDGE   = 50   # PWM  (applied fwd/back   to centre marker vertically)

# --- Cooldown between correction commands (seconds) ---
# The drone needs time to physically respond before we check again.
CORRECTION_COOLDOWN = 0.8   # seconds — increase if drone oscillates

# --- Log file ---
LOG_FILE = "aruco_follow_mission.log"

###################################################################################


# ──────────────────────────────────────────────────────────────────────────────
# Logging
# ──────────────────────────────────────────────────────────────────────────────

def log(text: str):
    """Timestamped print + append to log file."""
    ts   = time.strftime("%H:%M:%S")
    line = f"[{ts}] {text}"
    print(line)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")


# ──────────────────────────────────────────────────────────────────────────────
# MAVLink helpers  (taken directly from Mission4_Updated.py pattern)
# ──────────────────────────────────────────────────────────────────────────────

def change_mode(master, mode: str):
    """Switch ArduPilot flight mode."""
    mapping = master.mode_mapping()
    if mode not in mapping:
        log(f"Unknown mode '{mode}'")
        return
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    log(f"Mode set: {mode}")
    time.sleep(1)


def arm_drone(master):
    """Arm motors."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    log("Arming motors...")
    time.sleep(2)


def disarm_drone(master):
    """Disarm motors."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0,
    )
    log("Disarmed.")


def rc_override(master, roll=0, pitch=0, throttle=0, yaw=0):
    """
    Send RC channel overrides.
    Pass 0 for any channel to leave it at its current value (ArduPilot ignores 0).
    CH1=roll  CH2=pitch  CH3=throttle  CH4=yaw
    """
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        roll, pitch, throttle, yaw,   # CH1-4
        0, 0, 0, 0,                   # CH5-8 unused
    )


def get_lidar_alt(master) -> float:
    """Read altitude from lidar/rangefinder in metres."""
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=1.0)
    if msg:
        return msg.current_distance / 100.0
    return 0.0


# ──────────────────────────────────────────────────────────────────────────────
# Vision — ArUco detection (extracted from uav_vision.py)
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class MarkerOffset:
    """Pixel offset of a marker centre from the frame centre."""
    marker_id: int
    dx: float   # positive = marker is to the RIGHT of centre
    dy: float   # positive = marker is BELOW  centre (OpenCV coords)


class CenterZone:
    """Rectangular acceptance box around the frame centre."""

    def __init__(self, box_w: int = ZONE_BOX_WIDTH, box_h: int = ZONE_BOX_HEIGHT):
        self.box_w = box_w
        self.box_h = box_h

    def contains(self, dx: float, dy: float) -> bool:
        return abs(dx) <= self.box_w / 2.0 and abs(dy) <= self.box_h / 2.0

    def draw(self, frame: np.ndarray, in_zone: bool) -> np.ndarray:
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        x1 = cx - self.box_w // 2
        y1 = cy - self.box_h // 2
        x2 = cx + self.box_w // 2
        y2 = cy + self.box_h // 2
        color  = (0, 255, 0) if in_zone else (0, 0, 255)
        label  = "IN ZONE" if in_zone else "OUT OF ZONE"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        cv2.drawMarker(frame, (cx, cy), (255, 255, 255),
                       cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
        return frame


class VisionThread:
    """
    Runs ArUco detection in a background thread so the flight-control loop
    is never blocked waiting for a camera frame.

    After each frame the latest MarkerOffset for TARGET_MARKER_ID (or None
    if not detected) is available via .get_target_offset().
    """

    def __init__(self, camera_index: int, calibration_file: str,
                 marker_size: float, aruco_dict_id: int,
                 target_id: int, zone: CenterZone):
        self.target_id = target_id
        self.zone      = zone
        self._offset: Optional[MarkerOffset] = None
        self._lock     = threading.Lock()
        self._running  = False
        self._thread   = None

        # Camera
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_AVFOUNDATION)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)

        # ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_id)
        fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("K").mat()
        self.dist_coeffs   = fs.getNode("D").mat()
        fs.release()
        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError(f"Bad calibration file: {calibration_file}")
        self.marker_size = marker_size

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log("Vision thread started.")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.cap.release()
        cv2.destroyAllWindows()
        log("Vision thread stopped.")

    def get_target_offset(self) -> Optional[MarkerOffset]:
        """Thread-safe read of the latest target marker offset."""
        with self._lock:
            return self._offset

    def _loop(self):
        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            frame_h, frame_w = frame.shape[:2]
            cx = frame_w / 2.0
            cy = frame_h / 2.0

            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)

            target_offset: Optional[MarkerOffset] = None
            in_zone = False

            if ids is not None:
                # Draw all detected markers
                frame = aruco.drawDetectedMarkers(frame, corners, ids)

                for i, mid in enumerate(ids.flatten()):
                    mc  = np.mean(corners[i][0], axis=0)
                    dx  = mc[0] - cx
                    dy  = mc[1] - cy

                    # Pose estimation for the distance label
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners[i], self.marker_size,
                        self.camera_matrix, self.dist_coeffs,
                    )
                    tvec = tvecs[0].flatten()
                    dist = float(np.linalg.norm(tvec))

                    color = (0, 255, 0) if mid == self.target_id else (0, 165, 255)
                    label = f"ID:{mid} D:{dist:.2f}m"
                    cv2.putText(frame, label,
                                tuple(mc.astype(int)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

                    if mid == self.target_id:
                        target_offset = MarkerOffset(
                            marker_id=int(mid), dx=float(dx), dy=float(dy)
                        )
                        in_zone = self.zone.contains(dx, dy)

            # Update shared state
            with self._lock:
                self._offset = target_offset

            # Draw zone box and show frame
            self.zone.draw(frame, in_zone)
            cv2.imshow("UAV ArUco Follow", frame)
            cv2.waitKey(1)


# ──────────────────────────────────────────────────────────────────────────────
# Correction logic
# ──────────────────────────────────────────────────────────────────────────────

def compute_correction(offset: MarkerOffset, zone: CenterZone) -> Tuple[int, int]:
    """
    Given a marker offset that is OUTSIDE the zone, return (roll_pwm, pitch_pwm)
    RC override values to nudge the drone toward the marker.

    Roll  (CH1): marker to the right  (dx > 0) → roll right → PWM > 1500
                 marker to the left   (dx < 0) → roll left  → PWM < 1500
    Pitch (CH2): marker below centre  (dy > 0) → pitch back → PWM > 1500
                 marker above centre  (dy < 0) → pitch fwd  → PWM < 1500

    Only the axis that is outside the zone gets corrected; the other stays at
    1500 (neutral) so we don't fight ourselves.
    """
    roll_pwm  = 1500
    pitch_pwm = 1500

    half_w = zone.box_w / 2.0
    half_h = zone.box_h / 2.0

    # Horizontal correction (roll)
    if abs(offset.dx) > half_w:
        roll_pwm = 1500 + int(np.sign(offset.dx) * ROLL_NUDGE)

    # Vertical correction (pitch)
    # Camera dy positive = marker is BELOW centre = drone needs to fly forward
    if abs(offset.dy) > half_h:
        pitch_pwm = 1500 + int(np.sign(offset.dy) * PITCH_NUDGE)

    return roll_pwm, pitch_pwm


# ──────────────────────────────────────────────────────────────────────────────
# Main mission
# ──────────────────────────────────────────────────────────────────────────────

def main():
    log("==========================================")
    log("   UAV ARUCO FOLLOW MISSION")
    log("==========================================")

    # 1. Connect to flight controller
    log(f"Connecting to flight controller: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    log("Heartbeat OK — drone is alive.")

    # 2. Start vision in background
    zone   = CenterZone(ZONE_BOX_WIDTH, ZONE_BOX_HEIGHT)
    vision = VisionThread(
        camera_index     = CAMERA_INDEX,
        calibration_file = CALIBRATION_FILE,
        marker_size      = MARKER_SIZE,
        aruco_dict_id    = ARUCO_DICT,
        target_id        = TARGET_MARKER_ID,
        zone             = zone,
    )
    vision.start()

    try:
        # ── Phase 1: Arm and climb ────────────────────────────────────────────
        change_mode(master, "STABILIZE")
        arm_drone(master)

        log(f"Climbing to {TARGET_ALT}m...")
        while True:
            alt = get_lidar_alt(master)
            print(f"\r  Altitude: {alt:.2f}m", end="", flush=True)
            if alt >= TARGET_ALT:
                rc_override(master, throttle=THROTTLE_HOVER)
                log(f"\nHover altitude reached: {alt:.2f}m")
                break
            rc_override(master, throttle=THROTTLE_CLIMB)
            time.sleep(0.1)

        log("Hovering. Scanning for ArUco marker...")

        # ── Phase 2: Marker follow loop ───────────────────────────────────────
        last_correction_time = 0.0   # tracks when we last sent a nudge

        while True:
            now = time.time()

            # ── Altitude hold (runs every iteration) ─────────────────────────
            alt = get_lidar_alt(master)
            if alt < TARGET_ALT - ALT_BAND:
                throttle_pwm = THROTTLE_HOVER + ALT_BOOST   # sinking → more power
            elif alt > TARGET_ALT + ALT_BAND:
                throttle_pwm = THROTTLE_HOVER - ALT_BOOST   # drifting high → less power
            else:
                throttle_pwm = THROTTLE_HOVER               # in the band → hold steady

            # ── Marker correction (rate-limited by cooldown) ──────────────────
            offset = vision.get_target_offset()

            if offset is None:
                # No marker visible — hold position, just keep throttle alive
                rc_override(master, roll=1500, pitch=1500, throttle=throttle_pwm, yaw=1500)
                log("Marker not detected — holding position.")
                time.sleep(0.2)
                continue

            in_zone = zone.contains(offset.dx, offset.dy)

            if in_zone:
                # Marker is centred — neutral roll/pitch, maintain altitude
                rc_override(master, roll=1500, pitch=1500, throttle=throttle_pwm, yaw=1500)
                log(f"Marker ID {offset.marker_id}: IN ZONE "
                    f"(dx={offset.dx:.1f}px, dy={offset.dy:.1f}px)")

            else:
                # Marker is off-centre — but only send a nudge if cooldown has expired
                if (now - last_correction_time) >= CORRECTION_COOLDOWN:
                    roll_pwm, pitch_pwm = compute_correction(offset, zone)
                    rc_override(master,
                                roll=roll_pwm, pitch=pitch_pwm,
                                throttle=throttle_pwm, yaw=1500)
                    log(f"Marker ID {offset.marker_id}: OUTSIDE ZONE "
                        f"dx={offset.dx:.1f}px dy={offset.dy:.1f}px "
                        f"→ roll={roll_pwm} pitch={pitch_pwm}")
                    last_correction_time = now
                else:
                    # Cooldown active — keep throttle alive but don't change attitude
                    rc_override(master, throttle=throttle_pwm)

            time.sleep(0.05)   # 20 Hz flight-control tick

    except KeyboardInterrupt:
        log("\n[!] Ctrl-C detected — initiating emergency landing...")

    finally:
        # ── Phase 3: Land ─────────────────────────────────────────────────────
        log("Switching to LAND mode...")
        change_mode(master, "LAND")
        rc_override(master, throttle=0)   # release override so autopilot lands cleanly

        log("Waiting for touchdown...")
        while True:
            alt = get_lidar_alt(master)
            print(f"\r  Land alt: {alt:.2f}m", end="", flush=True)
            msg = master.recv_match(type="HEARTBEAT", blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                log("\nTouchdown confirmed. Motors stopped.")
                break
            time.sleep(0.5)

        vision.stop()
        log("Mission complete.")


if __name__ == "__main__":
    main()

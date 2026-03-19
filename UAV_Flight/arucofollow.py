"""
UAV Mission 5 - Vision Guided ArUco Follow
============================================
Flow:
  Phase 1 — Arm + climb to TARGET_ALT
  Phase 2 — Fly forward ~5 m (timed pitch override)
  Phase 3 — Scan for ArUco marker, then centre on it for CENTER_HOLD_TIME seconds
             Corrections use real tvec metres from pose estimation (not pixel offsets)
             A cooldown gate prevents spamming the flight controller
  Phase 4 — Final re-centre tightened to LAND_DEADBAND, then switch to LAND mode
  Land    — Wait for autopilot to confirm disarm

Camera / vision is handled entirely by UAVVision from uav_vision.py.
That module is imported and used exactly as-is — do NOT change any camera
settings here; they live in uav_vision.py (HD1080, DEPTH_MODE.NONE, exposure=1).

MAVLink pattern is the same confirmed approach from Mission4_Updated.py:
  STABILIZE mode + RC channel override for throttle / roll / pitch.
"""

# ── standard imports ──────────────────────────────────────────────────────────
from pymavlink import mavutil
import time
import cv2
import numpy as np

# ── vision module — import everything directly so camera settings are untouched
from uav_vision import UAVVision, MarkerPosition


# ═════════════════════════════════════════════════════════════════════════════
# TUNABLE MISSION CONFIG
# ═════════════════════════════════════════════════════════════════════════════

# --- MAVLink ---
CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE         = 57600

# --- Calibration / vision ---
CALIBRATION_FILE = "../CameraCalibration/calibration_chessboard.yaml"
MARKER_SIZE      = 0.1          # metres — must match your printed marker

# --- Flight ---
TARGET_ALT     = 1.3            # hover height in metres
THROTTLE_CLIMB = 1650           # PWM to climb
THROTTLE_HOVER = 1500           # PWM neutral hover
ALT_BAND       = 0.1            # ±m band before altitude correction fires
ALT_BOOST      = 100            # PWM delta applied when outside ALT_BAND

# --- Forward flight phase ---
# Tune FORWARD_PITCH_PWM + FORWARD_FLIGHT_TIME so distance ≈ 5 m.
# At ~1580 PWM expect 0.6-0.8 m/s  →  7 s ≈ 5 m
FORWARD_PITCH_PWM   = 1580
FORWARD_FLIGHT_TIME = 7.0       # seconds

# --- Marker acquisition ---
CONFIRM_NEEDED = 3              # consecutive detections before locking onto a marker

# --- Centering PID (metre-based, uses tvec from pose estimation) ---
# tvec x = side offset  →  roll correction
# tvec y = fwd  offset  →  pitch correction
# Both are 0 when drone is directly above the marker.
KP_ROLL        = 300            # gain:  0.3 m * 300 = 90 PWM nudge
KP_PITCH       = 300
MAX_NUDGE      = 150            # hard cap on PWM offset from RC_CENTER
METER_DEADBAND = 0.05           # 5 cm — ignore errors smaller than this

# --- Cooldown between correction commands ---
# Drone needs time to physically respond; don't flood it with commands.
CORRECTION_COOLDOWN = 0.8       # seconds

# --- Hold timer ---
CENTER_HOLD_TIME = 60.0         # seconds to stay centred before land phase
LAND_DEADBAND    = 0.075        # 7.5 cm — relaxed gate used just before landing

# --- Loop rate ---
FOLLOW_HZ = 10                  # Hz for the centering loop

# --- Display ---
WINDOW_NAME = "UAV Mission 5 - Vision Guided"

# --- RC neutral ---
RC_CENTER = 1500

# --- Log file ---
LOG_FILE = "mission5.log"


# ═════════════════════════════════════════════════════════════════════════════
# LOGGING
# ═════════════════════════════════════════════════════════════════════════════

def log(text: str):
    ts   = time.strftime("%H:%M:%S")
    line = f"[{ts}] {text}"
    print(line)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")


# ═════════════════════════════════════════════════════════════════════════════
# MAVLINK HELPERS  (confirmed Mission4 pattern — unchanged)
# ═════════════════════════════════════════════════════════════════════════════

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
    """Simple bang-bang altitude hold around TARGET_ALT."""
    if alt < TARGET_ALT - ALT_BAND:
        return THROTTLE_HOVER + ALT_BOOST   # sinking → more power
    if alt > TARGET_ALT + ALT_BAND:
        return THROTTLE_HOVER - ALT_BOOST   # too high → less power
    return THROTTLE_HOVER


# ═════════════════════════════════════════════════════════════════════════════
# CENTERING HELPERS
# ═════════════════════════════════════════════════════════════════════════════

def compute_correction(pos: MarkerPosition):
    """
    Convert tvec metres into roll/pitch PWM values.

    pos.x  side offset  →  roll   (right = positive x = roll right = PWM > 1500)
    pos.y  fwd  offset  →  pitch  (forward = positive y = pitch fwd = PWM < 1500)

    Note on pitch sign:
      In the camera frame, tvec y is positive when the marker is BELOW the
      camera (drone needs to fly forward to centre).  ArduPilot CH2 < 1500
      means pitch forward, so we subtract the nudge.
    """
    ex = 0.0 if abs(pos.x) < METER_DEADBAND else pos.x
    ey = 0.0 if abs(pos.y) < METER_DEADBAND else pos.y

    roll_pwm  = int(RC_CENTER + max(-MAX_NUDGE, min(MAX_NUDGE,  KP_ROLL  * ex)))
    pitch_pwm = int(RC_CENTER + max(-MAX_NUDGE, min(MAX_NUDGE, -KP_PITCH * ey)))
    return roll_pwm, pitch_pwm


def is_centered(pos: MarkerPosition, deadband: float = METER_DEADBAND) -> bool:
    return abs(pos.x) <= deadband and abs(pos.y) <= deadband


# ═════════════════════════════════════════════════════════════════════════════
# DISPLAY HELPER
# ═════════════════════════════════════════════════════════════════════════════

def show(frame, status: str, vision, target_in_zone: bool):
    """
    Overlay a status bar on the annotated frame and push it to the window.
    Also draws the centre-zone box using UAVVision's CenterZone helper.
    """
    if frame is None:
        return

    out = frame.copy()

    # Draw the centre-zone box (green = in, red = out / not detected)
    vision.center_zone.draw(out, target_in_zone)

    # Draw crosshair at exact frame centre
    h, w = out.shape[:2]
    cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 255),
                   cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)

    # Status bar along the top
    cv2.rectangle(out, (0, 0), (w, 36), (0, 0, 0), -1)
    cv2.putText(out, status, (8, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow(WINDOW_NAME, out)
    cv2.waitKey(1)


# ═════════════════════════════════════════════════════════════════════════════
# MAIN MISSION
# ═════════════════════════════════════════════════════════════════════════════

def main():
    log("==========================================")
    log("   UAV MISSION 5 - VISION GUIDED")
    log("   Takeoff -> 5m Forward -> Centre ArUco (60s) -> Land")
    log("==========================================")

    # ── Connect to flight controller ─────────────────────────────────────────
    log(f"[FC] Connecting: {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    master.wait_heartbeat()
    log("[FC] Heartbeat OK.")

    # ── Init vision (camera settings live entirely in uav_vision.py) ─────────
    log("[Vision] Starting UAVVision with ZED camera...")
    try:
        vision = UAVVision(
            calibration_file=CALIBRATION_FILE,
            marker_size=MARKER_SIZE,
            use_zed=True,           # ZED camera — settings in uav_vision.py
            zone_box_width=200,
            zone_box_height=200,
        )
    except Exception as e:
        log(f"[!] Vision init failed: {e}")
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    loop_dt = 1.0 / FOLLOW_HZ

    try:
        ######################################################################
        # PHASE 1: ARM + CLIMB
        ######################################################################
        log("\n--- PHASE 1: TAKEOFF ---")
        change_mode(master, "STABILIZE")
        arm_drone(master)

        while True:
            alt = get_lidar_alt(master, blocking=True)
            print(f"\r  Alt: {alt:.2f}m / {TARGET_ALT}m", end="", flush=True)

            positions, annotated, _, _ = vision.process_frame(display=False)
            show(annotated,
                 f"TAKEOFF  Alt:{alt:.2f}m  Target:{TARGET_ALT}m",
                 vision, False)

            if alt >= TARGET_ALT:
                set_rc_override(master, throttle=THROTTLE_HOVER)
                log(f"\n[FC] Hover altitude reached: {alt:.2f}m")
                break

            set_rc_override(master, throttle=THROTTLE_CLIMB)
            time.sleep(0.1)

        ######################################################################
        # PHASE 2: FLY FORWARD ~5 METRES
        ######################################################################
        log(f"\n--- PHASE 2: FORWARD FLIGHT ({FORWARD_FLIGHT_TIME}s) ---")

        fwd_start = time.time()
        while (time.time() - fwd_start) < FORWARD_FLIGHT_TIME:
            elapsed   = time.time() - fwd_start
            remaining = FORWARD_FLIGHT_TIME - elapsed

            alt = get_lidar_alt(master)
            set_rc_override(master,
                            pitch=FORWARD_PITCH_PWM,
                            throttle=throttle_hold(alt))

            positions, annotated, _, _ = vision.process_frame(display=False)
            show(annotated,
                 f"FORWARD  {elapsed:.1f}s / {FORWARD_FLIGHT_TIME}s  "
                 f"remaining:{remaining:.1f}s  Alt:{alt:.2f}m",
                 vision, False)

            print(f"\r  Flying {elapsed:.1f}s  Alt:{alt:.2f}m",
                  end="", flush=True)
            time.sleep(0.1)

        # Stop forward motion — return pitch to neutral
        alt = get_lidar_alt(master)
        set_rc_override(master, throttle=throttle_hold(alt))
        log("\n[FC] Forward flight complete. Hovering.")

        ######################################################################
        # PHASE 3: ACQUIRE MARKER + CENTRE FOR 60 SECONDS
        ######################################################################
        log(f"\n--- PHASE 3: ACQUIRE + CENTRE ({CENTER_HOLD_TIME}s) ---")

        target_id            = None   # locked marker ID
        confirm_count        = 0      # consecutive detections before lock
        center_timer         = None   # when we first achieved lock
        last_correction_time = 0.0    # cooldown tracker

        while True:
            loop_start = time.time()

            alt = get_lidar_alt(master)
            thr = throttle_hold(alt)

            # process_frame returns (positions, annotated_frame, corners, ids)
            positions, annotated, corners, ids = vision.process_frame(display=False)

            if positions is None:
                # Frame grab failed — keep throttle alive
                set_rc_override(master, throttle=thr)
                time.sleep(0.05)
                continue

            # ── Acquisition mode ──────────────────────────────────────────────
            if target_id is None:
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

            # ── Centring mode ─────────────────────────────────────────────────
            else:
                pos = next((p for p in positions
                            if p.marker_id == target_id), None)

                if pos:
                    in_zone   = is_centered(pos, METER_DEADBAND)
                    time_held = time.time() - center_timer
                    time_left = CENTER_HOLD_TIME - time_held

                    now = time.time()
                    if not in_zone and (now - last_correction_time) >= CORRECTION_COOLDOWN:
                        # Marker outside deadband and cooldown expired — nudge
                        roll_pwm, pitch_pwm = compute_correction(pos)
                        set_rc_override(master,
                                        roll=roll_pwm,
                                        pitch=pitch_pwm,
                                        throttle=thr)
                        log(f"[Centre] x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                            f"roll:{roll_pwm} pitch:{pitch_pwm}  "
                            f"held:{time_held:.1f}s left:{time_left:.1f}s")
                        last_correction_time = now
                    else:
                        # In zone OR cooldown still active — hold neutral attitude
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
                    # Target temporarily lost — hold position
                    set_rc_override(master, throttle=thr)
                    status = f"MARKER LOST — holding  ID:{target_id}  Alt:{alt:.2f}m"
                    show(annotated, status, vision, False)

            # Rate limit the loop
            sleep_t = loop_dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        ######################################################################
        # PHASE 4: FINAL RE-CENTRE BEFORE LAND
        ######################################################################
        log(f"\n--- PHASE 4: FINAL CENTRE (deadband={LAND_DEADBAND*100:.1f}cm) ---")

        last_correction_time = 0.0   # reset cooldown for this phase

        while True:
            loop_start = time.time()

            alt = get_lidar_alt(master)
            thr = throttle_hold(alt)

            positions, annotated, _, _ = vision.process_frame(display=False)

            if positions is not None:
                pos = next((p for p in positions
                            if p.marker_id == target_id), None)
                if pos:
                    centered = is_centered(pos, LAND_DEADBAND)
                    if centered:
                        set_rc_override(master, throttle=thr)
                        status = (f"CENTRED  x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                                  f"→ LANDING")
                        show(annotated, status, vision, True)
                        log("[ArUco] Centred within land gate. Initiating land.")
                        break
                    else:
                        now = time.time()
                        if (now - last_correction_time) >= CORRECTION_COOLDOWN:
                            roll_pwm, pitch_pwm = compute_correction(pos)
                            set_rc_override(master,
                                            roll=roll_pwm,
                                            pitch=pitch_pwm,
                                            throttle=thr)
                            log(f"[LandGate] x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                                f"need<{LAND_DEADBAND:.3f}m  "
                                f"roll:{roll_pwm} pitch:{pitch_pwm}")
                            last_correction_time = now
                        else:
                            set_rc_override(master, throttle=thr)

                        status = (f"FINAL CENTRE  x:{pos.x:.3f}m y:{pos.y:.3f}m  "
                                  f"need<{LAND_DEADBAND*100:.1f}cm  Alt:{alt:.2f}m")
                        show(annotated, status, vision, False)
                else:
                    set_rc_override(master, throttle=thr)
                    status = f"FINAL CENTRE — target lost  Alt:{alt:.2f}m"
                    show(annotated, status, vision, False)
            else:
                set_rc_override(master, throttle=thr)
                status = f"FINAL CENTRE — no frame  Alt:{alt:.2f}m"
                show(None, status, vision, False)

            sleep_t = loop_dt - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

        ######################################################################
        # LAND
        ######################################################################
        log("\n[FC] Switching to LAND mode...")
        change_mode(master, "LAND")
        set_rc_override(master, roll=RC_CENTER, pitch=RC_CENTER,
                        throttle=0, yaw=RC_CENTER)   # release override

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
        vision.close()   # closes ZED camera — same call as standalone uav_vision test
        log("Mission 5 finalized.")


if __name__ == "__main__":
    main()
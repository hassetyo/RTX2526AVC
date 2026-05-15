"""
Challenge2_ugv.py  —  UGV CONTROLLER HOST  (Jetson / RPi on the robot)
=======================================================================
Runs on the machine connected to the UGV's flight controller (Pixhawk/ArduRover).

What it does:
  1. Opens the V2V radio bridge to receive telemetry from the camera host.
  2. Connects to the UGV via DroneKit exactly like Challenge1.
  3. Arms the UGV and enters GUIDED mode.
  4. Runs a closed-loop approach loop:
        a. Read latest telemetry → distance_m, bearing_deg, marker flags.
        b. If we have a valid fix on both markers:
             • Compute the heading error between UGV compass and required bearing.
             • If |error| > HEADING_TOLERANCE_DEG  → execute an in-place turn.
             • Drive forward one DRIVE_STEP_M burst.
        c. Repeat until distance_m < ARRIVAL_THRESHOLD_M.
  5. Stops and disarms.

Coordinate convention (must match Challenge2_camera.py):
  bearing_deg = 0   → camera image UP direction
  bearing_deg = 90  → camera image RIGHT direction
  The camera should be oriented so image-UP aligns with a known compass
  bearing (e.g. north).  Set CAMERA_NORTH_DEG to that compass bearing.
  Example: if camera image-up points East, set CAMERA_NORTH_DEG = 90.

Telemetry slot mapping (TELEM_FMT "<IIffBB"):
   vx     → distance_m   (metres to target; -1 = no valid fix)
   vy     → bearing_deg  (CW degrees in camera frame)
   marker → bitmask  0b01=UGV seen, 0b10=target seen, 0b11=both seen
   estop  → 1 = camera host wants us to stop immediately
"""

import time
import math
import sys

from dronekit import connect, VehicleMode
from pymavlink import mavutil

sys.path.insert(0, ".")
from v2v_bridge import V2VBridge

# ════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ════════════════════════════════════════════════════════════════════════════

# Serial ports
UGV_CONTROL_PORT  = "/dev/ttyACM0"   # flight-controller USB on UGV host
UGV_BAUD_RATE     = 115200

RADIO_PORT        = "/dev/ttyACM1"   # ESP32 radio USB on UGV host
RADIO_BAUD_RATE   = 115200

# Camera orientation
# The bearing the camera sends is relative to camera-image-UP.
# CAMERA_NORTH_DEG is the COMPASS bearing that camera-image-UP represents.
# Set to 0 if camera-up == magnetic north, 90 if it == east, etc.
CAMERA_NORTH_DEG  = 0.0

# Approach loop tuning
ARRIVAL_THRESHOLD_M   = 0.25    # stop when this close (metres)
DRIVE_STEP_M          = 0.30    # forward burst per loop iteration (metres)
DRIVE_SPEED_MPS       = 0.35    # (≈ 0.8 mph, matches Challenge 1 spirit)
HEADING_TOLERANCE_DEG = 8.0     # turn only if error is bigger than this
STALE_TELEM_TIMEOUT_S = 2.0     # treat telemetry as stale after this many seconds
MAX_WAIT_FOR_FIX_S    = 10.0    # abort if we can't see both markers for this long
MAX_ITERATIONS        = 200     # hard limit so we never loop forever

# Turn tuning (re-used from Challenge1 logic)
TURN_RATE_DEG_S       = 10.0
TURN_TOLERANCE_DEG    = 5.0
STOP_EARLY_DEG        = 5.0
HEADING_CHECK_INTERVAL_S = 0.20
STABLE_COUNT_REQUIRED    = 2

# Velocity control
MOVEMENT_EPS_MPS      = 0.05
DETECTION_WINDOW_S    = 1.5

# ════════════════════════════════════════════════════════════════════════════
#  DRONEKIT HELPERS  (mostly lifted from Challenge1 for consistency)
# ════════════════════════════════════════════════════════════════════════════

def wait_for_mode(vehicle, mode_name, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while vehicle.mode.name != mode_name and time.time() < deadline:
        time.sleep(0.1)
    return vehicle.mode.name == mode_name


def wait_for_armed(vehicle, armed_state, timeout_s=3.0):
    deadline = time.time() + timeout_s
    while vehicle.armed != armed_state and time.time() < deadline:
        time.sleep(0.1)
    return vehicle.armed == armed_state


def arm_ugv(vehicle):
    if not vehicle.is_armable:
        print("Warning: vehicle reports not armable; attempting arm anyway.")
    for label, state in (
        ("FIRST ARM", True),
        ("RESET DISARM", False),
        ("FINAL ARM", True),
    ):
        print(f"  {label} in mode {vehicle.mode.name}...")
        vehicle.armed = state
        if not wait_for_armed(vehicle, state):
            raise RuntimeError(f"Failed to set armed={state}")
        time.sleep(1.0)

    print(f"  Switching {vehicle.mode.name} → GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    if not wait_for_mode(vehicle, "GUIDED"):
        raise RuntimeError(f"Could not enter GUIDED mode (still {vehicle.mode.name})")
    print("  Armed and in GUIDED mode.")


def build_velocity_msg(vehicle, speed_mps):
    return vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0x0DE7,
        0, 0, 0,
        speed_mps, 0, 0,
        0, 0, 0, 0, 0,
    )


def build_attitude_msg(vehicle, throttle_fraction, yaw_rate_deg_s=0.0):
    return vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0xA3,
        [1.0, 0.0, 0.0, 0.0],
        0.0, 0.0,
        math.radians(yaw_rate_deg_s),
        throttle_fraction,
    )


def send_stop(vehicle):
    vehicle.send_mavlink(build_velocity_msg(vehicle, 0.0))
    time.sleep(0.5)


def get_heading(vehicle):
    h = vehicle.heading
    if h is None:
        raise RuntimeError("vehicle.heading is unavailable")
    return float(h)


def angle_diff_deg(current_deg, start_deg):
    """Smallest signed angle from start_deg to current_deg. Positive = CW."""
    return ((current_deg - start_deg + 540) % 360) - 180


# ════════════════════════════════════════════════════════════════════════════
#  TURN FUNCTIONS  (identical logic to Challenge1, just broken out cleanly)
# ════════════════════════════════════════════════════════════════════════════

def _turn(vehicle, angle_deg, clockwise):
    """Generic turn. clockwise=True → right, clockwise=False → left."""
    if abs(angle_deg) < 1.0:
        return

    direction_sign = 1 if clockwise else -1
    dir_label      = "RIGHT" if clockwise else "LEFT"
    target_change  = abs(angle_deg)
    stop_target    = target_change - STOP_EARLY_DEG

    start_heading  = get_heading(vehicle)
    turn_msg = build_attitude_msg(
        vehicle,
        throttle_fraction=0.0,
        yaw_rate_deg_s=direction_sign * TURN_RATE_DEG_S
    )

    stable_count = 0
    last_print   = 0.0

    print(f"  TURN {dir_label}: start={start_heading:.1f}° "
          f"target={direction_sign * target_change:+.1f}° "
          f"stop_at={direction_sign * stop_target:+.1f}°")

    while True:
        vehicle.send_mavlink(turn_msg)
        time.sleep(HEADING_CHECK_INTERVAL_S)

        current = get_heading(vehicle)
        delta   = angle_diff_deg(current, start_heading)

        now = time.time()
        if now - last_print >= 0.4:
            print(f"    heading={current:.1f}° Δ={delta:+.1f}°")
            last_print = now

        reached = (delta >= (stop_target - TURN_TOLERANCE_DEG)) if clockwise \
             else (delta <= -(stop_target - TURN_TOLERANCE_DEG))
        stable_count = (stable_count + 1) if reached else 0

        if stable_count >= STABLE_COUNT_REQUIRED:
            break

    send_stop(vehicle)
    time.sleep(0.6)
    final = get_heading(vehicle)
    print(f"  TURN {dir_label} done: final={final:.1f}°  Δ={angle_diff_deg(final, start_heading):+.1f}°")


def turn_to_heading(vehicle, target_compass_deg):
    """
    Turn the UGV to face target_compass_deg (absolute compass bearing).
    Chooses shortest direction (CW or CCW).
    """
    current = get_heading(vehicle)
    error   = angle_diff_deg(target_compass_deg, current)   # signed

    print(f"  turn_to_heading: current={current:.1f}° target={target_compass_deg:.1f}° error={error:+.1f}°")

    if abs(error) <= HEADING_TOLERANCE_DEG:
        print("  Already on heading — no turn needed.")
        return

    if error > 0:
        _turn(vehicle, error, clockwise=True)
    else:
        _turn(vehicle, abs(error), clockwise=False)


# ════════════════════════════════════════════════════════════════════════════
#  DRIVE FUNCTIONS
# ════════════════════════════════════════════════════════════════════════════

def drive_step(vehicle, distance_m, speed_mps):
    """
    Drive forward distance_m at speed_mps using velocity control.
    Returns True if movement was detected, False otherwise.
    """
    duration_s    = distance_m / speed_mps
    drive_msg     = build_velocity_msg(vehicle, speed_mps)
    stop_msg      = build_velocity_msg(vehicle, 0.0)
    start_t       = time.time()
    movement_seen = False

    print(f"  Drive {distance_m:.2f}m @ {speed_mps:.2f}m/s "
          f"(≈{duration_s:.1f}s)  armed={vehicle.armed} mode={vehicle.mode.name}")

    while (time.time() - start_t) < duration_s:
        vehicle.send_mavlink(drive_msg)
        gs = vehicle.groundspeed or 0.0
        if gs >= MOVEMENT_EPS_MPS:
            movement_seen = True
        if (time.time() - start_t) >= DETECTION_WINDOW_S and not movement_seen:
            print("  No movement detected — aborting step.")
            break
        time.sleep(0.1)

    vehicle.send_mavlink(stop_msg)
    time.sleep(0.5)
    return movement_seen


# ════════════════════════════════════════════════════════════════════════════
#  TELEMETRY HELPERS
# ════════════════════════════════════════════════════════════════════════════

def camera_bearing_to_compass(camera_bearing_deg):
    """
    Convert the bearing the camera sends (CW from camera-frame UP)
    to an absolute compass bearing by adding the known camera orientation.
    """
    return (camera_bearing_deg + CAMERA_NORTH_DEG) % 360.0


def wait_for_valid_telemetry(bridge, timeout_s):
    """
    Block until we receive a telemetry frame where BOTH markers are visible
    (marker bits == 0b11) and distance is positive.
    Returns (distance_m, bearing_deg) or raises RuntimeError on timeout.
    """
    deadline = time.time() + timeout_s
    print(f"  Waiting up to {timeout_s:.0f}s for valid marker fix...")
    while time.time() < deadline:
        telem = bridge.get_telemetry()   # non-consuming peek would be better,
                                         # but this is fine at 10 Hz camera rate
        if telem is not None:
            _, _, dist, bearing, marker_bits, estop = telem
            if (marker_bits & 0b11) == 0b11 and dist > 0.0:
                print(f"  Got fix: dist={dist:.3f}m  bearing={bearing:.1f}°")
                return dist, bearing
        time.sleep(0.05)
    raise RuntimeError("Timed out waiting for valid ArUco fix from camera.")


def get_latest_fix(bridge):
    """
    Return (distance_m, bearing_deg, valid, estop) from the latest telemetry.
    valid=True only if both markers are visible and distance is positive.
    """
    telem = bridge.get_telemetry()
    if telem is None:
        return None, None, False, False

    _, t_ms, dist, bearing, marker_bits, estop = telem
    both_seen = (marker_bits & 0b11) == 0b11
    valid     = both_seen and dist > 0.0
    return dist, bearing, valid, bool(estop)


# ════════════════════════════════════════════════════════════════════════════
#  MAIN APPROACH LOOP
# ════════════════════════════════════════════════════════════════════════════

def approach_loop(vehicle, bridge):
    """
    Closed-loop approach: turn to face target, drive a step, repeat.
    Exits when within ARRIVAL_THRESHOLD_M or MAX_ITERATIONS exceeded.
    """
    print("\n── Starting closed-loop approach ──")

    for iteration in range(1, MAX_ITERATIONS + 1):
        # ── 1. Get a fresh fix (wait a bit if stale) ───────────────────────
        dist, bearing, valid, estop = get_latest_fix(bridge)

        if estop:
            print("ESTOP received from camera host — stopping immediately.")
            send_stop(vehicle)
            return

        if not valid:
            print(f"[iter {iteration}] No valid fix — waiting up to "
                  f"{STALE_TELEM_TIMEOUT_S:.1f}s...")
            try:
                dist, bearing = wait_for_valid_telemetry(
                    bridge, STALE_TELEM_TIMEOUT_S
                )
            except RuntimeError as e:
                print(f"  {e}  Aborting approach.")
                send_stop(vehicle)
                return

        print(f"\n[iter {iteration}]  dist={dist:.3f}m  "
              f"cam_bearing={bearing:.1f}°  "
              f"ugv_heading={get_heading(vehicle):.1f}°")

        # ── 2. Check arrival ────────────────────────────────────────────────
        if dist <= ARRIVAL_THRESHOLD_M:
            print(f"  ✓ Arrived within {ARRIVAL_THRESHOLD_M:.2f}m threshold!")
            send_stop(vehicle)
            return

        # ── 3. Compute required compass heading ─────────────────────────────
        required_compass = camera_bearing_to_compass(bearing)
        print(f"  Required compass heading: {required_compass:.1f}°")

        # ── 4. Turn to face target ───────────────────────────────────────────
        turn_to_heading(vehicle, required_compass)

        # ── 5. Re-check distance after the turn (heading may have changed) ──
        time.sleep(0.3)   # let the bridge queue refresh
        dist_after, bearing_after, valid_after, _ = get_latest_fix(bridge)
        if valid_after:
            dist = dist_after
            print(f"  Post-turn fix: dist={dist:.3f}m  bearing={bearing_after:.1f}°")
            if dist <= ARRIVAL_THRESHOLD_M:
                print(f"  ✓ Already within threshold after turn — done.")
                send_stop(vehicle)
                return

        # ── 6. Drive forward one step (but not more than remaining distance) ─
        step = min(DRIVE_STEP_M, max(dist - ARRIVAL_THRESHOLD_M, 0.05))
        moved = drive_step(vehicle, step, DRIVE_SPEED_MPS)
        if not moved:
            print("  Drive step returned no movement — check UGV state.")

    print(f"Reached MAX_ITERATIONS ({MAX_ITERATIONS}) — stopping.")
    send_stop(vehicle)


# ════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("UGV Challenge 2: Vision-Guided Approach to ArUco Target")
    print("=" * 60)
    print(f"UGV port  : {UGV_CONTROL_PORT}  @ {UGV_BAUD_RATE}")
    print(f"Radio port: {RADIO_PORT}  @ {RADIO_BAUD_RATE}")
    print(f"Arrival threshold : {ARRIVAL_THRESHOLD_M:.2f}m")
    print(f"Drive step        : {DRIVE_STEP_M:.2f}m")
    print(f"Camera north offset: {CAMERA_NORTH_DEG:.1f}°")
    print()

    # ── open radio bridge first so telemetry starts buffering ───────────────
    bridge = V2VBridge(RADIO_PORT, RADIO_BAUD_RATE, name="UGV")
    bridge.connect()
    time.sleep(0.5)

    # ── connect to UGV ───────────────────────────────────────────────────────
    print(f"Connecting to UGV at {UGV_CONTROL_PORT}...")
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)

    try:
        print(f"Initial state: armed={vehicle.armed}  mode={vehicle.mode.name}  "
              f"armable={vehicle.is_armable}")

        # ── wait for a valid initial fix before arming ───────────────────────
        print("\nWaiting for initial camera fix before arming...")
        wait_for_valid_telemetry(bridge, MAX_WAIT_FOR_FIX_S)

        # ── arm ──────────────────────────────────────────────────────────────
        arm_ugv(vehicle)

        # ── run the approach loop ────────────────────────────────────────────
        approach_loop(vehicle, bridge)

        time.sleep(1.0)
        print("\nChallenge 2 complete.  Disarming...")

    finally:
        # Always disarm and clean up no matter what went wrong
        try:
            send_stop(vehicle)
            vehicle.armed = False
            wait_for_armed(vehicle, False)
            print("UGV disarmed.")
        except Exception as e:
            print(f"Disarm error (non-fatal): {e}")
        vehicle.close()
        bridge.stop()
        print("Done.")


if __name__ == "__main__":
    main()

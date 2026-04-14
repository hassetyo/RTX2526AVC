from dronekit import connect, VehicleMode
import time
import math
import v2v_bridge
from pymavlink import mavutil

# ==========================================
# UGV GROUND STATION
# - Arms rover at startup
# - Forces GUIDED mode
# - Accepts bridge commands for turn/move/stop
# ==========================================

UGV_BAUD_RATE = 115200
UGV_CONTROL_PORT = "/dev/ttyACM0"
ESP32_BRIDGE_PORT = "/dev/ttyUSB0"

SPEED_MPS = 1.5
TELEM_SEND_HZ = 10

TURN_YAW_RATE_DEG = 30.0
MIN_MOVE_TIME_SEC = 0.15
GOTO_BODY_Y_EPS = 1e-3

print("==========================================")
print("   UGV GROUND STATION - MISSION READY")
print("==========================================")
print(f"[Ground] Connecting to UGV at {UGV_CONTROL_PORT}...")

try:
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=UGV_BAUD_RATE)
    print("[Ground] Connected! Ready to sync.")
except Exception as e:
    print(f"!!! Error connecting to UGV: {e} !!!")
    raise SystemExit(1)


class ObstacleDetector:
    def __init__(self):
        self.safe_distance_m = 1.0

    def check_for_buckets(self):
        return False, 0, 0


def log_avoidance(text):
    timestamp = time.strftime("%H:%M:%S")
    with open("ugv_mission3_avoidance.txt", "a") as f:
        f.write(f"[{timestamp}] [AVOIDANCE] {text}\n")
    print(f"[{timestamp}] [AVOIDANCE] {text}")


def wait_for_armed(vehicle, desired_state, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if vehicle.armed == desired_state:
            return True
        time.sleep(0.1)
    return vehicle.armed == desired_state


def wait_for_mode(vehicle, mode_name, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if vehicle.mode.name == mode_name:
            return True
        time.sleep(0.1)
    return vehicle.mode.name == mode_name


def force_guided(vehicle):
    if vehicle.mode.name != "GUIDED":
        print(f"[Ground] Switching mode {vehicle.mode.name} -> GUIDED...")
        vehicle.mode = VehicleMode("GUIDED")
        if not wait_for_mode(vehicle, "GUIDED", timeout_s=5.0):
            raise RuntimeError(f"Failed to enter GUIDED mode. Current mode: {vehicle.mode.name}")
    print("[Ground] GUIDED mode confirmed.")


def arm_ugv(vehicle):
    print("\n[Ground] >>> ARMING UGV")

    # Optional hybrid sequence because some rover setups are stubborn
    for label, state in [
        ("FIRST ARM", True),
        ("RESET DISARM", False),
        ("FINAL ARM", True),
    ]:
        print(f"[Ground] {label}...")
        vehicle.armed = state
        if not wait_for_armed(vehicle, state, timeout_s=5.0):
            raise RuntimeError(f"Failed to set armed={state}")
        time.sleep(1.0)

    print("[Ground] Armed confirmed.")
    force_guided(vehicle)
    print("[Ground] UGV FULLY ARMED AND IN GUIDED MODE.\n")


def ensure_ready(vehicle):
    if not vehicle.armed:
        print("[Ground] Vehicle not armed. Re-arming...")
        arm_ugv(vehicle)
    elif vehicle.mode.name != "GUIDED":
        print("[Ground] Vehicle not in GUIDED. Forcing GUIDED...")
        force_guided(vehicle)


def broadcast_status(bridge, seq):
    armed_val = 1 if vehicle.armed else 0
    m = vehicle.mode.name
    mode_idx = v2v_bridge.MODE_INITIAL

    if m == "GUIDED":
        mode_idx = v2v_bridge.MODE_GUIDED
    elif m == "AUTO":
        mode_idx = v2v_bridge.MODE_AUTO
    elif m == "LAND":
        mode_idx = v2v_bridge.MODE_LAND

    armable_bit = 0x10 if vehicle.is_armable else 0x00
    gps_bit = 0x20 if (vehicle.gps_0.fix_type > 0) else 0x00
    safety_byte = (mode_idx & 0x0F) | armable_bit | gps_bit

    t_ms = int(time.time() * 1000) & 0xFFFFFFFF
    v_mps = vehicle.groundspeed if vehicle.groundspeed is not None else 0.0

    bridge.send_telemetry(seq, t_ms, v_mps, 0.0, armed_val, safety_byte)


def build_stop_msg():
    return vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111111111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def build_body_velocity_msg(vx_mps=0.0, vy_mps=0.0):
    return vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx_mps, vy_mps, 0,
        0, 0, 0,
        0, 0
    )


def build_turn_msg(direction):
    # direction: +1 = right, -1 = left
    # final two fields are yaw, yaw_rate
    return vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111111111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0.0,
        math.radians(TURN_YAW_RATE_DEG * direction)
    )


def build_local_ned_goto_msg(x, y):
    return vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        x, y, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def execute_drive(distance_m):
    ensure_ready(vehicle)

    print(f"[Ground] DRIVE: {distance_m:.3f}m at {SPEED_MPS:.2f}m/s")
    direction = 1.0 if distance_m >= 0 else -1.0
    msg = build_body_velocity_msg(vx_mps=direction * SPEED_MPS, vy_mps=0.0)

    start_t = time.time()
    duration = max(abs(distance_m) / SPEED_MPS, MIN_MOVE_TIME_SEC)

    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

    vehicle.send_mavlink(build_stop_msg())
    time.sleep(0.3)


def execute_turn(angle_deg):
    ensure_ready(vehicle)

    print(f"[Ground] >>> EXECUTING TIMED TURN {angle_deg} DEGREES")
    direction = 1 if angle_deg > 0 else -1
    msg = build_turn_msg(direction)
    duration = abs(angle_deg) / TURN_YAW_RATE_DEG

    start_t = time.time()
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

    vehicle.send_mavlink(build_stop_msg())
    time.sleep(0.5)


def execute_drive_forever(speed_mps):
    ensure_ready(vehicle)
    print(f"[Ground] >>> DRIVING STRAIGHT AT {speed_mps} m/s")
    return build_body_velocity_msg(vx_mps=speed_mps, vy_mps=0.0)


def execute_circle(speed, yaw_rate_deg, circles=1):
    ensure_ready(vehicle)

    print(f"[Ground] CIRCLE: Speed {speed}m/s | Yaw Rate {yaw_rate_deg}deg/s | count {circles}")
    duration = (360.0 / abs(yaw_rate_deg)) * circles

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0x05C7,
        0, 0, 0,
        speed, 0, 0,
        0, 0, 0,
        0.0,
        math.radians(yaw_rate_deg)
    )

    start_t = time.time()
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

    vehicle.send_mavlink(build_stop_msg())
    time.sleep(0.3)


def execute_gps_denied_goto(x, y, avoidance_mode=False):
    ensure_ready(vehicle)

    print(f"[Ground] >>> GOTO RELATIVE: X={x}m, Y={y}m (Avoidance: {avoidance_mode})")

    if not avoidance_mode:
        msg = build_local_ned_goto_msg(x, y)
        vehicle.send_mavlink(msg)
    else:
        detector = ObstacleDetector()
        found, angle, dist = detector.check_for_buckets()

        if found:
            log_avoidance(f"Obstacle Detected at {dist}m, Angle {angle}!")
            log_avoidance("Path Decision: Re-routing right (90 deg pivot)")
            execute_turn(90)
            execute_drive(1.5)
            log_avoidance("Avoidance Maneuver Complete. Returning to Target Path.")

        msg = build_local_ned_goto_msg(x, y)
        vehicle.send_mavlink(msg)


def queue_body_move_from_goto(x_val, y_val):
    if abs(y_val) <= GOTO_BODY_Y_EPS:
        duration = max(abs(x_val) / SPEED_MPS, MIN_MOVE_TIME_SEC)
        vx = SPEED_MPS if x_val >= 0 else -SPEED_MPS
        msg = build_body_velocity_msg(vx_mps=vx, vy_mps=0.0)
        print(f"[Ground] >>> BODY FORWARD MOVE from GOTO: x={x_val:.3f}m duration={duration:.2f}s")
        return msg, time.time() + duration, True

    return None, 0.0, False


def main():
    bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="UGV-Bridge")

    try:
        # Arm immediately on startup and force GUIDED
        arm_ugv(vehicle)

        bridge.connect()
        bridge.send_message("ground station is live . awaiting drone orders")
    except Exception as e:
        print(f"!!! [ERROR] Startup failed: {e}")
        return

    seq = 0
    mission_active = False
    mission_3_avoidance = False
    drive_msg = None

    active_turn_msg = None
    active_drive_msg = None
    active_drive_until = 0.0

    try:
        while True:
            broadcast_status(bridge, seq)
            seq += 1

            msg_str = bridge.get_message()
            if msg_str:
                print(f"[Ground] Incoming Shout: {msg_str}")
                if msg_str.startswith("GOTO:"):
                    try:
                        _, coords = msg_str.split(":")
                        x_val, y_val = map(float, coords.split(","))

                        ensure_ready(vehicle)

                        active_turn_msg = None
                        mission_active = False
                        drive_msg = None

                        queued_msg, queued_until, handled_as_body = queue_body_move_from_goto(x_val, y_val)
                        if handled_as_body:
                            active_drive_msg = queued_msg
                            active_drive_until = queued_until
                        else:
                            active_drive_msg = None
                            active_drive_until = 0.0
                            execute_gps_denied_goto(
                                x_val,
                                y_val,
                                avoidance_mode=mission_3_avoidance
                            )
                    except Exception as e:
                        print(f"!!! [ERROR] Failed to parse coordinates: {e} !!!")

            cmd = bridge.get_command()
            if cmd:
                cmdSeq, cmdVal, eStop = cmd
                print(f"[Ground] Got Choice Order: {cmdVal}")

                if eStop == 1:
                    print("!!! [ABORT] EMERGENCY DISARM !!!")
                    mission_active = False
                    drive_msg = None
                    active_turn_msg = None
                    active_drive_msg = None
                    active_drive_until = 0.0
                    vehicle.send_mavlink(build_stop_msg())
                    vehicle.armed = False
                    continue

                if cmdVal == v2v_bridge.CMD_ARM:
                    arm_ugv(vehicle)
                    continue

                elif cmdVal == v2v_bridge.CMD_DISARM:
                    vehicle.send_mavlink(build_stop_msg())
                    vehicle.armed = False
                    continue

                ensure_ready(vehicle)

                if cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                    active_turn_msg = None
                    active_drive_msg = build_body_velocity_msg(vx_mps=SPEED_MPS, vy_mps=0.0)
                    active_drive_until = time.time() + max(3.048 / SPEED_MPS, MIN_MOVE_TIME_SEC)
                    print("[Ground] >>> MOVE FORWARD STARTED")

                elif cmdVal == v2v_bridge.CMD_MOVE_2FT:
                    active_turn_msg = None
                    active_drive_msg = build_body_velocity_msg(vx_mps=SPEED_MPS, vy_mps=0.0)
                    active_drive_until = time.time() + max(0.61 / SPEED_MPS, MIN_MOVE_TIME_SEC)
                    print("[Ground] >>> MOVE 2FT STARTED")

                elif cmdVal == v2v_bridge.CMD_TURN_RIGHT:
                    mission_active = False
                    drive_msg = None
                    active_drive_msg = None
                    active_drive_until = 0.0
                    active_turn_msg = build_turn_msg(+1)
                    print("[Ground] >>> CONTINUOUS TURN RIGHT STARTED")

                elif cmdVal == v2v_bridge.CMD_TURN_LEFT:
                    mission_active = False
                    drive_msg = None
                    active_drive_msg = None
                    active_drive_until = 0.0
                    active_turn_msg = build_turn_msg(-1)
                    print("[Ground] >>> CONTINUOUS TURN LEFT STARTED")

                elif cmdVal == v2v_bridge.CMD_CIRCLE:
                    active_turn_msg = None
                    active_drive_msg = None
                    active_drive_until = 0.0
                    execute_circle(1.0, 45, circles=2)

                elif cmdVal in (
                    v2v_bridge.CMD_MISSION_1,
                    v2v_bridge.CMD_MISSION_2,
                    v2v_bridge.CMD_MISSION_3,
                ):
                    active_turn_msg = None
                    active_drive_msg = None
                    active_drive_until = 0.0
                    mission_active = True
                    drive_msg = execute_drive_forever(0.15)
                    if cmdVal == v2v_bridge.CMD_MISSION_3:
                        mission_3_avoidance = True
                        log_avoidance("Mission 3 Active: Obstacle Avoidance Engaged.")

                elif cmdVal == v2v_bridge.CMD_STOP:
                    mission_active = False
                    mission_3_avoidance = False
                    drive_msg = None
                    active_turn_msg = None
                    active_drive_msg = None
                    active_drive_until = 0.0
                    vehicle.send_mavlink(build_stop_msg())
                    print("[Ground] >>> STOPPED")

            now = time.time()

            if active_turn_msg is not None:
                ensure_ready(vehicle)
                vehicle.send_mavlink(active_turn_msg)

            elif active_drive_msg is not None:
                if now < active_drive_until:
                    ensure_ready(vehicle)
                    vehicle.send_mavlink(active_drive_msg)
                else:
                    active_drive_msg = None
                    active_drive_until = 0.0
                    vehicle.send_mavlink(build_stop_msg())
                    print("[Ground] >>> DRIVE STEP COMPLETE")

            elif mission_active and drive_msg is not None:
                ensure_ready(vehicle)
                vehicle.send_mavlink(drive_msg)

            time.sleep(1.0 / TELEM_SEND_HZ)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            vehicle.send_mavlink(build_stop_msg())
        except Exception:
            pass
        bridge.stop()
        vehicle.close()


if __name__ == "__main__":
    main()
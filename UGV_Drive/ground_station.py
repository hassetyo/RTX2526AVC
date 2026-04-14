from dronekit import connect, VehicleMode
import time
import math
import v2v_bridge
from pymavlink import mavutil

# UGV GROUND STATION SCRIPT
# Patched for Challenge 2 visual-turn workflow:
# - TURN_LEFT / TURN_RIGHT are continuous turn commands until CMD_STOP
# - GOTO:x,0 is treated as a body-forward timed move
# - legacy LOCAL_NED goto is still available when y != 0

############## config stuff
UGV_CONTROL_PORT = "/dev/ttyACM0"   # Pixhawk/Cube connection
ESP32_BRIDGE_PORT = "/dev/ttyUSB0"  # ESP32 bridge

# drive settings
DIST_M = 3.048
SPEED_MPS = 1.5
TELEM_SEND_HZ = 5

# patched motion settings
TURN_YAW_RATE_DEG = 35.0
MIN_MOVE_TIME_SEC = 0.15
GOTO_BODY_Y_EPS = 1e-3

print("==========================================")
print("   UGV GROUND STATION - MISSION READY")
print("==========================================")
print(f"[Ground] Connecting to UGV at {UGV_CONTROL_PORT}...")

try:
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=115200)
    print(f"[Ground] Connected! Ready to sync.")
except Exception as e:
    print(f"!!! Error connecting to UGV: {e} !!!")
    exit()


######################### avoidance brain

class ObstacleDetector:
    def __init__(self):
        self.safe_distance_m = 1.0
        pass

    def check_for_buckets(self):
        return False, 0, 0


def log_avoidance(text):
    timestamp = time.strftime("%H:%M:%S")
    with open("ugv_mission3_avoidance.txt", "a") as f:
        f.write(f"[{timestamp}] [AVOIDANCE] {text}\n")
    print(f"[{timestamp}] [AVOIDANCE] {text}")


######################### status logic

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


def arm_and_sync(bridge):
    print("\n[Ground] >>> INITIATING HYBRID ARM SEQUENCE")
    if not vehicle.is_armable:
        print(f"!!! [WARNING] PRE-ARM CHECKS FAILED (GPS: {vehicle.gps_0.fix_type}) !!!")

    for attempt in ["FIRST ARM", "RESET DISARM", "FINAL ARM"]:
        state = True if "ARM" in attempt else False
        print(f"  [FORCE-SYNC] Initiating {attempt} in {vehicle.mode.name} mode...")
        for retry in range(3):
            vehicle.armed = state
            timeout = time.time() + 3
            while vehicle.armed != state:
                if time.time() > timeout:
                    break
                broadcast_status(bridge, 0)
                time.sleep(0.1)
            if vehicle.armed == state:
                break
        time.sleep(1.0)

    if not vehicle.armed:
        return

    print(f"  [MODE] Switching {vehicle.mode.name} -> GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    m_timeout = time.time() + 5
    while vehicle.mode.name != "GUIDED" and time.time() < m_timeout:
        broadcast_status(bridge, 0)
        time.sleep(0.1)

    print("!!! UGV FULLY ARMED AND SYNCED !!!\n")


####### MAVLink message builders

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
    return vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000101111111111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        math.radians(TURN_YAW_RATE_DEG * direction),
        0
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


####### execution helpers

def execute_drive(bridge, distance_m):
    print(f"[Ground] DRIVE: {distance_m}m at {SPEED_MPS}m/s")
    direction = 1.0 if distance_m >= 0 else -1.0
    msg = build_body_velocity_msg(vx_mps=direction * SPEED_MPS, vy_mps=0.0)

    start_t = time.time()
    duration = max(abs(distance_m) / SPEED_MPS, MIN_MOVE_TIME_SEC)
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        broadcast_status(bridge, 0)
        time.sleep(0.1)

    vehicle.send_mavlink(build_stop_msg())
    time.sleep(0.5)


def execute_turn(bridge, angle_deg):
    # kept for backward compatibility with old paths
    print(f"[Ground] >>> EXECUTING LEGACY TIMED TURN {angle_deg} DEGREES")
    yaw_rate = TURN_YAW_RATE_DEG
    direction = 1 if angle_deg > 0 else -1
    msg = build_turn_msg(direction)

    duration = abs(angle_deg) / yaw_rate
    start_t = time.time()
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        broadcast_status(bridge, 0)
        time.sleep(0.1)

    vehicle.send_mavlink(build_stop_msg())
    time.sleep(1.0)


def execute_drive_forever(bridge, speed_mps):
    print(f"[Ground] >>> MISSION 1: DRIVING STRAIGHT AT {speed_mps} m/s")
    msg = build_body_velocity_msg(vx_mps=speed_mps, vy_mps=0.0)
    return msg


def execute_circle(bridge, speed, yaw_rate_deg, circles=1):
    print(f"[Ground] CIRCLE: Speed {speed}m/s | Yaw Rate {yaw_rate_deg}deg/s | count {circles}")
    duration = (360.0 / abs(yaw_rate_deg)) * circles

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0x05C7,
        0, 0, 0,
        speed, 0, 0,
        0, 0, 0,
        math.radians(yaw_rate_deg)
    )

    start_t = time.time()
    while (time.time() - start_t) < duration:
        vehicle.send_mavlink(msg)
        broadcast_status(bridge, 0)
        time.sleep(0.1)

    vehicle.send_mavlink(build_stop_msg())
    time.sleep(0.5)


def execute_gps_denied_goto(bridge, x, y, avoidance_mode=False):
    # legacy LOCAL_NED goto kept for compatibility
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
            execute_turn(bridge, 90)
            execute_drive(bridge, 1.5)
            log_avoidance("Avoidance Maneuver Complete. Returning to Target Path.")

        msg = build_local_ned_goto_msg(x, y)
        vehicle.send_mavlink(msg)


def queue_body_move_from_goto(x_val, y_val):
    """
    Challenge 2 patch:
    - If y == 0, treat GOTO:x,0 as a body-forward/back timed move.
    - If y != 0, fall back to the old LOCAL_NED interpretation.
    """
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
        bridge.connect()
        bridge.send_message("ground station is live . awaiting drone orders")
    except:
        return

    seq = 0
    mission_active = False
    mission_3_avoidance = False
    drive_msg = None

    # patch state
    active_turn_msg = None
    active_drive_msg = None
    active_drive_until = 0.0

    try:
        while True:
            broadcast_status(bridge, seq)
            seq += 1

            # 1. raw string messages (GOTO)
            msg_str = bridge.get_message()
            if msg_str:
                print(f"[Ground] Incoming Shout: {msg_str}")
                if msg_str.startswith("GOTO:"):
                    try:
                        _, coords = msg_str.split(":")
                        x_val, y_val = map(float, coords.split(","))

                        if not vehicle.armed:
                            arm_and_sync(bridge)

                        # stop any active turn before moving
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
                                bridge,
                                x_val,
                                y_val,
                                avoidance_mode=mission_3_avoidance
                            )
                    except Exception:
                        print("!!! [ERROR] Failed to parse coordinates !!!")

            # 2. explicit commands
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

                if not vehicle.armed:
                    arm_and_sync(bridge)

                if cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                    active_turn_msg = None
                    active_drive_msg = build_body_velocity_msg(vx_mps=SPEED_MPS, vy_mps=0.0)
                    active_drive_until = time.time() + max(3.048 / SPEED_MPS, MIN_MOVE_TIME_SEC)

                elif cmdVal == v2v_bridge.CMD_MOVE_2FT:
                    active_turn_msg = None
                    active_drive_msg = build_body_velocity_msg(vx_mps=SPEED_MPS, vy_mps=0.0)
                    active_drive_until = time.time() + max(0.61 / SPEED_MPS, MIN_MOVE_TIME_SEC)

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
                    execute_circle(bridge, 1.0, 45, circles=2)

                elif cmdVal == v2v_bridge.CMD_MISSION_1 or cmdVal == v2v_bridge.CMD_MISSION_2 or cmdVal == v2v_bridge.CMD_MISSION_3:
                    active_turn_msg = None
                    active_drive_msg = None
                    active_drive_until = 0.0
                    mission_active = True
                    drive_msg = execute_drive_forever(bridge, 0.15)
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
                    if vehicle.armed:
                        log_avoidance("Destination Arrival / Manual Stop.")
                    vehicle.send_mavlink(build_stop_msg())
                    print("[Ground] >>> STOPPED")

            # 3. stream whatever is currently active
            now = time.time()

            if active_turn_msg is not None:
                vehicle.send_mavlink(active_turn_msg)

            elif active_drive_msg is not None:
                if now < active_drive_until:
                    vehicle.send_mavlink(active_drive_msg)
                else:
                    active_drive_msg = None
                    active_drive_until = 0.0
                    vehicle.send_mavlink(build_stop_msg())

            elif mission_active and drive_msg:
                vehicle.send_mavlink(drive_msg)

            time.sleep(1.0 / TELEM_SEND_HZ)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            vehicle.send_mavlink(build_stop_msg())
        except:
            pass
        bridge.stop()
        vehicle.close()


if __name__ == "__main__":
    main()
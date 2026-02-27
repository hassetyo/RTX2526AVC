from pymavlink import mavutil
import time
import math

# =========================
# CONFIG
# =========================
CONNECTION_STRING = '127.0.0.1:14551' #'COM3' #e.g. "COM6" on Windows, or "udp:127.0.0.1:14551" in SITL
BAUD_RATE = 57600

TARGET_DISTANCE_M = 5.0
SPEED_MPS = 0.6                
TURN_SPEED_MPS = 0.3           # Slightly slower for better turning precision
MAX_YAW_RATE = 0.6             # Maximum turn speed (rad/s)
MIN_YAW_RATE = 0.15            # Minimum speed to ensure motors keep moving
KP_YAW = 0.03                  # Proportional gain: higher = more aggressive turn

DIST_TOL_M = 0.10              
HEADING_TOL_DEG = 2.0          # Tightened tolerance thanks to Smart Stop
COMMAND_HZ = 10                
MOVE_TIMEOUT_S = 30            

USE_GROUNDSPEED_FALLBACK = True

# =========================
# CONNECT & INITIALIZE
# =========================
print(f"Connecting to {CONNECTION_STRING} at {BAUD_RATE} baud...")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
master.wait_heartbeat()
print(f"Heartbeat found (sys={master.target_system}, comp={master.target_component})")

def request_message_streams():
    try:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )
    except Exception: pass

def change_mode(mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping: raise RuntimeError(f"Unknown mode '{mode}'")
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[mode])
    time.sleep(1.0)

def arm():
    print("Arming...")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(2.0)

def disarm():
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

def send_body_velocity(vx_mps: float, yaw_rate_rads: float = 0.0):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask, 0, 0, 0, vx_mps, 0, 0, 0, 0, 0, 0, yaw_rate_rads
    )

def stop_vehicle():
    for _ in range(5):
        send_body_velocity(0.0, 0.0)
        time.sleep(0.1)

def get_vfr_hud(): return master.recv_match(type='VFR_HUD', blocking=True, timeout=0.2)
def get_local_position_ned(): return master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.2)

# =========================
# SMART LOGIC
# =========================

def turn_heading_smart(relative_angle_deg: float):
    """
    Turns the vehicle using Proportional control (Smart Stop).
    Automatically determines the shortest direction to turn.
    """
    request_message_streams()
    msg = get_vfr_hud()
    if msg is None: return
    
    start_heading = float(msg.heading)
    target_heading = (start_heading + relative_angle_deg) % 360
    print(f"Start Heading: {start_heading:.1f} | Target: {target_heading:.1f}")

    t0 = time.time()
    while True:
        msg = get_vfr_hud()
        if msg:
            current_heading = float(msg.heading)
            # Calculate shortest error (-180 to 180)
            error = (target_heading - current_heading + 180) % 360 - 180
            
            # Smart Stop: Calculate yaw rate proportional to error
            # target_yaw = error * gain (converts degrees error to rads/s)
            target_yaw_rate = error * KP_YAW
            
            # Clamp to Max/Min limits
            if abs(target_yaw_rate) > MAX_YAW_RATE:
                target_yaw_rate = math.copysign(MAX_YAW_RATE, target_yaw_rate)
            if abs(target_yaw_rate) < MIN_YAW_RATE:
                target_yaw_rate = math.copysign(MIN_YAW_RATE, target_yaw_rate)

            # Extra spaces at the end clear out old "degggg" artifacts
            print(f"\rHeading: {current_heading:.1f} | Error: {error:.1f} deg | Rate: {target_yaw_rate:.2f}    ", end="", flush=True)

            if abs(error) <= HEADING_TOL_DEG:
                print("\nTarget heading reached.")
                break
        
        send_body_velocity(TURN_SPEED_MPS, target_yaw_rate)
        if (time.time() - t0) > MOVE_TIMEOUT_S: break
        time.sleep(1.0 / COMMAND_HZ)
    stop_vehicle()

def move_forward_distance_smart(distance_m: float, speed_mps: float):
    request_message_streams()
    start_lp = get_local_position_ned()
    x0, y0 = (start_lp.x, start_lp.y) if start_lp else (0, 0)
    
    traveled = 0.0
    t0 = last_time = time.time()

    while traveled < (distance_m - DIST_TOL_M):
        now = time.time()
        send_body_velocity(speed_mps, 0.0)
        
        lp = get_local_position_ned()
        if lp and start_lp:
            traveled = math.sqrt((lp.x - x0)**2 + (lp.y - y0)**2)
        else:
            vfr = get_vfr_hud()
            if vfr: traveled += float(vfr.groundspeed) * (now - last_time)

        # Added spaces here too for clean output
        print(f"\rTraveled: {traveled:.2f} / {distance_m:.2f} m    ", end="", flush=True)
        if (now - t0) > MOVE_TIMEOUT_S: break
        last_time = now
        time.sleep(1.0 / COMMAND_HZ)
    
    print("\nDistance reached.")
    stop_vehicle()

# =========================
# MAIN EXECUTION
# =========================
if __name__ == "__main__":
    try:
        change_mode("GUIDED")
        arm()
        time.sleep(2)

        print(">>> STEP 1: FORWARD 5m <<<")
        move_forward_distance_smart(TARGET_DISTANCE_M, SPEED_MPS)

        print("\n>>> STEP 2: SMART TURN 180 <<<")
        turn_heading_smart(180) # Positive 180 is a right turn

        print("\n>>> STEP 3: RETURN 5m <<<")
        move_forward_distance_smart(TARGET_DISTANCE_M, SPEED_MPS)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        stop_vehicle()
    except Exception as e:
        print(f"\nERROR: {e}")
        try:
            stop_vehicle()
        except Exception:
            pass
    finally:
        try:
            disarm()
        except Exception as e:
            print(f"Disarm warning: {e}")
        print("Test complete.")

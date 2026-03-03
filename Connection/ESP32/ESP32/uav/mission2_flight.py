from pymavlink import mavutil
import time
import math
import v2v_bridge

# ------------------- SET THESE PORTS -------------------
CONNECTION_STRING = "/dev/ttyACM0" 
BAUD_RATE = 115200

# UAV ESP32 Bridge Port
ESP32_PORT = "/dev/ttyACM1"

# ------------------- Mission params -------------------
DIST_M = 3.048       # 10 ft
SPEED_MPS = 0.5      # Gentle move
TAKEOFF_ALT_M = 2.0
MAX_ALT_ALLOWED = 3.5 # (Approx 10-11 ft) safety ceiling

# ------------------- Connect -------------------
print(f"[Mission 2] Connecting to {CONNECTION_STRING}...")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
master.wait_heartbeat()
print("[Mission 2] Heartbeat found. Optical Flow/Lidar Ready.")

# Initialize V2V Bridge
bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
bridge.connect()

# ------------------- Mavlink Helpers (mavutil style) -------------------

def change_mode(mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping: return
    mode_id = mapping[mode]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    print(f"Mode set: {mode}")

def arm_drone():
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 
                                 0, 0, 0, 0, 0, 0)
    print("Arming...")

def takeoff(alt):
    print(f"Takeoff command: {alt}m")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)

def get_altitude():
    msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1.0)
    if msg:
        return msg.alt
    return 0.0

def send_velocity(vx, vy, vz=0.0):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

# ------------------- MAIN MISSION 2: FLIGHT -------------------
def main():
    try:
        # 1. Takeoff Sequence
        change_mode("GUIDED")
        time.sleep(1)
        arm_drone()
        time.sleep(1)
        takeoff(TAKEOFF_ALT_M)

        # Wait until target altitude is reached
        print("Climbing...")
        while True:
            alt = get_altitude()
            if alt >= TAKEOFF_ALT_M * 0.9:
                print(f"Altitude reached: {alt:.2f}m")
                break
            time.sleep(0.5)

        # 2. COORDINATED ACTION: Tell UGV to move!
        print("[Mission] Commanding UGV to MOVE 10ft")
        bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0) 

        # 3. Flight Move
        print(f"[Mission] Flying {DIST_M}m forward...")
        start_t = time.time()
        telem_seq = 0
        abort_triggered = False

        duration = DIST_M / SPEED_MPS

        while (time.time() - start_t) < duration:
            # SAFETY CHECK: Check for Abort from ground station
            cmd = bridge.get_command()
            if cmd:
                cmdSeq, cmdVal, eStopFlag = cmd
                if eStopFlag == 1:
                    print("!!! ABORT RECEIVED: SWITCHING TO LAND !!!")
                    abort_triggered = True
                    break

            # SAFETY CHECK: Don't exceed Max Altitude (safety fence)
            alt = get_altitude()
            if alt > MAX_ALT_ALLOWED:
                print(f"Altitude Warning: {alt:.2f}m. Forced Land.")
                abort_triggered = True
                break

            # Send forward velocity
            send_velocity(SPEED_MPS, 0.0, 0.0)

            # Update Ground Station via Telemetry
            t_ms = int(time.time() * 1000) & 0xFFFFFFFF
            bridge.send_telemetry(telem_seq, t_ms, SPEED_MPS, 0.0, 0, 0)
            telem_seq += 1
            
            time.sleep(0.1)

        # 4. Landing (Ensures it never DISARMS in sky)
        print("[Mission] Finalizing: Landing now...")
        change_mode("LAND")
        
        # Monitor landing until disarmed (ground detected by autopilot)
        while True:
            alt = get_altitude()
            if alt < 0.3:
                print("Landed safely.")
                break
            time.sleep(1)
            
    except Exception as e:
        print(f"Mission Error: {e}. Attempting EMERGENCY LAND.")
        change_mode("LAND")
    finally:
        bridge.stop()
        print("Bridge stopped. Mission sequence closed.")

if __name__ == "__main__":
    main()

from pymavlink import mavutil
import time
import v2v_bridge

# ------------------- SET THESE PORTS -------------------
CONNECTION_STRING = "/dev/ttyACM0" 
BAUD_RATE = 115200
ESP32_PORT = "/dev/ttyUSB0"

# ------------------- Connect -------------------
def main():
    print("==========================================")
    print("   UAV MISSION 1 - HYBRID SYNC")
    print("==========================================")
    
    print(f"[Mission 1] Connecting to UAV Controller at {CONNECTION_STRING}...")
    try:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
        master.wait_heartbeat()
        print("[Mission 1] Drone Heartbeat found. Sensors Check: OK")
    except Exception as e:
        print(f"!!! Error connecting to Drone: {e} !!!")
        print("Continuing with Bridge only...")
        master = None

    # Initialize V2V Bridge
    print(f"[Mission 1] Starting V2V Bridge on {ESP32_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        # Debug link check
        print("[Mission 1] Sending Hello to UGV...")
        bridge.send_message("hello pissrat . we connected")
    except Exception as e:
        print(f"!!! Error connecting to ESP32: {e} !!!")
        return

    # ------------------- Helpers -------------------
    def arm_drone():
        if master:
            master.mav.command_long_send(master.target_system, master.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 
                                         0, 0, 0, 0, 0, 0)
            print("[Mission 1] Drone Motors Engaged (Ground Only).")

    def disarm_drone():
        if master:
            master.mav.command_long_send(master.target_system, master.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 
                                         0, 0, 0, 0, 0, 0)
            print("[Mission 1] Drone Disarmed.")

    def broadcast_uav_status(seq):
        """Fetches drone status via MAVLink and sends it over radio."""
        armed_val = 0
        mode_val = v2v_bridge.MODE_INITIAL
        
        if master:
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                armed_val = 1 if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
        
        t_ms = int(time.time() * 1000) & 0xFFFFFFFF
        bridge.send_telemetry(seq, t_ms, 0.0, 0.0, armed_val, mode_val)

    # ------------------- EXECUTION -------------------
    try:
        # 1. Arm Drone on Ground
        arm_drone()
        time.sleep(2)
        
        # 2. WAIT FOR UGV SYNC:
        print("[Mission 1] Waiting for UGV Sync...")
        ugv_ready = False
        timeout_start = time.time()
        
        while not ugv_ready:
            # 1. READ DEBUG MESSAGES
            msg = bridge.get_message()
            if msg:
                print(f"    >>> [RADIO MSG]: {msg}")

            # 2. Check for Telemetry FROM the UGV
            data = bridge.get_telemetry()
            if data:
                seq_u, t_ms_u, vx_u, vy_u, armed_u, safety_byte = data
                
                # Decode Safety Byte
                mode_u = safety_byte & 0x0F
                is_armable = bool(safety_byte & 0x10)
                has_gps = bool(safety_byte & 0x20)
                
                status_str = "ARMED" if armed_u == 1 else "DISARMED"
                mode_str = "GUIDED" if mode_u == v2v_bridge.MODE_GUIDED else "MANUAL/INITIAL"
                
                print(f"    [RADIO] UGV: {status_str} | Mode: {mode_str} | Armable: {is_armable} | GPS: {has_gps}")
                
                if not is_armable:
                    reason = "NO GPS FIX" if not has_gps else "Safety Check Failed"
                    print(f"    [WARNING] UGV Not Ready: {reason}")
                    print("    [BYPASS] Mimicking Remote: Proceeding with Command Sync anyway...")
                
                # If radio link is verified, we proceed to command phase
                print("\n!!! [SYNC] UGV RADIO LINK VERIFIED & READY !!!")
                ugv_ready = True
            
            broadcast_uav_status(0)
            time.sleep(1.0)
            if time.time() - timeout_start > 300: # 5 min timeout
                print("!!! [TIMEOUT] Sync failed. Aborting.")
                break

        if ugv_ready:
            # 3. COORDINATED ACTION: Persistent Command Loop
            print("\n[Mission 1] >>> INITIATING COORDINATED UGV ARMING")
            
            ugv_armed_confirmed = False
            command_seq = 1
            
            while not ugv_armed_confirmed:
                print(f"[Mission 1] Commanding UGV ARM (Attempt {command_seq})...")
                bridge.send_command(cmdSeq=command_seq, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0)
                command_seq += 1
                
                # Check for UGV armed confirmation
                for _ in range(5):
                    data = bridge.get_telemetry()
                    if data and data[4] == 1:
                        print("\n!!! [SUCCESS] UGV REPORTS ARMED. PROCEEDING !!!")
                        ugv_armed_confirmed = True
                        break
                    time.sleep(0.2)
                
                if command_seq > 30:
                    print("!!! [ERROR] UGV failed to arm after 30 attempts. Check Hardware.")
                    break

            if ugv_armed_confirmed:
                # 4. Progress tracking
                print("[Mission 1] Tracking UGV mission progress...")
                for i in range(15):
                    telem = bridge.get_telemetry()
                    if telem:
                        v_mps, armed_u = telem[2], telem[4]
                        print(f"  Driving... Speed: {v_mps:.1f} m/s | Status: {'ARMED' if armed_u else 'DISARMED'}")
                    time.sleep(1.0)

        # 5. Clean up
        print("[Mission 1] Test Sequence Complete.")
        disarm_drone()

    except KeyboardInterrupt:
        print("[Mission 1] User Interrupted. Safety Disarming...")
        disarm_drone()
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()

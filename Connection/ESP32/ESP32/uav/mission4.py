from pymavlink import mavutil
import time
import v2v_bridge

# UAV MISSION 4 SCRIPT 🚁
# This is the "Brain" script. It tells the ground vehicle
# to make a full circle twice.

################################# config stuff
# where the fly controller and radio are plugged in
CONNECTION_STRING = "/dev/ttyACM0"   # UAV Flight Controller
BAUD_RATE = 115200
ESP32_PORT = "/dev/ttyUSB0"          # The radio bridge

def main():
    print("==========================================")
    print("   UAV MISSION 4 - CIRCLE MANEUVERS")
    print("==========================================")
    
    # tie into the drones flight controller
    print(f"[Mission 4] Connecting to UAV Controller at {CONNECTION_STRING}...")
    try:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
        master.wait_heartbeat()
        print("[Mission 4] Drone Heartbeat found. Sensors Check: OK")
    except Exception as e:
        print(f"!!! Error connecting to Drone: {e} !!!")
        master = None

    # start the bridge link (v2v_bridge.py)
    print(f"[Mission 4] Starting V2V Bridge on {ESP32_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        bridge.send_message("MISSION 4 STARTING: DOUBLE CIRCLE")
    except Exception as e:
        print(f"!!! Error connecting to ESP32: {e} !!!")
        return

    def broadcast_uav_status():
        # shoves drone arm status into the radio telemetry link
        armed_val = 0
        if master:
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                armed_val = 1 if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
        t_ms = int(time.time() * 1000) & 0xFFFFFFFF
        bridge.send_telemetry(0, t_ms, 0.0, 0.0, armed_val, 0)

    ############################ Mission Logic

    try:
        # 1. wait for the wheel robot to stop being a ghost and sync up
        print("[Mission 4] Waiting for UGV Sync...")
        ugv_ready = False
        while not ugv_ready:
            data = bridge.get_telemetry()
            if data:
                seq_u, t_ms_u, v_u, _, armed_u, safety_byte = data
                is_armable = bool(safety_byte & 0x10)
                print(f"    [RADIO] UGV Sync: ARMED={armed_u}, Armable={is_armable}")
                ugv_ready = True
            broadcast_uav_status()
            time.sleep(1.0)

        # 2. THE CIRCLE MISSION
        print("\n[Mission 4] >>> INITIATING DOUBLE CIRCLE (2x360)")
        
        # Shout the CIRCLE command at the bridge
        bridge.send_command(cmdSeq=400, cmd=v2v_bridge.CMD_CIRCLE, estop=0)
        
        start_mission = time.time()
        # duration for 2 laps
        total_duration = 18.0
        
        while (time.time() - start_mission) < total_duration:
            # grab speed status coming back from the radio while it spins
            data = bridge.get_telemetry()
            if data:
                v_mps = data[2]
                elapsed = time.time() - start_mission
                print(f"  Driving Circle... T+{elapsed:.1f}s | Speed: {v_mps:.2f} m/s", end='\r')
            time.sleep(0.5)

        print("\n[Mission 4] CIRCLE SEQUENCE COMPLETE.")

    except KeyboardInterrupt:
        print("[Mission 4] Interrupted.")
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()

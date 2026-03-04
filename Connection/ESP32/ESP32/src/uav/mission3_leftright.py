from pymavlink import mavutil
import time
import v2v_bridge

# UAV MISSION 3 SCRIPT 🚁
# This is the "Brain" script. It tells the ground vehicle
# to do a 5-stage zig-zag maneuver.

################################# config stuff
# where the flight controller and radio are plugged in
CONNECTION_STRING = "/dev/ttyACM0"   # UAV Flight Controller
BAUD_RATE = 115200                   
ESP32_PORT = "/dev/ttyUSB0"          # The radio bridge

def main():
    print("==========================================")
    print("   UAV MISSION 3 - ZIG-ZAG MANEUVERS")
    print("==========================================")
    
    # tie into the drones flight controller
    print(f"[Mission 3] Tying into Drone at {CONNECTION_STRING}...")
    try:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
        master.wait_heartbeat()
        print("[Mission 3] Drone Heartbeat found. Sensors Check: OK")
    except Exception as e:
        print(f"!!! Error connecting to Drone: {e} !!!")
        master = None

    # start the radio bridge (v2v_bridge.py)
    print(f"[Mission 3] Starting Radio Bridge on {ESP32_PORT}...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge")
    try:
        bridge.connect()
        bridge.send_message("MISSION 3 STARTING...")
    except Exception as e:
        print(f"!!! Error connecting to ESP32: {e} !!!")
        return

    def broadcast_uav_status():
        # tells the ground vehicle if the drone is armed or not
        armed_val = 0
        if master:
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                armed_val = 1 if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
        t_ms = int(time.time() * 1000) & 0xFFFFFFFF
        bridge.send_telemetry(0, t_ms, 0.0, 0.0, armed_val, 0)

    ############################ Mission Logic

    try:
        # 1. wait for the ground vehicle to yell status back
        print("[Mission 3] Waiting for UGV Sync...")
        ugv_ready = False
        while not ugv_ready:
            data = bridge.get_telemetry()
            if data:
                seq_u, t_ms_u, v_u, _, armed_u, safety_byte = data
                is_armable = bool(safety_byte & 0x10)
                has_gps = bool(safety_byte & 0x20)
                print(f"    [RADIO] UGV Sync: ARMED={armed_u}, Armable={is_armable}, GPS={has_gps}")
                ugv_ready = True
            broadcast_uav_status()
            time.sleep(1.0)

        # 2. THE ZIG-ZAG SEQUENCE
        # this is the list of chores the ground robot has to do
        print("\n[Mission 3] >>> INITIATING ZIG-ZAG MANEUVERS")
        
        sequence = [
            (v2v_bridge.CMD_MOVE_2FT, "DRIVE FORWARD 2FT"),
            (v2v_bridge.CMD_TURN_RIGHT, "TURN RIGHT 90"),
            (v2v_bridge.CMD_MOVE_2FT, "DRIVE FORWARD 2FT"),
            (v2v_bridge.CMD_TURN_LEFT, "TURN LEFT 90"),
            (v2v_bridge.CMD_MOVE_2FT, "DRIVE FORWARD 2FT (FINAL)")
        ]
        
        for i, (cmd, desc) in enumerate(sequence):
            print(f"\n[Mission 3] Task {i+1}/5: {desc}")
            # SHOUT the command at the bridge (Packs it via v2v_bridge.py)
            bridge.send_command(cmdSeq=300+i, cmd=cmd, estop=0)
            
            # sit here and wait for the wheel robot to finish
            wait_time = 4.0 if "DRIVE" in desc else 6.0
            start_segment = time.time()
            while (time.time() - start_segment) < wait_time:
                # catch speed telemetry data from the radio
                data = bridge.get_telemetry()
                if data:
                    v_mps = data[2]
                    print(f"  UGV Moving... Speed: {v_mps:.2f} m/s", end='\r')
                time.sleep(0.5)
            print(f"\n[Mission 3] Task {i+1} Complete.")

        print("\n[Mission 3] FULL ZIG-ZAG SEQUENCE COMPLETE.")

    except KeyboardInterrupt:
        print("[Mission 3] Interrupted.")
    finally:
        bridge.stop()

if __name__ == "__main__":
    main()

from pymavlink import mavutil
import time
import sys

ACK_RESULTS = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
}

# --- Connection Settings ---
CONNECTION_STRING = "COM4" 
BAUD_RATE = 57600
print(f"Connecting to {CONNECTION_STRING} at {BAUD_RATE} baud")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)

master.wait_heartbeat()
print("Heartbeat found")

# --- Helper Functions ---
def change_mode(mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise RuntimeError(f"Unknown mode '{mode}'. Available: {list(mapping.keys())}")
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode: {mode}")
    time.sleep(1)

def arm_drone():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0 
    )
    print("Arming...")
    time.sleep(2)

def disarm_drone():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0 
    )
    print("Disarming...")
    time.sleep(2)

def set_throttle(pwm_value):
    """
    Overrides RC Channel 3 (Throttle) with a specific PWM value.
    1000 = Min Throttle, 1500 = Mid Throttle, 2000 = Max Throttle.
    Sending 0 to other channels leaves them alone.
    """
    print(f"Setting Throttle to {pwm_value} PWM")
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, pwm_value, 0, 0, 0, 0, 0  # Channel 3 is throttle
    )

def read_messages():
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            print(f"Status: {msg.text}")
        elif mtype == "COMMAND_ACK":
            if (msg.result != 0 and msg.result != 5):
                return msg.result

def wait_for_msg():
    end = time.time() + 3
    while time.time() < end:
        msgID = read_messages()
        time.sleep(0.05)
        if msgID is not None:
            return msgID

# --- Main Test Execution ---
if __name__ == "__main__":
    try:
        print("\n*** Starting Simulated Takeoff Desk Test ***")
        
        # 1. Change to STABILIZE 
        change_mode("STABILIZE")
        msgID = wait_for_msg()
        if msgID is not None:
            print(f"Mode Change Warning: {ACK_RESULTS.get(msgID, msgID)}")

        # 2. Arm the drone
        arm_drone()
        msgID = wait_for_msg()
        if msgID is not None:
            print(f"Arming Error: {ACK_RESULTS.get(msgID, msgID)}")
            sys.exit(1)

        # 3. Simulate Takeoff by ramping up the throttle
        print("\n--- INITIATING THROTTLE RAMP ---")
        
        # Low idle (just above resting)
        set_throttle(1200) 
        time.sleep(2)

        # "Taking off" (Motors will spin fast!)
        print("Simulating climb...")
        set_throttle(1650) 
        time.sleep(4)

        # "Descending" (Motors slow down)
        print("Simulating descent...")
        set_throttle(1200)
        time.sleep(3)

        # "Landed" (Motors at bottom idle)
        set_throttle(1000)
        time.sleep(1)
        
        print("--- THROTTLE RAMP COMPLETE ---\n")

        # 4. Release RC Override (0 tells ArduPilot to stop overriding)
        set_throttle(0)

        # 5. Disarm securely
        disarm_drone()
        print("Test complete. Motors shut down.")

    except KeyboardInterrupt:
        print("\n[!] Script manually stopped! Cutting throttle and disarming safely.")
        set_throttle(1000)
        time.sleep(0.5)
        set_throttle(0)
        disarm_drone()
    except Exception as e:
        print(f"\n[!] An error occurred: {e}")
        set_throttle(0)
        disarm_drone()

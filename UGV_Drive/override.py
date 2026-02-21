from pymavlink import mavutil
import time

# --- CONFIGURATION ---
connection_string = 'COM3' 
baud_rate = 57600

print(f"Connecting to {connection_string} at {baud_rate} baud...")
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master.wait_heartbeat()
print(f"Heartbeat found (sys={master.target_system}, comp={master.target_component})")

def change_mode(mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"Unknown mode '{mode}'.")
        return
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to: {mode}")
    time.sleep(1.0)

def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Arming sent...")
    time.sleep(2)

def disarm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Disarmed.")

if __name__ == "__main__":
    try:
        # 1. Switch to MANUAL to bypass GPS requirements
        change_mode("MANUAL")
        
        # 2. Arm the vehicle
        arm()
        print("\n*** PRESS THE SAFETY SWITCH NOW SO IT IS SOLID RED ***\n")
        time.sleep(3) # Give you time to press the button
        
        print(">>> FORCING THROTTLE FORWARD FOR 5 SECONDS <<<")
        
        # 3. Send raw RC overrides (Must be looped, or it times out!)
        # Channel 3 is Throttle in ArduRover. 1500 = Neutral, 1600 = Forward
        for _ in range(25):  # Loop 50 times at 0.1s intervals = 5 seconds
            # The 8 numbers represent Channels 1 through 8. 
            # 65535 tells the Cube to ignore that channel.
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component,
                65535, 65535, 1600, 65535, 65535, 65535, 65535, 65535
            )
            time.sleep(0.1)
            
        print("Stopping...")
        
        # 4. Return Throttle to Neutral (1500)
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            65535, 65535, 1500, 65535, 65535, 65535, 65535, 65535
        )
        time.sleep(0.5)
        
        # 5. Clear all overrides (0 = release control)
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
    finally:
        disarm()
        print("Test Complete.")
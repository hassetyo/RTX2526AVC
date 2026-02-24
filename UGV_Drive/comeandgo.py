from pymavlink import mavutil
import time

# --- CONFIGURATION ---
connection_string = '127.0.0.1:14551' #'COM3'
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

def send_rc(steering, throttle, duration):
    steps = int(duration / 0.1)
    for _ in range(steps):
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            steering,      # CH1 - Steering
            65535,
            throttle,      # CH3 - Throttle
            65535,
            65535, 65535, 65535, 65535
        )
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        change_mode("MANUAL")
        arm()

        print("\n*** PRESS THE SAFETY SWITCH NOW ***\n")
        time.sleep(3)

        print(">>> MOVING FORWARD <<<")
        send_rc(1500, 1600, 5)   # Forward 5 seconds

        print(">>> TURNING RIGHT 180 DEG <<<")
        send_rc(1700, 1500, 3)   # Pivot right ~3 seconds (adjust as needed)

        print(">>> MOVING FORWARD AGAIN <<<")
        send_rc(1500, 1600, 5)   # Forward again

        print("Stopping...")

        # Return to neutral
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 65535, 1500, 65535,
            65535, 65535, 65535, 65535
        )

        time.sleep(0.5)

        # Clear overrides
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    finally:
        disarm()
        print("Test Complete.")

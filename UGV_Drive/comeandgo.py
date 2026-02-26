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
def send_velocity(vx, yaw_rate, duration):
    """
    vx: forward velocity (m/s)
    yaw_rate: yaw rate (rad/s)
    duration: seconds
    """


    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )


    steps = int(duration / 0.1)


    for _ in range(steps):
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,      # position (ignored)
            vx, 0, 0,     # velocity (forward only)
            0, 0, 0,      # acceleration (ignored)
            0,            # yaw (ignored)
            yaw_rate      # yaw rate (rad/s)
        )
        time.sleep(0.1)


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
        change_mode("GUIDED")
        arm()


        print(">>> MOVING FORWARD <<<")
        send_velocity(vx=1.0, yaw_rate=0.0, duration=5)


        print(">>> TURNING RIGHT ~180 <<<")
        send_velocity(vx=0.8, yaw_rate=-0.6, duration=7)


        print(">>> MOVING FORWARD AGAIN <<<")
        send_velocity(vx=1.0, yaw_rate=0.0, duration=5)


        print("Stopping...")
        send_velocity(0, 0, 1)


    finally:
        disarm()
        print("Test Complete.")

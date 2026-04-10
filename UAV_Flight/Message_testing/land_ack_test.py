from pymavlink import mavutil

CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE = 115200

master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

def arm_drone(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm
        0, 0, 0, 0, 0, 0
    )
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")
    print("Sent arm command")

def disarm_drone(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        0,  # disarm
        0, 0, 0, 0, 0, 0
    )
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")
    print("Sent disarm command")

def main():
    arm_drone(master)
    time.sleep(15)  # wait for 5 seconds while armed
    disarm_drone(master)
   

if __name__ == "__main__":
    main()
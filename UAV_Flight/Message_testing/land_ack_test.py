from pymavlink import mavutil
import time


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

def change_mode(master, mode_name):
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"Mode {mode_name} not found")
        return
    master.set_mode(mode_id)
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")
    print(f"Sent change mode command to {mode_name}")

def takeoff_drone(master, altitude):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0,  # pitch
        0,  
        0.5,  # takeoff speed
        0,  # yaw
        0,  # y
        0,  # x
        altitude  # z (altitude)
    )
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK:  {ack_msg}")
    print(f"Sent takeoff command to {altitude}m")

def land_drone(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,  # confirmation
        0,  # target
        3,  # offset/acceptable error in meters
        0.5,  # landing speed
        0,  # yaw
        0,  # y
        0,  # x
        0   # z (altitude)
    )
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Land ACK:  {ack_msg}")
    print("Sent land command")

def main():
    change_mode(master, "STABILIZE")
    arm_drone(master)
    takeoff_drone(master, 4)  
    time.sleep(15)  
    change_mode(master, "LAND")
    land_drone(master)
    disarm_drone(master)
   

if __name__ == "__main__":
    main()
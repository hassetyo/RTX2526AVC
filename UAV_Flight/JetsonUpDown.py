from pymavlink import mavutil
import time
import math

#Variables used for consistant movements and motion
ALTITUDE_M = 10
SPEED_MS = 1.0
POS_TOL_M = 0.25
MOVE_TIMEOUT_S = 90
YAW_TOL_DEG = 2.0
TURN_TIMEOUT_S = 20
YAW_STABLE_TIME_S = 0.5

ACK_RESULTS = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
}

# this is a way to go from simulation testing to irl testing

#False if irl testing
#True if simulation testing
USE_SITL = False

if USE_SITL:
    CONNECTION_STRING = "udp:127.0.0.1:14551"   
    print(f"Connecting to SITL on {CONNECTION_STRING}")
    master = mavutil.mavlink_connection(CONNECTION_STRING)
else:
    # --- MINIMAL CHANGES FOR JETSON NANO ---
    # If using USB cable, use "/dev/ttyACM0"
    # If using GPIO Pins (8/10), use "/dev/ttyTHS1"
    CONNECTION_STRING = "/dev/ttyACM0" 
    BAUD_RATE = 115200
    print(f"Connecting to {CONNECTION_STRING} at {BAUD_RATE} baud")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)

master.wait_heartbeat()
print("Heartbeat found")


#Requests all data to be sent at a specific rate
def request_message_streams():
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  
            1
        )
    except Exception:
        pass

#sets the rate in which messages/data are sent to the system
def set_interval(msg_id, hz):
    us = int(1e6 / hz)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        us,
        0, 0, 0, 0, 0
    )

#tells the drone to send its altitude and local position ever 20 Hertz 
try:
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)
except Exception:
    pass

request_message_streams()

#changes the mode of the drone
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

#Arms the drone to be ready for takeoff
def arm_drone():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming...")
    time.sleep(2)

#Lifts the drone off the ground by 'alt' meters
def takeoff(alt: float):
    print(f"Taking off to {alt}m")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        alt
    )
    time.sleep(10)

#This function will request any error messages that may be caused
def read_messages():
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        
        mtype = msg.get_type()

        if mtype == "STATUSTEXT":
            print("Status: ", msg.text)

        elif mtype == "COMMAND_ACK":
            #print("Ack Error: ", msg.command, msg.result)
            if (msg.result != 0 and msg.result != 5):
                return msg

#return the error message, if there is one
def wait_for_msg():
    end = time.time() + 5
    while time.time() < end:
        msgID = read_messages()
        time.sleep(0.05)
        if msgID is not None:
            return msgID



#Main function: moves north, turns wround, moves south, then lands
if __name__ == "__main__":
    
    #Makes sure teh drone is ready for movement
    change_mode("GUIDED")
    msgID = wait_for_msg()
    if msgID is not None:
        #this is the error
        print("***** Error encountered *****")
        change_mode("LAND")
        raise RuntimeError(f"ACK Error: Command {msgID.command} {ACK_RESULTS.get(msgID.result, msgID.result)}")

    arm_drone()
    msgID = wait_for_msg()
    if msgID is not None:
        print("***** Error Encountered *****")
        change_mode("LAND")
        raise RuntimeError(f"ACK Error: Command {msgID.command} {ACK_RESULTS.get(msgID.result, msgID.result)}")

    takeoff(ALTITUDE_M)
    msgID = wait_for_msg()
    if msgID is not None:
        print("***** Error encountered *****")
        change_mode("LAND")
        raise RuntimeError(f"ACK Error: Command {msgID.command} {ACK_RESULTS.get(msgID.result, msgID.result)}")


    print(f"We up in this bitch!")
    time.sleep(2)
    
    print("Landing.")
    change_mode("LAND")
    print("Mission Complete")

from pymavlink import mavutil
import time
import math


ALTITUDE_M = 10
LEG_DISTANCE_M = 5
STEP_DISTANCE_M = 3
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

#returns the local (x,y) coords of the drone
def get_local_xy(timeout=0.5):
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if not msg:
        return None
    return float(msg.x), float(msg.y)

#returns the angle of the drone in degrees
def get_yaw_rad(timeout=0.5):
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)
    if not msg:
        return None
    return float(msg.yaw)

#changes x to an equivilant value that is in the range [0 - 2pi]
def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

#changes the velocity of the drone
def send_velocity_local_ned(vx, vy, vz=0.0):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )

#tells the drone to stop moving
def send_stop(duration_s=1.0):
    end = time.time() + duration_s
    while time.time() < end:
        send_velocity_local_ned(0.0, 0.0, 0.0)
        time.sleep(0.1)

#moves the drone to a local (x,y) position
def move_to_local_xy(xt, yt, speed_ms=SPEED_MS, pos_tol=POS_TOL_M, timeout_s=MOVE_TIMEOUT_S):
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        pos = get_local_xy(timeout=0.2)
        if pos is None:
            continue
        x, y = pos
        ex, ey = (xt - x), (yt - y)
        dist = math.hypot(ex, ey)
        if dist <= pos_tol:
            break
        vx = (ex / dist) * speed_ms
        vy = (ey / dist) * speed_ms
        send_velocity_local_ned(vx, vy, 0.0)
        time.sleep(0.05)
    send_stop(1.0)

#moves the drone +-x meters north/south and/or +-y meters east/west
def move_by_offset_local(dx, dy, speed_ms=SPEED_MS, pos_tol=POS_TOL_M, timeout_s=MOVE_TIMEOUT_S):
    start = get_local_xy(timeout=1.0)
    if start is None:
        raise RuntimeError("Lost local position.")
    x0, y0 = start
    xt, yt = x0 + dx, y0 + dy
    print(f"Moving offset: {dx}, {dy} -> Target: {xt:.2f}, {yt:.2f}")
    move_to_local_xy(xt, yt, speed_ms=speed_ms, pos_tol=pos_tol, timeout_s=timeout_s)

#turns the drone x degrees
def turn_relative_stable(deg, tol_deg=YAW_TOL_DEG, stable_time_s=YAW_STABLE_TIME_S, timeout_s=TURN_TIMEOUT_S):
    yaw0 = get_yaw_rad(timeout=1.0)
    if yaw0 is None:
        raise RuntimeError("Lost Attitude data")
    target = wrap_pi(yaw0 + math.radians(deg))
    tol = math.radians(tol_deg)
    print(f"Turning {deg} degrees")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        abs(deg),
        0,
        1 if deg > 0 else -1,
        1,
        0, 0, 0
    )
    t_end = time.time() + timeout_s
    stable_start = None
    while time.time() < t_end:
        yaw = get_yaw_rad(timeout=0.5)
        if yaw is None:
            continue
        err = abs(wrap_pi(target - yaw))
        if err <= tol:
            if stable_start is None:
                stable_start = time.time()
            elif (time.time() - stable_start) >= stable_time_s:
                return
        else:
            stable_start = None
        time.sleep(0.05)

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
                return msg.result

#return the error message, if there is one
def wait_for_msg():
    end = time.time() + 3
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
        raise RuntimeError("ACK Error: Command ", ACK_RESULTS.get(msgID, msgID))

    arm_drone()
    msgID = wait_for_msg()
    if msgID is not None:
        raise RuntimeError("ACK Error: Command ", ACK_RESULTS.get(msgID, msgID))

    takeoff(ALTITUDE_M)
    msgID = wait_for_msg()
    if msgID is not None:
        raise RuntimeError("ACK Error: Command ", ACK_RESULTS.get(msgID, msgID))


    home = get_local_xy(timeout=2.0)
    if home is None:
        raise RuntimeError("No position data.")
    home_x, home_y = home
    print(f"Home set at: {home_x:.2f}, {home_y:.2f}")

    #Start of the North and South movement
    print("*** Starting Movement ***")

    print("Moving North 5 meters")
    move_by_offset_local(+5,0)

    currentLocation = get_local_xy(timeout=2.0)
    currentLocation_x, currentLocation_y = currentLocation
    print(f"Drone is currently located at: {currentLocation_x:.2f}, {currentLocation_y:.2f}")

    print("Turning Around...")
    turn_relative_stable(180)

    print("Moving South 5 meters")
    move_by_offset_local(+5,0)

    currentLocation = get_local_xy(timeout=2.0)
    currentLocation_x, currentLocation_y = currentLocation
    print(f"Drone is currently located at: {currentLocation_x:.2f}, {currentLocation_y:.2f}")


    print("Returning Home")
    move_to_local_xy(home_x, home_y)
    print("Landing.")
    change_mode("LAND")
    print("Mission Complete")
from pymavlink import mavutil # i had to grab mavlink to actually talk to the drone flight controller brain
import time # i imported time for timing and loop sleeps so the cpu doesnt completely melt
import math # i added math for doing boring degree to radian math



# mission params i tuned
DIST_M = 3.048       # the 10 ft move target cause imperial units are annoying
SPEED_MPS = 0.5      # slow and steady wins the race so it doesnt crash
TAKEOFF_ALT_M = 2.0  # the target height i want it to hover at
MAX_ALT_ALLOWED = 3.5 # safety fence height around 11 ft so it doesnt hit the ceiling

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
    # the uav radio box port i set
    ESP32_PORT = "/dev/ttyUSB0"
    print(f"[Mission 2] Connecting to {CONNECTION_STRING}...") # logging that i am trying to connect
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE) # actually opening the mavlink link i setup


# time to start the drone brain connection
master.wait_heartbeat() # waiting forever for the drone to say hello
print("[Mission 2] Heartbeat found. Optical Flow/Lidar Ready.") # checking if it was a success


############################ the mavlink helpers i wrote

def change_mode(mode: str): # i made this to change the drone flight mode
    mapping = master.mode_mapping() # asking the drone for the valid mode list
    if mode not in mapping: return # abort everything if the mode is totally fake
    mode_id = mapping[mode] # i had to find the secret id for the mode
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id) # blasting the mode pulse to the brain
    print(f"Mode set: {mode}") # logging the change so i can see it

def arm_drone(): # i wrote this to engage the scary drone motors
    master.mav.command_long_send(master.target_system, master.target_component, # sending the actual mavlink pulse
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, # shoving the arm flag in here
                                 0, 0, 0, 0, 0, 0) # a bunch of empty parameters cause the code needs them
    print("Arming...") # logging that it is about to get dangerous

def takeoff(alt): # this tells the drone to actually lift off the floor
    print(f"Takeoff command: {alt}m") # logging the exact height
    master.mav.command_long_send(master.target_system, master.target_component, # sending another mavlink pulse
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt) # the takeoff pulse with the altitude i passed in

def get_altitude(): # i did this to check the current height from the floor
    msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1.0) # grabbing the altitude packet from the stream
    if msg: # if we actually caught a real message
        return msg.alt # return the height in meters
    return 0.0 # just return zero if it completely fails

def send_velocity(vx, vy, vz=0.0): # i use this to move the drone by setting the speed
    master.mav.set_position_target_local_ned_send( # sending the mavlink velocity pulse
        0, master.target_system, master.target_component, # making sure it targets the drone
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111, # setting the velocity mask bitmask thing
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0) # plugging in the speed vectors i want

#################### the main mission 2 logic i built

def main(): # the main boss function that runs it all
    try: # trying to fly without crashing into the wall
        # step 1 is the takeoff sequence
        change_mode("GUIDED") # telling the drone to ignore the sticks and listen to my code
        time.sleep(1) # wait a second for the mode to actually shift
        arm_drone() # spin up the propellers
        time.sleep(1) # wait for the motors to settle down
        takeoff(TAKEOFF_ALT_M) # lift off the floor finally

        # i make it wait until the target height is reached
        print("Climbing...") # log the takeoff progress
        while True: # loop forever until it is high enough
            alt = get_altitude() # check exactly how high we are right now
            if alt >= TAKEOFF_ALT_M * 0.9: # if we reached 90 percent of the target
                print(f"Altitude reached: {alt:.2f}m") # log that it was a huge success
                break # break out of the climbing loop
            time.sleep(0.5) # sleep so we only check every half second

        # step 3 the actual flight maneuver
        print(f"[Mission] Flying {DIST_M}m forward...") # log the flight chore
        start_t = time.time() # i start the clock here
        telem_seq = 0 # setting up the status counter
        abort_triggered = False # i reset the safety flag just in case

        duration = DIST_M / SPEED_MPS # doing math to calculate the total fly time

        # step 4 landing where the autopilot handles sensing the ground
        print("[Mission] Finalizing: Landing now...") # log that we are finishing up
        change_mode("LAND") # forcing the mode to land for a slow descent
        
        # wait around until we finally hit the floor
        while True: # endless loop until it is safe
            alt = get_altitude() # check the altitude again
            if alt < 0.3: # if we are barely 30cm off the floor
                print("Landed safely.") # declare success cause we landed
                break # break out of the landing loop
            time.sleep(1) # check the height every single second
            
    except Exception as e: # if literally anything goes wrong
        print(f"Mission Error: {e}. Attempting EMERGENCY LAND.") # log the massive error panic
        change_mode("LAND") # try to come down safe by forcing land

if __name__ == "__main__": # the standard python entry point
    main() # actually run the main boss function

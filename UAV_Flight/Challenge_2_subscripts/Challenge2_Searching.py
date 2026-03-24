'''
Part 2: Searching
- UGV is still not moving at this moment
- UAV will being the search pattern to find the goal aruco marker

    Seaching Algorithm: Snake pattern
    - The drone will move in a snake-like pattern across the whole field
    - As the drone moves, it will constantly update its current position in the form of (x,y)
        - When the drone finds the goal, it will tell the UGV its coords and will fly back to the UGV
    Steps:
    1. Drone moves forward ~15 yards
    2. Drone turns 90 degrees left/right (depending on the starting location)
    3. Drone moves forward 5 feet
    4. Drone turns 90 degrees left/right (do the same direction as step 2, this should make a U shape)
    5. Drome moves forward ~15 yards
    6. Drone turns 90 degrees right/left (do the opposite direction as steps 2 & 4)
    7. Drone moves forward 5 feet
    8. Drone turn 90 degrees right/left (choose the same direction as step 6)
    9. Repeat until goal ArUco marker is found
'''

from math import log

from pymavlink import mavutil # using the confirmed mavlink pattern instead of dronekit
import time # for timing and sleeps
import sys # for clean exits
#(Uncomment)
#import v2v_bridge # our custom radio bridge talker

# Part 1: Initiation
# UGV is stationary at this moment
# UAV arms and takes off to +4 ft in the air


################################# config stuff i setup
# connection settings from your working test script
CONNECTION_STRING = "/dev/ttyACM0"   # drone wire (use COM4 if testing on windows)
BAUD_RATE = 57600                    # using the confirmed 57600 speed
ESP32_PORT = "/dev/ttyUSB0"          # the radio bridge usb wire

# mission params
TARGET_ALT = 1.3    # hover height in meters (4.2 ft)
CIRCLE_TIME = 18.0  # duration for the rover maneuvers

# throttle settings i tuned
THROTTLE_MIN = 1000   # motors off
THROTTLE_IDLE = 1150  # props spinning but no lift
THROTTLE_CLIMB = 1650 # power to lift off the floor
THROTTLE_HOVER = 1500 # rough middle ground for holding height

# --- Forward flight phase ---
# Tune FORWARD_PITCH_PWM + FORWARD_FLIGHT_TIME so distance ≈ 5 m.
# At ~1580 PWM expect 0.6-0.8 m/s  →  7 s ≈ 5 m
FORWARD_PITCH_PWM   = 1580
FORWARD_FLIGHT_TIME = 7.0       # seconds

# --- RC neutral ---
RC_CENTER = 1500


# --- Flight ---
TARGET_ALT     = 1.3            # hover height in metres
THROTTLE_CLIMB = 1650           # PWM to climb
THROTTLE_HOVER = 1500           # PWM neutral hover
ALT_BAND       = 0.1            # ±m band before altitude correction fires
ALT_BOOST      = 100            # PWM delta applied when outside ALT_BAND

# --- Log file ---
LOG_FILE = "Challenge2_Searching.log"

############################ the mavlink helpers

def change_mode(master, mode: str): # changes the flight controller mode
    mapping = master.mode_mapping() # ask for the list of modes
    if mode not in mapping: # if the mode is fake
        print(f"Unknown mode '{mode}'") # log the error
        return # bail out
    mode_id = mapping[mode] # find the secret mode id
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id) # blast it
    print(f"Mode set: {mode}") # log the change
    time.sleep(1) # wait for the mode to settle

def arm_drone(master): # engages the scary drone motors
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0 
    )
    print("Arming motors...") # log the arming
    time.sleep(2) # wait for the spinning to start

def disarm_drone(master): # stops the motors securely
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0 
    )
    print("Disarmed.") # log the safety

def set_rc_override(master, roll=RC_CENTER, pitch=RC_CENTER,
                    throttle=THROTTLE_HOVER, yaw=RC_CENTER):
    """Send RC channel overrides CH1-4. All channels always sent explicitly."""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        roll, pitch, throttle, yaw,
        0, 0, 0, 0,
    )

def set_throttle(master, pwm): # physically pushes the throttle via rc override
    # channel 3 is the throttle in ardupilot
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )

def throttle_hold(alt: float) -> int:
    """Simple bang-bang altitude hold around TARGET_ALT."""
    if alt < TARGET_ALT - ALT_BAND:
        return THROTTLE_HOVER + ALT_BOOST   # sinking → more power
    if alt > TARGET_ALT + ALT_BAND:
        return THROTTLE_HOVER - ALT_BOOST   # too high → less power
    return THROTTLE_HOVER

def get_lidar_alt(master): # checks the floor distance via lidar
    #(Uncomment)
    #msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0) # wait for lidar packet
    #if msg: # if we got a real message
    #    return msg.current_distance / 100.0 # return height in meters
    return 1.3 # return zero if lidar is dead

#Logging events in a text doc
def log_event(text): # helper to write required logs
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}\n"
    print(line.strip())
    with open(LOG_FILE, "a") as f: f.write(line)

def yaw_right_90(master):
    print("Turning 90 degrees right...")
    # This is a 'timed' turn since we aren't reading compass/IMU yet
    # Adjust TURN_TIME based on your drone's sensitivity
    TURN_TIME = 1.5 
    start = time.time()
    while (time.time() - start) < TURN_TIME:
        # CH4 is Yaw. 1500 is neutral, 1600 is right rotation
        set_rc_override(master, yaw=1600, throttle=THROTTLE_HOVER)
        time.sleep(0.1)
    set_rc_override(master, yaw=RC_CENTER, throttle=THROTTLE_HOVER)

#moves the drone forward for a set duration
def move_forward_timed(master, seconds, pitch_pwm=1580, debug_log=True):
    #- pitch_pwm: 1580 is a gentle forward tilt.

    print(f"\n>>> COMMAND: Moving FORWARD for {seconds}s")
    start_time = time.time()
    
    while (time.time() - start_time) < seconds:
    
        current_alt = get_lidar_alt(master) #Get current altitude
        
        
        thr = throttle_hold(current_alt) #calculate throttle based on altitude error
        
        # 3. Send RC Override to move forward while maintaining altitude
        set_rc_override(master, pitch=pitch_pwm, throttle=thr, roll=RC_CENTER, yaw=RC_CENTER)
        
        if debug_log:
            elapsed = time.time() - start_time
            print(f" [FLYING] Time: {elapsed:.1f}/{seconds}s | Pitch: {pitch_pwm} | Alt: {current_alt}m", end='\r')
            
        time.sleep(0.1) # 10Hz update rate is standard for MAVLink overrides

    # 4. BRAKE / NEUTRALIZE
    # After the time is up, we must center the pitch or the drone will keep drifting
    print(f"\n>>> Forward move complete. Neutralizing pitch.")
    set_rc_override(master, pitch=RC_CENTER, throttle=THROTTLE_HOVER)

#Searching algorithm: snake pattern
#Temporarily goes forward to test if thats even possible
def snake_search_pattern(master, passes=3):
    fwd_start = time.time()
    print("Moving forward for 20 seconds to test forward flight pattern...")
    move_forward_timed(master, seconds=20, pitch_pwm=FORWARD_PITCH_PWM)
    print("Forward flight test complete.")

#################### the main Challenge 2 logic

def main(): # the main boss function
    print("=========================================") # header
    print("        UAV CHALLENGE 2: Searching") # title
    print("=========================================") # footer
    
    # step 1: connect to the wires
    print(f"Connecting to Drone: {CONNECTION_STRING}...") # login
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE) # open link
    master.wait_heartbeat() # wait for buzz
    print("Drone Heartbeat OK.") # success

    ''' # we dont need to connect to the ugv  for now (Uncomment)
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # radio bridge
    try: # try to open radio
        bridge.connect() # open serial wires
        bridge.send_message("MISSION 4: MAVLINK MODE START") # yell over air
    except: # if radio is missing
        log_event("Radio Bridge Fail.") # log fail
        return # bail
    '''
    try: # wrapping mission in safety block
        # step 2: takeoff sequence (using your working pattern)
        change_mode(master, "STABILIZE") # switch to stabilize for manual throttle control
        arm_drone(master) # start the props
        
        print("Climbing to 1.3m...") # log the climb
        while True: # loop until target height
            #(Uncomment)
            #alt = get_lidar_alt(master) # check lidar
            alt = 0.0 # placeholder until we test the lidar in this script
            print(f" Altitude: {alt:.2f}m", end='\r') # log height
            #temporarily break the loop to test the forward flight pattern without lidar input
            break
            if alt >= TARGET_ALT: # if we hit the hover point
                set_throttle(master, THROTTLE_HOVER) # pull back to hover power
                print(f"\nHover altitude reached: {alt:.2f}m") # declare success
                break # break the climb
            set_throttle(master, THROTTLE_CLIMB) # keep pushing up
            time.sleep(0.1) # quick loop

        # Step 3: Snake search pattern
        snake_search_pattern(master)

        #Step 4: land and shutdown
        print("\nLanding sequence engaged...") # start descent
        change_mode(master, "LAND") # switch to official land mode for graceful touchdown
        
        set_throttle(master, 0) # release throttle override so autopilot takes over

        while True: # loop until we hit the floor
            #(Uncomment)
            #alt = get_lidar_alt(master) # check lidar
            alt = 0.0 # placeholder until we test the lidar in this script
            print(f" Land Alt: {alt:.2f}m", end='\r') # log altitude
            
            # checking if the drone disarmed itself (autopilot does this after landing)
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("\nTouchdown confirmed. Motors stopped.") # log success
                break # exit
            time.sleep(0.5) # slower loop for checking
    

    except KeyboardInterrupt: # someone hit ctrl+c
        print("\n[!] Emergency: User Triggered Landing...") # abort log
        change_mode(master, "LAND") # force land mode immediately
        set_throttle(master, 0) # release override
        time.sleep(1) # wait for command to hit
    finally: # final chores
        #(Uncomment)
        #bridge.stop() # close radio wire 
        print("Mission finalized.") # end log

if __name__ == "__main__": # entry point
    main() # run it
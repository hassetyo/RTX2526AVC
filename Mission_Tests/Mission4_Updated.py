from pymavlink import mavutil # using the confirmed mavlink pattern instead of dronekit
import time # for timing and sleeps
import sys # for clean exits
import v2v_bridge # our custom radio bridge talker

# uav mission 4 - autonomous flight (throttle override) + ugv circle
# this uses the pattern the user confirmed works (stabilize + rc override)

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

############################ the mavlink helpers i wrote

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

def set_throttle(master, pwm): # physically pushes the throttle via rc override
    # channel 3 is the throttle in ardupilot
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )

def get_lidar_alt(master): # checks the floor distance via lidar
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0) # wait for lidar packet
    if msg: # if we got a real message
        return msg.current_distance / 100.0 # return height in meters
    return 0.0 # return zero if lidar is dead

#Logging events in a text doc
def log_event(text): # helper to write required logs
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}\n"
    print(line.strip())
    with open(LOG_FILE, "a") as f: f.write(line)

#################### the main mission 4 logic

def main(): # the main boss function
    log_event("==========================================") # header
    log_event("   UAV MISSION 4 - CONFIRMED MAVLINK PATTERN") # title
    log_event("==========================================") # footer
    
    # step 1: connect to the wires
    log_event(f"Connecting to Drone: {CONNECTION_STRING}...") # login
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE) # open link
    master.wait_heartbeat() # wait for buzz
    log_event("Drone Heartbeat OK.") # success

    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # radio bridge
    try: # try to open radio
        bridge.connect() # open serial wires
        bridge.send_message("MISSION 4: MAVLINK MODE START") # yell over air
    except: # if radio is missing
        log_event("Radio Bridge Fail.") # log fail
        return # bail

    try: # wrapping mission in safety block
        # step 2: takeoff sequence (using your working pattern)
        change_mode(master, "STABILIZE") # switch to stabilize for manual throttle control
        arm_drone(master) # start the props
        
        log_event("Climbing to 1.3m...") # log the climb
        while True: # loop until target height
            alt = get_lidar_alt(master) # check lidar
            log_event(f" Altitude: {alt:.2f}m", end='\r') # log height
            if alt >= TARGET_ALT: # if we hit the hover point
                set_throttle(master, THROTTLE_HOVER) # pull back to hover power
                log_event(f"\nHover altitude reached: {alt:.2f}m") # declare success
                break # break the climb
            set_throttle(master, THROTTLE_CLIMB) # keep pushing up
            time.sleep(0.1) # quick loop

        # step 3: sync with ground rover
        log_event("Waiting for UGV sync...") # logging wait
        while True: # loop until radio sync
            set_throttle(master, THROTTLE_HOVER) # MUST keep sending hover pulse or it crashes
            data = bridge.get_telemetry() # pull from mailbox
            if data: # if we got a packet
                log_event("UGV Ready. Initiating Circles.") # log coordination
                break # done
            time.sleep(0.2) # check 5 times a second to keep rc heartbeat alive

        # step 4: command the rover work
        bridge.send_command(cmdSeq=400, cmd=v2v_bridge.CMD_CIRCLE, estop=0) # blast command
        
        start_t = time.time() # start clock
        while (time.time() - start_t) < CIRCLE_TIME: # loop for duration
            data = bridge.get_telemetry() # check status
            if data: # if real status
                log_event(f" UGV Speed: {data[2]:.2f} m/s", end='\r') # log rover stats
            
            # small "crude" altitude hold logic i added
            alt = get_lidar_alt(master) # check lidar
            if alt < TARGET_ALT - 0.1: # if we are sinking
                set_throttle(master, THROTTLE_HOVER + 100) # give it more juice
            elif alt > TARGET_ALT + 0.1: # if we are drifting too high
                set_throttle(master, THROTTLE_HOVER - 100) # cut power
            else: # if we are golden
                set_throttle(master, THROTTLE_HOVER) # keep steady
            
            time.sleep(0.1) # 10hz loop

        # step 5: land and shutdown
        log_event("\nLanding sequence engaged...") # start descent
        change_mode(master, "LAND") # switch to official land mode for graceful touchdown
        set_throttle(master, 0) # release throttle override so autopilot takes over

        while True: # loop until we hit the floor
            alt = get_lidar_alt(master) # check lidar
            log_event(f" Land Alt: {alt:.2f}m", end='\r') # log altitude
            
            # checking if the drone disarmed itself (autopilot does this after landing)
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                log_event("\nTouchdown confirmed. Motors stopped.") # log success
                break # exit
            time.sleep(0.5) # slower loop for checking

    except KeyboardInterrupt: # someone hit ctrl+c
        log_event("\n[!] Emergency: User Triggered Landing...") # abort log
        change_mode(master, "LAND") # force land mode immediately
        set_throttle(master, 0) # release override
        time.sleep(1) # wait for command to hit
    finally: # final chores
        bridge.stop() # close radio wire
        log_event("Mission finalized.") # end log

if __name__ == "__main__": # entry point
    main() # run it

from dronekit import connect, VehicleMode # library to bully the pixhawk
import time # for clocking things
import math # for doing circles and turns
import v2v_bridge # our custom translator for the radio boxes
from pymavlink import mavutil # raw mavlink pulses for the speed commands

# UGV GROUND STATION SCRIPT
# This runs on the Raspberry Pi and bullies the Pixhawk into moving
# based on what the Jetson/Drone says over the radio

############## config stuff
# where the hardware is plugged in
UGV_CONTROL_PORT = "/dev/ttyACM0"   # The Pixhawk/Cube connection
ESP32_BRIDGE_PORT = "/dev/ttyUSB0"  # The ESP32 talking to the drone

# drive settings
DIST_M = 3.048       # 10ft default distance
SPEED_MPS = 1.5      # m/s target speed
TELEM_SEND_HZ = 5    # status update rate

# start the connection to the wheel ugv
print("==========================================") # header
print("   UGV GROUND STATION - MISSION READY") # title
print("==========================================") # footer
print(f"[Ground] Connecting to UGV at {UGV_CONTROL_PORT}...") # log start

try: # try to connect without dying
    vehicle = connect(UGV_CONTROL_PORT, wait_ready=True, baud=115200) # open link
    print(f"[Ground] Connected! Ready to sync.") # success
except Exception as e: # if it fail
    print(f"!!! Error connecting to UGV: {e} !!!") # log error
    exit() # give up

######################### avoidance brain

class ObstacleDetector: # interface for the LUXONIS OAK-D-Lite
    def __init__(self): # setup logic
        # would init the depthai pipeline here
        self.safe_distance_m = 1.0 # 5ft radius requirement (approx 1.5m, using 1m for tight)
        pass # placeholder

    def check_for_buckets(self): # scanning loop
        # sweeps the depth cam for boxes or buckets
        # returns (found, angle_to_obj, distance)
        return False, 0, 0 # placeholder for actual cam feed logic

def log_avoidance(text): # official competition mission 3 logs
    timestamp = time.strftime("%H:%M:%S") # get clock
    with open("ugv_mission3_avoidance.txt", "a") as f: # open file
        f.write(f"[{timestamp}] [AVOIDANCE] {text}\n") # write line
    print(f"[{timestamp}] [AVOIDANCE] {text}") # show console

#########################status Logic

def broadcast_status(bridge, seq): # yells status back to drone
    # grabs status from the pixhawk and yells it back to the drone
    armed_val = 1 if vehicle.armed else 0 # check motor state
    m = vehicle.mode.name # get current mode string
    mode_idx = v2v_bridge.MODE_INITIAL # default
    
    # mapping mode names to numbers
    if m == "GUIDED": mode_idx = v2v_bridge.MODE_GUIDED # guided bits
    elif m == "AUTO": mode_idx = v2v_bridge.MODE_AUTO # auto bits
    elif m == "LAND": mode_idx = v2v_bridge.MODE_LAND # land bits
    
    # pack GPS and safety flags into a single byte
    armable_bit = 0x10 if vehicle.is_armable else 0x00 # can we arm
    gps_bit = 0x20 if (vehicle.gps_0.fix_type > 0) else 0x00 # do we have gps
    safety_byte = (mode_idx & 0x0F) | armable_bit | gps_bit # merge bits
    
    # current time and ground speed
    t_ms = int(time.time() * 1000) & 0xFFFFFFFF # time in ms
    v_mps = vehicle.groundspeed if vehicle.groundspeed is not None else 0.0 # groundspeed
    
    # shove status down the USB wire to the bridge
    bridge.send_telemetry(seq, t_ms, v_mps, 0.0, armed_val, safety_byte) # bridge pulse

def arm_and_sync(bridge): # forces motor engagement
    # forces the UGV to arm even if its being a pissrat about GPS
    print("\n[Ground] >>> INITIATING HYBRID ARM SEQUENCE") # log start
    if not vehicle.is_armable: # if safety checks fail
        print(f"!!! [WARNING] PRE-ARM CHECKS FAILED (GPS: {vehicle.gps_0.fix_type}) !!!") # log fail
    
    # aggressive arm-disarm-arm sequence to wake up the Pixhawk
    for attempt in ["FIRST ARM", "RESET DISARM", "FINAL ARM"]: # 3 steps
        state = True if "ARM" in attempt else False # figure out target
        print(f"  [FORCE-SYNC] Initiating {attempt} in {vehicle.mode.name} mode...") # log attempt
        for retry in range(3): # try 3 times
            vehicle.armed = state # tell pixhawk
            timeout = time.time() + 3 # set timeout
            while vehicle.armed != state: # wait for it
                if time.time() > timeout: break # stop waiting
                broadcast_status(bridge, 0) # keep drone updated
                time.sleep(0.1) # wait
            if vehicle.armed == state: break # stop retries if successful
        time.sleep(1.0) # let settle

    if not vehicle.armed: return # stop if failed

    # switch to GUIDED mode so it takes radio commands
    print(f"  [MODE] Switching {vehicle.mode.name} -> GUIDED...") # log mode shift
    vehicle.mode = VehicleMode("GUIDED") # tell pixhawk
    m_timeout = time.time() + 5 # set timeout
    while vehicle.mode.name != "GUIDED" and time.time() < m_timeout: # wait for shift
        broadcast_status(bridge, 0) # keep drone updated
        time.sleep(0.1) # wait
    
    print("!!! UGV FULLY ARMED AND SYNCED !!!\n") # success

#######execution Engine (The "Slave")

def execute_drive(bridge, distance_m): # moves ugv for a set distance
    # sends a pulsed MAVLink message to drive forward
    print(f"[Ground] DRIVE: {distance_m}m at {SPEED_MPS}m/s") # log drive
    msg = vehicle.message_factory.set_position_target_local_ned_encode( # build pulse
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111, # local mode
        0, 0, 0, SPEED_MPS, 0, 0, 0, 0, 0, 0, 0) # speed vectors
    
    start_t = time.time() # start clock
    duration = distance_m / SPEED_MPS # calc time
    while (time.time() - start_t) < duration: # loop for duration
        vehicle.send_mavlink(msg) # blast mavlink pulse
        broadcast_status(bridge, 0) # keep drone happy
        time.sleep(0.1) # 10hz
    
    # full stop
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode( # build stop
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111, # local mode
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) # zero speed
    vehicle.send_mavlink(stop_msg) # blast stop
    time.sleep(0.5) # let settle

def execute_turn(bridge, angle_deg): # spins in place
    # spins the ugv in place to a specific relative angle
    print(f"[Ground] >>> EXECUTING TURN {angle_deg} DEGREES") # log turn
    target_yaw = angle_deg # target
    yaw_rate = 60 # deg/s speed of pivot
    direction = 1 if angle_deg > 0 else -1 # 1 = right, -1 = left
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode( # build pivot pulse
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000101111111111, # yaw rate mode
        0, 0, 0, 0, 0, 0, 0, 0, 0, math.radians(yaw_rate * direction), 0) # yaw speed
    
    duration = abs(angle_deg) / yaw_rate # calc time
    start_t = time.time() # start clock
    while (time.time() - start_t) < duration: # loop
        vehicle.send_mavlink(msg) # blast mavlink
        broadcast_status(bridge, 0) # update drone
        time.sleep(0.1) # 10hz
    
    # send a stop message at the end
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode( # build stop
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111111111, # zero everything
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) # zeros
    vehicle.send_mavlink(stop_msg) # blast stop
    time.sleep(1.0) # let it settle

def execute_drive_forever(bridge, speed_mps): # constant drive
    # drives straight at a constant speed until CMD_STOP is received
    print(f"[Ground] >>> MISSION 1: DRIVING STRAIGHT AT {speed_mps} m/s") # log mission
    msg = vehicle.message_factory.set_position_target_local_ned_encode( # build msg
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111, # local mode
        0, 0, 0, speed_mps, 0, 0, 0, 0, 0, 0, 0) # speed
    
    # this will keep looping in the main script now
    return msg # return msg to main loop

def execute_circle(bridge, speed, yaw_rate_deg, circles=1): # arc move
    # blends forward juice and turning juice to make an arc
    print(f"[Ground] CIRCLE: Speed {speed}m/s | Yaw Rate {yaw_rate_deg}deg/s | count {circles}") # log circle
    duration = (360.0 / abs(yaw_rate_deg)) * circles # calc time
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode( # build circle msg
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0x05C7, # velocity mask
        0, 0, 0, speed, 0, 0, 0, 0, 0, 0, # forward speed
        math.radians(yaw_rate_deg)) # yaw rate
    
    start_t = time.time() # start clock
    while (time.time() - start_t) < duration: # loop
        vehicle.send_mavlink(msg) # blast mavlink
        broadcast_status(bridge, 0) # update drone
        time.sleep(0.1) # 10hz
        
    # hard stop
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode( # build stop
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0x05C7, # zeros
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) # zeros
    vehicle.send_mavlink(stop_msg) # blast stop
    time.sleep(0.5) # let settle

####

def execute_gps_denied_goto(bridge, x, y, avoidance_mode=False): # navigate to relative coords
    # tells the ugv to move to a relative position (NED)
    # x = forward/back, y = right/left
    print(f"[Ground] >>> GOTO RELATIVE: X={x}m, Y={y}m (Avoidance: {avoidance_mode})") # log goto
    
    if not avoidance_mode: # if direct path
        # standard direct path
        msg = vehicle.message_factory.set_position_target_local_ned_encode( # build target msg
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000, # position mask
            x, y, 0, 0, 0, 0, 0, 0, 0, 0, 0) # target coords
        vehicle.send_mavlink(msg) # blast mavlink
    else: # if dodging buckets
        # simplistic avoidance loop (the 5ft rule)
        # we would use OAK-D-Lite depth here to steer away
        detector = ObstacleDetector() # setup cam
        found, angle, dist = detector.check_for_buckets() # search field
        
        if found: # if something is there
            log_avoidance(f"Obstacle Detected at {dist}m, Angle {angle}!") # log detection
            log_avoidance("Path Decision: Re-routing right (90 deg pivot)") # log path
            execute_turn(bridge, 90) # steer away
            execute_drive(bridge, 1.5) # move 5ft away
            log_avoidance("Avoidance Maneuver Complete. Returning to Target Path.") # log success
        
        # continue to final coords
        msg = vehicle.message_factory.set_position_target_local_ned_encode( # build target coords
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000, # position mask
            x, y, 0, 0, 0, 0, 0, 0, 0, 0, 0) # target
        vehicle.send_mavlink(msg) # blast mavlink

def main(): # the main executor loop
    # start the bridge link to the drone
    bridge = v2v_bridge.V2VBridge(ESP32_BRIDGE_PORT, name="UGV-Bridge") # bridge object
    try: # try to open serial
        bridge.connect() # open serial wires
        bridge.send_message("ground station is live . awaiting drone orders") # yell startup
    except: return # fail if no wire

    seq = 0 # status sequence
    mission_active = False # mission flag
    mission_3_avoidance = False # dodge flag
    drive_msg = None # current move command

    try: # main loop
        while True: # loop forever
            # 1. broadcast status so the drone doesn't worry
            broadcast_status(bridge, seq) # update drone
            seq += 1 # increment

            # [DEBUG] check for raw string talk (Coordinate Parsing)
            msg_str = bridge.get_message() # check string mailbox
            if msg_str: # if there is a shout
                print(f"[Ground] Incoming Shout: {msg_str}") # log it
                if msg_str.startswith("GOTO:"): # if it's coordinates
                    try: # try to parse
                        _, coords = msg_str.split(":") # split shout
                        x_val, y_val = map(float, coords.split(",")) # parse numbers
                        if not vehicle.armed: arm_and_sync(bridge) # arm if needed
                        execute_gps_denied_goto(bridge, x_val, y_val, avoidance_mode=mission_3_avoidance) # move
                    except: print("!!! [ERROR] Failed to parse coordinates !!!") # fail log

            # 2. check for orders from the drone
            cmd = bridge.get_command() # check command mailbox
            if cmd: # if drone wants a chore
                cmdSeq, cmdVal, eStop = cmd # unpack command
                print(f"[Ground] Got Choice Order: {cmdVal}") # log it

                if eStop == 1: # if we hit the big red button
                    print("!!! [ABORT] EMERGENCY DISARM !!!") # log panic
                    mission_active = False # kill mission
                    drive_msg = None # kill move
                    vehicle.armed = False # kill motors
                    continue # skip to next loop

                if not vehicle.armed: # if motors off
                    arm_and_sync(bridge) # wake it up

                if cmdVal == v2v_bridge.CMD_MOVE_FORWARD: # forward chore
                    execute_drive(bridge, 3.048) # 10ft
                elif cmdVal == v2v_bridge.CMD_MOVE_2FT: # 2ft chore
                    execute_drive(bridge, 0.61) # 2ft
                elif cmdVal == v2v_bridge.CMD_TURN_RIGHT: # right chore
                    execute_turn(bridge, 90) # pivot 90
                elif cmdVal == v2v_bridge.CMD_TURN_LEFT: # left chore
                    execute_turn(bridge, -90) # pivot -90
                elif cmdVal == v2v_bridge.CMD_CIRCLE: # circle chore
                    execute_circle(bridge, 1.0, 45, circles=2) # spin 2 laps
                elif cmdVal == v2v_bridge.CMD_MISSION_1 or cmdVal == v2v_bridge.CMD_MISSION_2 or cmdVal == v2v_bridge.CMD_MISSION_3: # mission start
                    mission_active = True # set live
                    drive_msg = execute_drive_forever(bridge, 0.15) # constant slow drive
                    if cmdVal == v2v_bridge.CMD_MISSION_3: # if obstacle dodging
                        mission_3_avoidance = True # flip dodge bit
                        log_avoidance("Mission 3 Active: Obstacle Avoidance Engaged.") # log avoidance
                elif cmdVal == v2v_bridge.CMD_STOP: # stop chore
                    mission_active = False # kill flag
                    mission_3_avoidance = False # kill dodge
                    drive_msg = None # kill move
                    if vehicle.armed: # if live
                        log_avoidance("Destination Arrival / Manual Stop.") # log arrival
                    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode( # build stop msg
                        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111111111, # zeros
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) # zeros
                    vehicle.send_mavlink(stop_msg) # blast stop

            if mission_active and drive_msg: # if mission is driving
                vehicle.send_mavlink(drive_msg) # keep blasting forward pulse

            time.sleep(1.0 / TELEM_SEND_HZ) # wait for next cycle

    except KeyboardInterrupt: # someone hit ctrl+c
        pass # just let it close
    finally: # clean up always
        bridge.stop() # shut down radio link
        vehicle.close() # shut down pixhawk link

if __name__ == "__main__": # entry point
    main() # run the boss

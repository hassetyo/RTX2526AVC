######### mission 1 - autonomous launch & moving landing
# MISSION 1: OPERATION TOUCHDOWN 
# objective: takeoff from ugv, hold height, track moving ugv, and land on it
# constraint: no gps allowed . vision and optical flow only
# this uses the confirmed working pattern (stabilize + rc override)

import time # library for timing and logs
import cv2 # opencv for aruco detection
import numpy as np # math junk for vision
from pymavlink import mavutil # confirmed working communication library
import v2v_bridge # our radio translator

################################# competition config i set up
ESP32_PORT = "/dev/ttyUSB0" # radio bridge wire
DRONE_PORT = "/dev/ttyACM0" # drone controller wire (use COM4 if on windows)
BAUD_RATE = 57600           # confirmed working speed
ARUCO_ID = 0                # target marker on rover deck
TARGET_ALT = 1.3            # hover height in meters
RIDE_TIME_REQ = 30          # seconds to ride after landing
MISSION_TIMEOUT = 420       # 7 min deadline

# throttle settings i tuned
THROTTLE_MIN = 1000   # motors off
THROTTLE_IDLE = 1150  # props spinning
THROTTLE_CLIMB = 1650 # power to lift off
THROTTLE_HOVER = 1500 # hold height power

# official logging file for the refs
LOG_FILE = "mission1_official_log.txt"

def log_event(text): # helper to write required logs
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}\n"
    print(line.strip())
    with open(LOG_FILE, "a") as f: f.write(line)

# helper to clear overrides and let the autopilot take control (important for safe landings)
def clear_overrides(master):
    """Releases all manual RC overrides so the autopilot takes full control."""
    master.mav.rc_channels_override_send(
        master.target_system, 
        master.target_component, 
        0, 0, 0, 0, 0, 0, 0, 0  # All zeros = 'Release to Autopilot'
    )
    log_event("RC Overrides cleared. Autopilot in control.")

#################### vision brain for the zed x camera

'''
class Tracker: # helper to find the rover deck
    def __init__(self): # setup detector
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dict, self.params)

    def find_ugv(self, frame): # find marker center pixel
        if frame is None: return None
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None and ARUCO_ID in ids:
            idx = list(ids.flatten()).index(ARUCO_ID)
            c = corners[idx][0]
            cx = np.mean(c[:, 0])
            cy = np.mean(c[:, 1])
            return (cx, cy)
        return None
'''

############################ drone control helpers

def change_mode(master, mode: str): # changes the flight controller mode
    mapping = master.mode_mapping() # ask for the list of modes
    if mode not in mapping: # if the mode is fake
        print(f"Unknown mode '{mode}'") # log the error
        return # bail out
    mode_id = mapping[mode] # find the secret mode id
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id) # blast it
    print(f"Mode set: {mode}") # log the change
    time.sleep(1) # wait for the mode to settle

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


def send_velocity_pulse(master, vx, vy, vz): # move relative to drone body
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

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

# EMERGENCY: Forces the motors to stop immediately. The drone will drop like a stone.
def hard_disarm(master):
    """
    EMERGENCY ONLY: Forces the motors to stop immediately.
    The drone will drop like a stone.
    """
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
        0, # Confirmation
        0, # 0 to Disarm
        21196, # Magic number to force disarm even if not on the ground (ArduPilot specific)
        0, 0, 0, 0, 0
    )
    log_event("!!! HARD DISARM SENT: MOTORS STOPPED !!!")


def main(): # main mission boss engine
    log_event("--- MISSION 1 STARTING: WORKING MAVLINK PATTERN ---")
    
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Mission-1")
    try: bridge.connect()
    except: return

    try:
        log_event("Connecting to Drone Cube Orange...")
        master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        log_event("Drone Heartbeat OK.")

        log_event("Waiting for UGV radio link...")
        ugv_synced = False
        while not ugv_synced: # sync loop
            set_throttle(master, 0) # keep control released while waiting
            if bridge.get_telemetry():
                log_event("UGV Found. Communications verified.")
                ugv_synced = True
            time.sleep(1.0)

        # autonomous launch (manual throttle ramp)
        log_event("UAV START TIME: Initiating Launch...")
        change_mode(master, "STABILIZE")
        
        # arming motors
        arm_drone(master) # start the props
        log_event("Climbing to 1.3m...") # log the climb
        while True: # loop until target height
            alt = get_lidar_alt(master) # check lidar
            print(f" Altitude: {alt:.2f}m", end='\r') # log height
            if alt >= (TARGET_ALT * 0.9): # if we hit the hover point
                set_throttle(master, THROTTLE_HOVER) # pull back to hover power
                print(f"\nHover altitude reached: {alt:.2f}m") # declare success
                break # break the climb
            set_throttle(master, THROTTLE_CLIMB) # keep pushing up
            time.sleep(0.1) # quick loop
        
        set_throttle(master, THROTTLE_HOVER) # switch to hover power
        log_event(f"UAV reached {TARGET_ALT}m altitude.")

        log_event("UGV START TIME: Commanding UGV to move...")
        bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MISSION_1, estop=0)
        
        '''
        tracker = Tracker()
        # cam = cv2.VideoCapture(0) # zed x camera
        
        log_event("Tracking UGV. Alignment Sequence Running...")
        is_home = False
        
        while not is_home: # tracking loop
            set_throttle(master, THROTTLE_HOVER) # HEARTBEAT
            
            if (time.time() - launch_t) > MISSION_TIMEOUT:
                log_event("!!! FAILURE: TIMEOUT EXCEEDED (7 MIN) !!!")
                break
                
            target = tracker.find_ugv(None) # find rover in camera
            if target: # if detected
                send_velocity_pulse(master, 0.2, 0.0, 0.1) # tracking drift
            else: # if lost
                send_velocity_pulse(master, 0.0, 0.0, 0.0) # station hold
                
            # check for touchdown (disarmed means landed)
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                log_event("UAV LANDING TIME: Platform landing confirmed.")
                is_home = True
                break
                
            time.sleep(0.1) # 10hz loop heartbeat

        # if we timed out or need to force a safe landing
        if not is_home:
            log_event("Initiating Safe Landing Sequence...")
            change_mode(master, "LAND")
            set_throttle(master, 0) # release to autopilot
            while True:
                msg = master.recv_match(type='HEARTBEAT', blocking=False)
                if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    log_event("Touchdown and disarm confirmed.")
                    is_home = True
                    break
                time.sleep(0.5)

        if is_home: # final ride duration
            log_event("Initiating 30-Second Ride Duration...")
            set_throttle(master, 0) # release control once landed
            ride_start = time.time()
            while (time.time() - ride_start) < RIDE_TIME_REQ:
                time.sleep(1.0)
            
            log_event("RIDE DURATION TIME COMPLETE: 30s achieved.")
            log_event("MISSION 1 SUCCESSFUL")
            bridge.send_command(cmdSeq=2, cmd=v2v_bridge.CMD_STOP, estop=0)
        '''

        # step 3: sync with ground rover
        print("Waiting for UGV sync...") # logging wait
        while True: # loop until radio sync
            set_throttle(master, THROTTLE_HOVER) # MUST keep sending hover pulse or it crashes
            data = bridge.get_telemetry() # pull from mailbox
            if data: # if we got a packet
                print("UGV Ready. Initiating Circles.") # log coordination
                break # done
            time.sleep(0.2) # check 5 times a second to keep rc heartbeat alive
        
        # step 4: command the rover work
        bridge.send_command(cmdSeq=400, cmd=v2v_bridge.CMD_MOVE_FORWARD, estop=0) # blast command

        start_t = time.time() # start clock
        while (time.time() - start_t) < 5: # loop for duration
            data = bridge.get_telemetry() # check status
            if data: # if real status
                print(f" UGV Speed: {data[2]:.2f} m/s", end='\r') # log rover stats
            
            # small "crude" altitude hold logic i added
            alt = get_lidar_alt(master) # check lidar
            if alt < TARGET_ALT - 0.1: # if we are sinking
                set_throttle(master, THROTTLE_HOVER + 100) # give it more juice
            elif alt > TARGET_ALT + 0.1: # if we are drifting too high
                set_throttle(master, THROTTLE_HOVER - 100) # cut power
            else: # if we are golden
                set_throttle(master, THROTTLE_HOVER) # keep steady
            
            time.sleep(0.1) # 10hz loop
        
        # Step 5: Land and shutdown
        log_event("Landing sequence engaged...") 
        change_mode(master, "LAND") 
        clear_overrides(master) # Let autopilot take the wheel

        landing_start_time = time.time()
        LANDING_TIMEOUT = 15 # Give it 15 seconds to find the ground
        
        while True:
            alt = get_lidar_alt(master)
            elapsed = time.time() - landing_start_time
            print(f" Land Alt: {alt:.2f}m | Timer: {elapsed:.1f}s", end='\r')
            
            # Check for automatic disarm (Autopilot logic)
            msg = master.recv_match(type='HEARTBEAT', blocking=False)
            is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if msg else True
            
            if msg and not is_armed:
                log_event("\nTouchdown confirmed. Motors stopped naturally.")
                break
            
            # EMERGENCY: If landing takes too long, force it.
            if elapsed > LANDING_TIMEOUT:
                log_event("\n[!] Landing Timeout: Forcing Disarm for safety.")
                hard_disarm(master) # Force the motors off
                break
                
            time.sleep(0.5)

    except KeyboardInterrupt:
        log_event("[!] Emergency: User Triggered Landing Sequence.")
        change_mode(master, "LAND")
        clear_overrides(master) # This replaces set_throttle(master, 0)

    finally:
        # Final cleanup: stop the communication bridge
        log_event("Closing connection...")
        bridge.stop()

if __name__ == "__main__":
    main()
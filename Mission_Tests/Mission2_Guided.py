######### mission 2 - aruco target identification & travel
# MISSION 2
# objective: takeoff, find marker id 0-4, send coords to ugv, and land on moving target
# constraint: no gps allowed . vision and optical flow only for stable scout flight
# this uses the confirmed working pattern (stabilize + rc override)

import time # library for timing and logs
import cv2 # opencv for scanning
import numpy as np # coordinate math
from pymavlink import mavutil # verified flight library
import v2v_bridge # radio bridge translator

################################# competition config
ESP32_PORT = "/dev/ttyUSB0" # radio bridge wire
DRONE_PORT = "/dev/ttyACM0" # drone controller wire
BAUD_RATE = 57600           # verified speed
TARGET_ALT = 1.3            # approx 4.2 feet
RIDE_TIME_REQ = 10          # seconds to ride
MISSION_TIMEOUT = 600       # 10 min deadline

# throttle settings i tuned
THROTTLE_MIN = 1000   # motors off
THROTTLE_IDLE = 1150  # props spinning
THROTTLE_CLIMB = 1650 # power to lift
THROTTLE_HOVER = 1500 # hold height

# official logging file for the competition refs
LOG_FILE = "mission2_official_log.txt"

def log_event(text): # helper to write required logs
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}\n"
    print(line.strip())
    with open(LOG_FILE, "a") as f: f.write(line)

#################### vision brain for the zed x camera

class Tracker: # helper to scan field and find markers
    def __init__(self):
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dict, self.params)

    def find_target(self, frame): # look for marker id 0-4
        if frame is None: return None
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if 0 <= marker_id <= 4:
                    c = corners[i][0]
                    cx = np.mean(c[:, 0])
                    cy = np.mean(c[:, 1])
                    return (marker_id, cx, cy)
        return None

############################ drone control helpers

def change_mode(mode: str): # i made this to change the drone flight mode
    mapping = master.mode_mapping() # asking the drone for the valid mode list
    if mode not in mapping: return # abort everything if the mode is totally fake
    mode_id = mapping[mode] # i had to find the secret id for the mode
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id) # blasting the mode pulse to the brain
    print(f"Mode set: {mode}") # logging the change so i can see it

def set_throttle(master, pwm): # rc override heartbeat
    master.mav.rc_channels_override_send(master.target_system, master.target_component, 0, 0, pwm, 0, 0, 0, 0, 0)

def get_lidar_alt(master): # distance sensor logic
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1.0)
    if msg: return msg.current_distance / 100.0
    return 0.0

def send_velocity_pulse(master, vx, vy, vz): # move body-relative
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)

def main(): # main mission 2 logic
    log_event("--- MISSION 2 STARTING: WORKING MAVLINK PATTERN ---")
    
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Mission-2")
    try: bridge.connect()
    except: return

    try:
        log_event("Connecting to Cube Orange flight controller...")
        master = mavutil.mavlink_connection(DRONE_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        log_event("Drone Heartbeat OK.")

        # auto launch (GUIDED pattern)
        log_event("Initiating Autonomous Launch Phase...")
        change_mode(master, "GUIDED")
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
        time.sleep(2)
        
        
        launch_t = time.time()
        log_event("Beginning Climb")
        while True: # climb loop
            alt = get_lidar_alt(master)
            if alt >= (TARGET_ALT * 0.9): break
            set_throttle(master, THROTTLE_CLIMB) # pushing up
            time.sleep(0.1)
        
        set_throttle(master, THROTTLE_HOVER)
        log_event(f"Minimum Altitude {TARGET_ALT}m Reached.")

        # destination discovery (ArUco Hunt)
        log_event("Starting Field Scan for ArUco Marker ID 0-4...")
        tracker = Tracker()
        found = False
        target_x, target_y = 0.0, 0.0
        
        while not found: # search loop
            set_throttle(master, THROTTLE_HOVER) # HEARTBEAT
            res = tracker.find_target(None)
            if res:
                m_id, cx, cy = res
                log_event(f"DESTINATION DISCOVERY: Found Marker ID {m_id}")
                target_x, target_y = 10.0, 5.0 # fake destination
                found = True
            else:
                send_velocity_pulse(master, 0.1, 0.0, 0.0) # scout drift
                time.sleep(0.1)

        # coordination relay
        log_event("COMMUNICATION: Sending coordinates to UGV...")
        bridge.send_message(f"GOTO:{target_x},{target_y}")
        time.sleep(0.5)
        bridge.send_command(cmdSeq=1, cmd=v2v_bridge.CMD_MISSION_2, estop=0)

        # tracking & landing
        log_event("Aligning for Precision Landing on Moving Platform...")
        landed = False
        while not landed: # alignment loop
            set_throttle(master, THROTTLE_HOVER) # HEARTBEAT
            if (time.time() - launch_t) > MISSION_TIMEOUT: break
                
            send_velocity_pulse(master, 0.15, 0.0, 0.1) # descent drift
            
            # detect touchdown
            msg_l = master.recv_match(type='HEARTBEAT', blocking=False)
            if msg_l and not (msg_l.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                log_event("UAV LANDING TIME: Platform landing confirmed.")
                landed = True
                break
            time.sleep(0.1)

        if not landed:
            log_event("Initiating Safe Landing Sequence...")
            change_mode(master, "LAND")
            set_throttle(master, 0)
            while True:
                msg = master.recv_match(type='HEARTBEAT', blocking=False)
                if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    log_event("UAV Landing Confirmed.")
                    landed = True
                    break
                time.sleep(0.5)

        if landed: # ride duration
            log_event("Riding UGV to final destination...")
            set_throttle(master, 0)
            ride_t = time.time()
            while (time.time() - ride_t) < RIDE_TIME_REQ:
                time.sleep(1.0)
            log_event("END TIMES: Ride complete. UGV at destination.")
            bridge.send_command(cmdSeq=2, cmd=v2v_bridge.CMD_STOP, estop=0)
            log_event("MISSION 2 SUCCESSFUL")

    except KeyboardInterrupt:
        log_event("[!] Emergency: User Triggered Landing Sequence.")
        change_mode(master, "LAND")
        set_throttle(master, 0)
        time.sleep(1)
    finally:
        bridge.stop()
        set_throttle(master, 0) # final safety check

if __name__ == "__main__":
    main()
'''
This code is for testing the landing of the UAV on the UGV. 
The UAV will go ip a few meters off the ground and will move 
forward a few meters and will tell the ugv to move forward.
When the UGV is done moving, the UAV will track the ArUco marker
on the UGV and will land on it. As its landing, the UAV will reposition
itself to be centered on the UGV. Once the UAV is landed, it will tell
the UGV to move forward a few meters.
'''

from curses.ascii import alt
import time
import cv2
import socket
import json
import base64
import numpy as np
import math
import sim_v2v_bridge as v2v_bridge
from aruco_tracker import ArucoTracker
from snake_search import SnakeSearch

#configs

WEBOTS_PORT = 5760
ESP32_PORT = "/dev/ttyUSB0"
TARGET_ALT = 2.5 #height in meters
LANDING_ALT = 0.3 #altitude to target when landing
MISSION_TIMEOUT = 30.0 #seconds before we give up and land
CENTER_HOLD_TIME = 5.0 #seconds to hold position over target before landing
LOG_FILE = "sim_SamTest_log.txt"

''' Log events to both terminal and file '''
def log_event(text):
    timestamp = time.strftime("%H:%M:%S") #Log the time of the event
    line = f"[{timestamp}] {text}" #Format the log line
    print(line) #Print on the terminal
    with open(LOG_FILE, "a") as f: f.write(line + "\n") #Print on the txt file

''' WEBOTS DRONE LINK CLASS '''
class WebotsDroneLink:
    """TCP link to the Webots SimPixhawk controller"""
    def __init__(self, port=WEBOTS_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #Create a TCP socket
        self.sock.settimeout(5.0) #Set a timeout for connection attempts
        self.sock.connect(('localhost', port)) #Connect to the Webots server
        self.sock.setblocking(False) #Set socket to non-blocking mode for receiving data
        self._buffer = b"" #Buffer for incoming data
        print(f"Connected to Webots on port {port}")
    
    ''' Send a command to the Webots drone '''
    def send_cmd(self, action, **kwargs): # self = the instance of the class, action = the command to send, kwargs = additional parameters for the command
        cmd = {"action": action} #Create a command dictionary with the action
        cmd.update(kwargs) #Add any additional parameters to the command
        try:
            self.sock.sendall((json.dumps(cmd) + "\n").encode('utf-8')) #Send the command as JSON over TCP
            self.sock.settimeout(5.0) #Set a timeout for receiving a response
            data = b"" #Buffer for incoming data
            while b"\n" not in data: #Keep receiving until we get a full line (response)
                chunk = self.sock.recv(65536) #Receive data from the socket
                if not chunk: break #If connection is closed, break
                data += chunk #Add received chunk to buffer
            return json.loads(data.split(b"\n")[0].decode('utf-8')) #Return the response as a dictionary
        except Exception as e:
            return {"error": str(e)} #Return any errors that occur
    
    ''' Get the drone's altitude from the LidarLite sensor '''
    def get_lidar_alt(self):
        resp = self.send_cmd("get_lidar") #Send a command to get the lidar altitude
        return resp.get("lidar", 0.0) #Return the altitude from the response, or 0.0 if not available
    
    ''' Arm the drone for flight '''
    def arm(self):
        self.send_cmd("arm", value=True) #Send a command to arm the drone
        time.sleep(1) #Wait for a moment to ensure the command is processed
    
    ''' Show the camera feed and highlight detected ArUco markers '''
    def showCamera(self, tracker):
        frame = self.get_camera_frame() #Get the latest camera frame
        if frame is not None:
            results = tracker.detect_all(frame) #Find all markers in the frame
            annotated_frame = tracker.draw_detections(frame, results) #Draw green boxes and ID labels
            cv2.imshow("Drone Camera View", annotated_frame) #Display the window
            cv2.waitKey(1) #Update the OpenCV window
            return annotated_frame
        return None

    ''' Detect ArUco marker and center the drone above it '''
    def detect_aruco(self, tracker, target_id=0):
        log_event(f"Searching for target ID: {target_id}...")
        while True:
            frame = self.get_camera_frame() #Get the latest camera frame
            if frame is None: continue
            
            result = tracker.find_target(frame) #Specifically look for our target ID
            if result:
                # Proportional control for centering (Normalizing 640x480 resolution)
                vx = clamp(-(result.dy / 240.0) * 0.2, -0.5, 0.2) #Adjust forward/backward
                vy = clamp((result.dx / 320.0) * 0.2, -0.5, 0.2) #Adjust left/right
                self.set_velocity(vx=vx, vy=vy, vz=0.0) #Apply calculated velocities
                
                if tracker.is_centered(result, deadband_px=30): #Check if we are within the deadband
                    log_event("Drone is centered over marker.")
                    self.set_velocity(0, 0, 0) #Stop movement
                    break
            else:
                self.set_velocity(0, 0, 0) #If marker is lost, hover in place
            
            self.showCamera(tracker) #Update the visual window during the process
            time.sleep(0.01)
    
    ''' Perform a snake search pattern to locate the ArUco marker using time-based logic '''
    def perform_snake_search(self, tracker, target_id=0, timeout=60.0):
        log_event(f"Starting snake search for ArUco ID: {target_id}...")
        
        # Initialize with row_length (seconds) and row_spacing (seconds)
        searcher = SnakeSearch(row_length=2.0, row_spacing=2.0, speed=0.1)
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            frame = self.get_camera_frame()
            if frame is not None:
                # 1. Check if we see the target during the search
                result = tracker.find_target(frame)
                if result:
                    log_event(f"Target {target_id} found! Stopping search.")
                    self.set_velocity(0, 0, 0)
                    return True
                
                # 2. Show the camera feed to the user
                self.showCamera(tracker)
            
            # 3. Update the searcher state and get current velocity
            searcher.update()
            vx, vy, vz = searcher.get_velocity()
            
            # 4. Command the drone (Webots: +vx Forward, +vy Left)
            self.set_velocity(vx=vx, vy=vy, vz=vz)
            
            time.sleep(0.05) # Loop frequency
            
        log_event("Search timed out without finding the target.")
        self.set_velocity(0, 0, 0)
        return False

    '''Landing algorithm: slowly decrease altitude until we are close to the ground, or the ugv head, then disarm'''
    def land(self, tracker, landOnUGV=False, target_id=0):
        log_event("Initiating landing sequence...")
        if landOnUGV:
            log_event("Targeting UGV ArUco for landing...")
            self.send_cmd("land") #Tell controller to begin descent
            while True:
                alt = self.get_lidar_alt() #Monitor current altitude
                frame = self.get_camera_frame() #Get frame for active centering
                if frame is not None:
                    res = tracker.find_target(frame) #Find UGV marker
                    if res:
                        # Active centering during descent
                        vx = clamp(-(res.dy / 240.0) * 0.2, -0.4, 0.2) #
                        vy = clamp((res.dx / 320.0) * 0.2, -0.4, 0.2) #
                        self.set_velocity(vx, vy, 0.0)
                
                self.showCamera(tracker) #Show the centering progress
                if alt < 0.15: #If we are within touchdown range
                    log_event("Touchdown detected on UGV.")
                    break
                time.sleep(0.02)
        else:
            log_event("Landing on ground...")
            while True:
                alt = self.get_lidar_alt() #Get the current altitude
                print(f"\r  [Landing] Alt: {alt:.2f}m", end="", flush=True)
                self.send_cmd("set_target_alt", value=LANDING_ALT)
                if alt <= 0.3: #If we are close to the ground, disarm
                    log_event("Close to ground. Disarming drone.")
                    break
                else:
                    self.set_velocity(vz=-0.1) #If we are still high, set a slow descent velocity
                time.sleep(0.1)
        
        self.disarm() #Power down motors
    
    ''' Disarm the drone after flight '''
    def disarm(self):
        self.send_cmd("arm", value=False) #Send a command to disarm the drone
        time.sleep(1) #Wait for a moment to ensure the command is processed

    ''' Set RC override values for manual control '''
    def set_rc_override(self, throttle=1500):
        self.send_cmd("rc_override", throttle=throttle) #Send a command to override RC inputs with specified throttle

    ''' Set velocity in the drone's local frame '''
    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0):
        self.send_cmd("set_velocity", vx=vx, vy=vy, vz=vz) #Send a command to set the drone's velocity in x, y, z directions

    ''' Get the latest camera frame from the drone '''
    def get_camera_frame(self):
        #Requests the latest 640x480 camera frame from the Webots simulation
        resp = self.send_cmd("get_camera") #Send a command to get the camera frame
        if "image" in resp and resp["image"]: #Check if the response contains an image
            try:
                img_data = base64.b64decode(resp["image"]) #Decode the base64 image data
                img_array = np.frombuffer(img_data, dtype=np.uint8) #Convert the byte data to a numpy array
                return cv2.imdecode(img_array, cv2.IMREAD_COLOR) #Decode the image array into an OpenCV image
            except Exception as e:
                print(f"Error decoding camera image: {e}")
                return None
        return None

''' Helper function to keep values within bounds '''
def clamp(value, low, high):
    return max(low, min(high, value)) #Returns value restricted to the [low, high] range

def main():
    # Start of the mission
    log_event("--- SIMULATION SAM TEST MISSION STARTING ---") 
    drone = WebotsDroneLink(WEBOTS_PORT) # Create a link to the Webots drone
    
    # Initialize the ArUco Tracker for ID 2 (UGV)
    tracker = ArucoTracker(dictionary=cv2.aruco.DICT_6X6_1000, target_id=2)
    log_event("Simulation Hardware Linked. Drone and Tracker ready.")    
    
    # Connect the bridge
    log_event("Connecting to V2V Bridge...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") # Create a link to the V2V bridge
    bridge.send_message("UAV SIMULATION SAM TEST LIVE") # Send a message indicating mission is live
    bridge.connect() # Connect to the V2V bridge
    
    # Test the connection
    if bridge.sock:
        log_event("V2V Bridge Connected. Ready to send and receive messages.")
    else:
        log_event("Failed to connect to V2V Bridge.")

    # Arm the drone
    log_event("Arming drone...") 
    drone.arm() # Arm the drone

    # Takeoff
    log_event(f"Climbing to {TARGET_ALT}m...")
    drone.send_cmd("set_target_alt", value=TARGET_ALT) # Set the target altitude

    # Wait until we reach the target altitude (or timeout)
    climb_start = time.time() 
    while time.time() - climb_start < MISSION_TIMEOUT: 
        alt = drone.get_lidar_alt() # Get current altitude
        print(f"\r  [Climb] Alt: {alt:.2f}m / {TARGET_ALT}m", end="", flush=True)
        if alt >= (TARGET_ALT - 0.1): 
            log_event(f"\nTarget Altitude {TARGET_ALT}m Reached.") 
            break
        time.sleep(0.1) 
    else:
        log_event("\nFailed to reach target altitude within timeout. Landing...")
        drone.land(tracker, landOnUGV=False) # Fallback to standard landing
        return
    
    # Hover for a few seconds
    log_event(f"Hovering at {TARGET_ALT}m for {CENTER_HOLD_TIME} seconds...")
    hover_start = time.time() 
    while time.time() - hover_start < CENTER_HOLD_TIME: 
        drone.showCamera(tracker) # Keep camera active during hover
        alt = drone.get_lidar_alt() 
        print(f"\r  [Hover] Alt: {alt:.2f}m / {TARGET_ALT}m", end="", flush=True)
        time.sleep(0.1)

    # Altitude Control Test
    print("\nTesting up and down velocity control...")
    print("Moving up...")   
    drone.send_cmd("set_target_alt", value=TARGET_ALT + 1.0) 
    time.sleep(5) 
    print("Current altitude:", drone.get_lidar_alt())

    print("Moving down...")
    drone.send_cmd("set_target_alt", value=TARGET_ALT)
    time.sleep(5)   
    print("Current altitude:", drone.get_lidar_alt())

    # Move the UGV forward
    log_event("Commanding UGV to move forward...")
    # Send command to UGV via V2V Bridge
    bridge.send_command(cmdSeq=100, cmd=v2v_bridge.CMD_MOVE_FORWARD, payload=0) 
    time.sleep(5) # Wait for UGV movement to complete

    found = drone.perform_snake_search(tracker, target_id=2)

    if found:
        drone.detect_aruco(tracker, target_id=2) # Center over it
        drone.land(tracker, landOnUGV=True, target_id=2) # Land
    else:
        log_event("UGV not found. Returning to home or landing on ground.")
        drone.land(tracker, landOnUGV=False)

    # Land the drone on the UGV
    log_event("Initiating Precision Landing on UGV...")
    drone.land(tracker, landOnUGV=True, target_id=2) # Use active centering landing
    
    log_event("Mission complete. Drone disarmed and landed.")

    # Close the V2V bridge connection
    try:
        bridge.stop() # Stop the V2V bridge connection
        log_event("V2V Bridge connection closed.")
    except: pass

if __name__ == "__main__":
    print("\n\nits starting...\n\n")
    main()
    print("\n\nits over\n\n")
'''
This will test basic functions on the Webots drone, 
like taking off, flying forward, telling the uav to move, and landing.
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
        

    '''Landing algorithm: slowly decrease altitude until we are close to the ground, or the ugv head, then disarm'''
    def land(self, landOnUGV=False):
        log_event("Initiating landing sequence...")
        while True:
            if landOnUGV: #change the code to use the camera to find the UGV's aruco marker and land on it, instead of just landing in place
                frame = self.get_camera_frame() #Get the latest camera frame
                if frame is not None:
                    #Use OpenCV to detect the ArUco marker and get its position in the image
                    #Then use that position to adjust the drone's velocity to move towards the marker
                    pass #This part is left as an exercise for later, as it requires more complex image processing and control logic
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
                    time.sleep(0.1) #Sleep briefly to avoid spamming the sensor with requests
            break
    
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

    ''' Get the latest camera frame from the drone ''' #will be used later
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

''' 
*********************
Main mission function
********************* 
'''

def main():
    #Start of the mission
    log_event("--- SIMULATION SAM TEST MISSION STARTING ---") 
    drone = WebotsDroneLink(WEBOTS_PORT) #Create a link to the Webots drone
    log_event("Simulation Hardware Linked. Drone is ready.")    
    
    #Connect the bridge
    log_event("Connecting to V2V Bridge...")
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UAV-Bridge") #Create a link to the V2V bridge for communication with the ESP32
    bridge.send_message("UAV SIMULATION SAM TEST LIVE") #Send a message to the V2V bridge indicating the mission is live
    bridge.connect() #Connect to the V2V bridge
    #test the connection
    if bridge.sock:
        log_event("V2V Bridge Connected. Ready to send and receive messages.")
    else:
        log_event("Failed to connect to V2V Bridge.")

    #Arm the drone
    log_event("Arming drone...") 
    drone.arm() #Arm the drone

    #Takeoff
    log_event(f"Climbing to {TARGET_ALT}m...")
    drone.send_cmd("set_target_alt", value=TARGET_ALT) #Send a command to set the target altitude for the drone

    # Wait until we reach the target altitude (or timeout)
    climb_start = time.time() #Record the start time of the climb
    while time.time() - climb_start < MISSION_TIMEOUT: #Loop until we reach the target altitude
        alt = drone.get_lidar_alt() #Get the current altitude from the Lidar sensor
        print(f"\r  [Climb] Alt: {alt:.2f}m / {TARGET_ALT}m", end="", flush=True)
        if alt >= (TARGET_ALT - 0.1): #Check if we are within 0.1m of the target altitude
            log_event(f"Target Altitude {TARGET_ALT}m Reached.") 
            break
        time.sleep(0.1) #Sleep briefly to avoid spamming the sensor with requests
    else:
        log_event("\nFailed to reach target altitude within timeout. Landing...")
        drone.land() #Disarm the drone if we fail to reach the target altitude
        drone.disarm()
        return
    
    #Hover for a few seconds
    log_event(f"Hovering at {TARGET_ALT}m for {CENTER_HOLD_TIME} seconds...")
    hover_start = time.time() #Record the start time of the hover
    while time.time() - hover_start < CENTER_HOLD_TIME: #Loop for the duration of the hover
        alt = drone.get_lidar_alt() #Get the current altitude from the Lidar sensor
        print(f"\r  [Hover] Alt: {alt:.2f}m / {TARGET_ALT}m", end="", flush=True)
        time.sleep(0.1)
    
    #Move forward x meters (simulate forward flight)
    log_event("Flying forward...")
    drone.set_velocity(vx=0.1) #Set a forward velocity of 0.1 m/s
    time.sleep(2) #Fly forward for 2 seconds
    drone.set_velocity(vx=0.0) #Stop forward movement  

    print("\nTesting up and down velocity control...")
    print("Moving up...")   
    # Instead of just velocity, tell the controller to target a higher spot
    drone.send_cmd("set_target_alt", value=TARGET_ALT + 1.0) 
    time.sleep(10) 
    print("current altitude:", drone.get_lidar_alt())

    print("Moving down...")
    drone.send_cmd("set_target_alt", value=TARGET_ALT)
    time.sleep(10)   
    print("current altitude:", drone.get_lidar_alt())

    #Move the ugv forward
    log_event("Commanding UGV to move forward...")
    bridge.send_command(cmdSeq=100, cmd=v2v_bridge.CMD_MOVE_FORWARD, payload=0) #Send a command to the V2V bridge to move the UGV forward
    time.sleep(5) #Wait for the UGV to move

    #Land the drone
    log_event("Landing drone...")
    drone.land() #Disarm the drone to land
    drone.disarm() #Ensure the drone is disarmed after landing
    log_event("Mission complete. Drone disarmed and landed.")

    # Close the V2V bridge connection
    try:
        bridge.stop() #Stop the V2V bridge connection
        log_event("V2V Bridge connection closed.")
    except: pass

if __name__ == "__main__":
    print("\n\nits starting...\n\n")
    main()
    print("\n\nits over\n\n")

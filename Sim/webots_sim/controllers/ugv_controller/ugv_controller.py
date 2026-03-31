"""
Simulated Pixhawk for Skid-Steer UGV
====================================
Runs inside Webots as the UGV controller.
Acts exactly like the real Pixhawk on the rover.

Opens a TCP Server on port 5761.
Your `sim_ground_station.py` script will connect to this port,
allowing you to run the ground station from your terminal.
"""

from controller import Robot
import socket
import select
import json
import math

# Skid steer constants
TURN_SPEED = 2.0
DRIVE_SPEED = 3.0

class SimPixhawkUGV:
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())

        self.wheel_fl = robot.getDevice("wheel_fl")
        self.wheel_fr = robot.getDevice("wheel_fr")
        self.wheel_rl = robot.getDevice("wheel_rl")
        self.wheel_rr = robot.getDevice("wheel_rr")

        for wheel in [self.wheel_fl, self.wheel_fr, self.wheel_rl, self.wheel_rr]:
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)

        # TCP Server (Port 5761 for UGV)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', 5761))
        self.server.listen(1)
        self.target_port = 5761
        self.client = None
        self.net_buffer = ""

        print("[UGV-Pixhawk] TCP Server running on 127.0.0.1:5761")
        print("[UGV-Pixhawk] Waiting for ground station script...")

        # State vars
        self.armed = False
        self.current_cmd = "STOP"
        self.cmd_timer = 0.0

    def process_network(self):
        if self.client is None:
            try:
                conn, addr = self.server.accept()
                conn.setblocking(False)
                self.client = conn
                print(f"[UGV-Pixhawk] Ground station connected from {addr}!")
            except BlockingIOError:
                pass
        
        if self.client is not None:
            try:
                ready = select.select([self.client], [], [], 0)
                if ready[0]:
                    chunk = self.client.recv(1024).decode('utf-8')
                    if not chunk:
                        print("[UGV-Pixhawk] Ground station disconnected.")
                        self.client.close()
                        self.client = None
                        self.current_cmd = "STOP"
                        self.armed = False
                        return
                    
                    self.net_buffer += chunk
                    while '\n' in self.net_buffer:
                        line, self.net_buffer = self.net_buffer.split('\n', 1)
                        if not line.strip(): continue
                        try:
                            self.handle_command(json.loads(line))
                        except: pass
            except: pass

    def handle_command(self, cmd):
        action = cmd.get("action")
        if action == "arm":
            self.armed = bool(cmd.get("value"))
            print(f"[UGV-Pixhawk] {'ARMED' if self.armed else 'DISARMED'}")
            
        elif action == "drive":
            if not self.armed: return
            direction = cmd.get("dir", "STOP")
            duration = cmd.get("duration", 0)
            self.current_cmd = direction
            if duration > 0:
                self.cmd_timer = self.robot.getTime() + duration
            else:
                self.cmd_timer = float('inf')
            print(f"[UGV-Pixhawk] Executing: {direction} for {duration}s")
            
        elif action == "stop":
            self.current_cmd = "STOP"
            
        elif action == "get_status":
            self.send_response({"armed": self.armed, "speed": 1.5 if self.current_cmd != "STOP" else 0.0})

    def send_response(self, data):
        if self.client:
            try:
                self.client.sendall((json.dumps(data) + "\n").encode('utf-8'))
            except: pass

    def run_step(self):
        self.process_network()
        t = self.robot.getTime()

        # Timeout timed commands
        if t > self.cmd_timer and self.current_cmd != "STOP":
            self.current_cmd = "STOP"
            print("[UGV-Pixhawk] Maneuver complete.")

        # Execute motor commands
        if self.current_cmd == "STOP" or not self.armed:
            speeds = [0, 0, 0, 0]
        elif self.current_cmd == "FORWARD":
            speeds = [DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED]
        elif self.current_cmd == "FORWARD_SLOW":
            speeds = [6.0, 6.0, 6.0, 6.0]
        elif self.current_cmd == "TURN_LEFT":
            speeds = [-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED]
        elif self.current_cmd == "TURN_RIGHT":
            speeds = [TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED]
        elif self.current_cmd == "CIRCLE":
            speeds = [TURN_SPEED, TURN_SPEED * 0.4, TURN_SPEED, TURN_SPEED * 0.4]
        else:
            speeds = [0, 0, 0, 0]

        self.wheel_fl.setVelocity(speeds[0])
        self.wheel_fr.setVelocity(speeds[1])
        self.wheel_rl.setVelocity(speeds[2])
        self.wheel_rr.setVelocity(speeds[3])

def main():
    robot = Robot()
    pix = SimPixhawkUGV(robot)
    while robot.step(pix.timestep) != -1:
        pix.run_step()

if __name__ == "__main__":
    main()

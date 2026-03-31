"""
UAV Controller for Webots Mavic 2 Pro PROTO
============================================
GPS-DENIED simulation: GPS is used internally for the proven Cyberbotics
altitude PID (simulating a real barometer + LidarLite v3 combo), but GPS
XY coordinates are NOT exposed to mission scripts.

Uses the EXACT same PID constants and motor mixing proven by Cyberbotics
in their official mavic2pro.c sample controller.
"""
from controller import Robot
import socket
import select
import json
import math
import numpy as np
import base64
import cv2

# ============================================================
# PROVEN Cyberbotics PID constants (from mavic2pro.c)
# ============================================================
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P = 1.5
K_ROLL_P = 30.0
K_PITCH_P = 20.0
K_YAW_P = 0.6


def clamp(value, low, high):
    return max(low, min(high, value))


class SimPixhawk:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # ---- Mavic 2 Pro Motors ----
        self.front_left_motor = self.robot.getDevice("front left propeller")
        self.front_right_motor = self.robot.getDevice("front right propeller")
        self.rear_left_motor = self.robot.getDevice("rear left propeller")
        self.rear_right_motor = self.robot.getDevice("rear right propeller")
        self.motors = [self.front_left_motor, self.front_right_motor,
                       self.rear_left_motor, self.rear_right_motor]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        # ---- Sensors ----
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)  # Used internally as barometer stand-in
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)

        # ---- Display (HUD Mirror) ----
        self.display = self.robot.getDevice("display")
        if self.display:
            self.display.attachCamera(self.camera) # Overlay on HUD

        # ---- Gimbal camera (built-in) ----
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        
        # Get Lidar from bodySlot
        self.lidar = self.robot.getDevice("lidar_alt")
        self.lidar.enable(self.timestep)

        # ---- Gimbal motors: point camera straight down ----
        try:
            cam_roll = self.robot.getDevice("camera roll")
            cam_roll.setPosition(0.0)
            cam_pitch = self.robot.getDevice("camera pitch")
            cam_pitch.setPosition(1.57)  # Pos 1.57 to look DOWN (previously -1.57 was sky)
            cam_yaw = self.robot.getDevice("camera yaw")
            cam_yaw.setPosition(0.0)
        except Exception:
            pass

        # ---- TCP Server ----
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', 5760))
        self.server.listen(1)
        self.server.setblocking(False)
        self.client = None

        print("[SimPixhawk] Mavic 2 Pro Controller ONLINE (GPS-DENIED)")
        print("[SimPixhawk] TCP Server running on 127.0.0.1:5760")
        print("[SimPixhawk] Waiting for mission script...")

        # Flight parameters
        self.armed = False
        self.target_altitude = 0.1
        self.desired_altitude = 3.0
        self.net_buffer = ""
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0
        self.target_yaw = 0.0
        
        self.brake_counter = 0 # Active counter-thrust timer
        
        # Filtered disturbances to prevent shaking
        self.filt_pitch_dist = 0.0
        self.filt_roll_dist = 0.0
        self.alpha = 0.50 # Smoothing factor (0.1 to 1.0)
        self.rc_throttle = 1500
        self.rc_roll = 1500
        self.rc_pitch = 1500
        self.rc_yaw = 1500

    def get_altitude_internal(self):
        """Internal altitude for PID using GPS Z (simulates barometer)."""
        return self.gps.getValues()[2]

    def get_altitude_lidar(self):
        """Lidar altitude for mission scripts (GPS-denied)."""
        alt = self.lidar.getValue() / 1000.0
        
        if alt is None or math.isnan(alt) or math.isinf(alt) or alt < 0.001:
            return self.get_altitude_internal() # Fallback to GPS Z
            
        rpy = self.imu.getRollPitchYaw()
        tilt = math.cos(rpy[0]) * math.cos(rpy[1])
        return alt * max(0.5, tilt)

    def process_network(self):
        if self.client is None:
            try:
                conn, addr = self.server.accept()
                conn.setblocking(False)
                self.client = conn
                print(f"[SimPixhawk] Mission script connected from {addr}!")
            except BlockingIOError:
                pass

        if self.client is not None:
            try:
                ready = select.select([self.client], [], [], 0)
                if ready[0]:
                    chunk = self.client.recv(8192).decode('utf-8')
                    if not chunk:
                        print("[SimPixhawk] Mission script disconnected.")
                        self.client.close()
                        self.client = None
                        self.armed = False
                        return
                    self.net_buffer += chunk
                    while '\n' in self.net_buffer:
                        line, self.net_buffer = self.net_buffer.split('\n', 1)
                        if not line.strip(): continue
                        try:
                            self.handle_command(json.loads(line))
                        except Exception: pass
            except Exception:
                pass

    def handle_command(self, cmd):
        action = cmd.get("action")
        if action == "arm":
            self.armed = bool(cmd.get("value"))
            print(f"[SimPixhawk] {'ARMED' if self.armed else 'DISARMED'}")
            self.send_response({"status": "ok"})

        elif action == "rc_override":
            self.rc_roll = cmd.get("roll", self.rc_roll)
            self.rc_pitch = cmd.get("pitch", self.rc_pitch)
            self.rc_throttle = cmd.get("throttle", self.rc_throttle)
            self.rc_yaw = cmd.get("yaw", self.rc_yaw)
            self.send_response({"status": "ok"})

        elif action == "set_velocity":
            self.target_vx = cmd.get("vx", 0.0)
            self.target_vy = cmd.get("vy", 0.0)
            self.target_vz = cmd.get("vz", 0.0)
            self.send_response({"status": "ok"})

        elif action == "set_target_alt":
            # Command from mission script to set a specific height
            new_target = float(cmd.get("value", 1.3))
            self.desired_altitude = new_target
            self.armed = True
            print(f"[SimPixhawk] TARGET ALTITUDE SET TO: {self.desired_altitude}m")
            self.send_response({"status": f"target_alt_set_{self.desired_altitude}"})

        elif action == "brake":
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.filt_pitch_dist = 0.0
            self.filt_roll_dist = 0.0
            self.brake_counter = 15 
            # Sync altitude target to current to prevent drift while braking
            self.target_altitude = self.get_altitude_internal()
            print("[SimPixhawk] ACTIVE ABS BRAKE ENGAGED")
            self.send_response({"status": "braking"})

        elif action == "hud":
            # Receive processed frame from mission script and mirror it to Webots Display
            if self.display and "b64" in cmd:
                try:
                    img_data = base64.b64decode(cmd["b64"])
                    img_array = np.frombuffer(img_data, dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    # Convert BGR to BGRA for Webots Display (4 channels)
                    frame_bgra = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
                    # To show in Webots Display, we have to draw onto it (640x480)
                    ir = self.display.imageNew(frame_bgra.tobytes(), Robot.BGRA, frame_bgra.shape[1], frame_bgra.shape[0])
                    self.display.imagePaste(ir, 0, 0, False)
                    self.display.imageDelete(ir)
                except Exception: pass
            self.send_response({"status": "ok"})

        elif action == "land":
            # Authoritative descent via desired_altitude slew
            self.desired_altitude = -0.2 
            print("[SimPixhawk] LAND MODE: Authoritative Descent (0.5m/s) ENGAGED")
            self.send_response({"status": "landing"})

        elif action == "get_lidar":
            # Return lidar altitude to mission scripts (GPS-denied interface)
            self.send_response({"lidar": self.get_altitude_lidar()})

        elif action == "get_camera":
            try:
                img_raw = self.camera.getImage()
                if img_raw:
                    w = self.camera.getWidth()
                    h = self.camera.getHeight()
                    # Convert BGRA (Webots) to BGR (OpenCV)
                    img_array = np.frombuffer(img_raw, np.uint8).copy().reshape((h, w, 4))
                    frame_bgr = np.ascontiguousarray(img_array[:, :, :3])
                    # FULL RESOLUTION 640x480 transmission for professional clarity
                    _, buf = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    b64 = base64.b64encode(buf).decode('utf-8')
                    self.send_response({"image": b64}) # SYNCED KEYNAME
                else:
                    self.send_response({"error": "no_image"})
            except Exception as e:
                self.send_response({"error": str(e)})

    def send_response(self, data):
        if self.client:
            try:
                self.client.sendall((json.dumps(data) + "\n").encode('utf-8'))
            except Exception:
                pass

    def flight_loop(self):
        """EXACT Cyberbotics PID with GPS altitude (simulates barometer)."""
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        altitude = self.get_altitude_internal()  # GPS Z as barometer
        roll_velocity = self.gyro.getValues()[0]
        pitch_velocity = self.gyro.getValues()[1]

        if not self.armed:
            for motor in self.motors:
                motor.setVelocity(1.0)
            return

        # Compute disturbances from TCP commands
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0
        dt = self.timestep / 1000.0

        if self.rc_throttle > 1550:
            self.target_altitude += 1.0 * dt
        elif self.rc_throttle < 1450 and self.rc_throttle > 1100:
            self.target_altitude -= 1.0 * dt

        if self.target_vz != 0:
            self.target_altitude -= self.target_vz * dt

        target_pitch_dist = 0.0
        target_roll_dist = 0.0
        if self.target_vx != 0:
            target_pitch_dist = -6.0 * clamp(self.target_vx, -2.5, 2.5) 
        if self.target_vy != 0:
            target_roll_dist = -6.0 * clamp(self.target_vy, -2.5, 2.5)

        # Apply low-pass filter to the disturbances!
        self.filt_pitch_dist = (self.alpha * target_pitch_dist) + ((1.0 - self.alpha) * self.filt_pitch_dist)
        self.filt_roll_dist = (self.alpha * target_roll_dist) + ((1.0 - self.alpha) * self.filt_roll_dist)
        
        # SLEW target_altitude towards desired_altitude (0.5m/s)
        if self.target_altitude < self.desired_altitude:
            self.target_altitude = min(self.desired_altitude, self.target_altitude + 0.5 * dt)
        elif self.target_altitude > self.desired_altitude:
            self.target_altitude = max(self.desired_altitude, self.target_altitude - 0.5 * dt)
        
        # ACTIVE BRAKING OVERRIDE
        if self.brake_counter > 0:
            # Force a brief tilt-BACK to cancel momentum (Highly stable)
            self.filt_pitch_dist = 1.2 
            self.brake_counter -= 1
        
        pitch_disturbance = self.filt_pitch_dist
        roll_disturbance = self.filt_roll_dist

        # Land detection
        if self.target_altitude <= 0.05 and self.get_altitude_internal() < 0.08:
            self.armed = False
            self.target_altitude = 0.1 # Reset for next takeoff
            print("[SimPixhawk] TOUCHDOWN. DISARMING.")

        self.target_altitude = clamp(self.target_altitude, 0.1, 40.0)

        # Yaw control: Hold target heading (GPS-denied mission usually stays North)
        current_yaw = self.imu.getRollPitchYaw()[2]
        yaw_velocity = self.gyro.getValues()[2]
        # Wrap yaw diff to -pi to pi
        yaw_diff = self.target_yaw - current_yaw
        while yaw_diff > math.pi: yaw_diff -= 2.0 * math.pi
        while yaw_diff < -math.pi: yaw_diff += 2.0 * math.pi
        
        yaw_input = K_YAW_P * yaw_diff - yaw_velocity + yaw_disturbance
        
        # RESTORED: Roll and Pitch PID
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        roll_velocity = self.gyro.getValues()[0]
        pitch_velocity = self.gyro.getValues()[1]
        
        roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_velocity + roll_disturbance
        pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance

        clamped_diff_alt = clamp(self.target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)
        vertical_input = K_VERTICAL_P * (clamped_diff_alt ** 3)

        fl = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
        fr = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
        rl = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
        rr = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

        self.front_left_motor.setVelocity(fl)
        self.front_right_motor.setVelocity(-fr)
        self.rear_left_motor.setVelocity(-rl)
        self.rear_right_motor.setVelocity(rr)

    def run_step(self):
        self.process_network()
        self.flight_loop()


def main():
    pix = SimPixhawk()
    while pix.robot.step(pix.timestep) != -1:
        pix.run_step()


if __name__ == "__main__":
    main()

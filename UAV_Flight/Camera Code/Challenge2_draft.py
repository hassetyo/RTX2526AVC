"""
Challenge 1 - UAV Precision Landing on Moving UGV
=================================================
Automated takeoff + Precision Loiter + Precision Land using MAVLink LANDING_TARGET.
Uses the robust CameraInterface from the user's friend's code.
"""
import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np
import os
import argparse
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
from pymavlink import mavutil
import v2v_bridge


# ─── USER CONFIGURATION ────────────────────────────────────────────────────────
MAVLINK_CONN = "/dev/ttyACM0"      # Or udp:127.0.0.1:14550
BAUD_RATE    = 921600

# 🔴 IMPORTANT: Set this to False when you go outside to actually fly!
# When True, it violently spoofs RC controllers to force motor sounds on your desk.
DESK_TESTING_NO_PROPELLERS = False

# mission params
HOVER_TIME_S = 8.0        # how long to hold altitude before landing
ALT_TOL = 0.12            # acceptable altitude error band
CLIMB_LOOP_DT = 0.10      # climb loop speed
HOVER_LOOP_DT = 0.10      # hover loop speed
LAND_LOOP_DT = 0.25       # landing print loop speed
LAND_TIMEOUT_S = 60.0     # safety timeout for landing
MOVEMENT_SPEED = 40       # used to control the speed of movements, too high will cause the drone to tilt too much and fall

# throttle settings i tuned
THROTTLE_MIN = 1000       # motors off / minimum throttle
THROTTLE_IDLE = 1150      # props spinning but no real lift
THROTTLE_CLIMB = 1650     # enough lift to climb
THROTTLE_HOVER = 1500     # mid-stick hover command for alt hold

# Nested ArUco Marker Setup
LARGE_MARKER_ID   = 5
SMALL_MARKER_ID   = 6
LARGE_MARKER_SIZE = 0.3048   # 12 inches -> meters
SMALL_MARKER_SIZE = 0.1016   # 4 inches -> meters

USE_SMALL_BELOW_M = 1.0
SEND_HZ = 15.0

# Flight Logic
TARGET_HEIGHT_M = 2.3
TAKEOFF_ALT_TOLERANCE_M = 0.25   # Start vision movement once we are within this band of target alt
TAKEOFF_MIN_CLIMB_M = 0.15       # Consider the vehicle airborne once it has climbed at least this much
TAKEOFF_TIMEOUT_S = 20.0
PLND_STABLE_FRAMES = 15   # Frames of solid tracking before engaging Precision Loiter
LAND_LATERAL_ERR_M = 0.20 # Meters of allowed lateral error before switching to LAND

# Error Handling
FAIL_COUNT = 0
MAX_FAIL_COUNT = 5


# ─── FRIEND'S ROBUST CAMERA & ARUCO CODE ───────────────────────────────────────
@dataclass
class MarkerPose:
    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    center_px: Tuple[int, int]
    corners: np.ndarray

class CameraInterface:
    def __init__(self, use_zed: bool = False, camera_index: int = 0, width: int = 1280, height: int = 720, fps: int = 30):
        self.use_zed = use_zed
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.zed = None
        self.sl = None
        self.camera_matrix = None
        self.dist_coeffs = None
        if self.use_zed:
            self._open_zed()
        else:
            self._open_standard()

    def _open_zed(self):
        try:
            import pyzed.sl as sl
        except ImportError as e:
            raise RuntimeError("pyzed.sl is not installed. Install the ZED SDK Python API first.") from e
        self.sl = sl
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = self.fps
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {err}")
        cam_info = self.zed.get_camera_information()
        calib = cam_info.camera_configuration.calibration_parameters.left_cam
        self.camera_matrix = np.array([[calib.fx, 0.0, calib.cx], [0.0, calib.fy, calib.cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array(calib.disto, dtype=np.float64).flatten()
        if dist.size >= 5: self.dist_coeffs = dist[:5].reshape(-1, 1)
        elif dist.size > 0: self.dist_coeffs = dist.reshape(-1, 1)
        else: self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def _open_standard(self):
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {self.camera_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def load_standard_calibration(self, yaml_path: Optional[str]):
        if yaml_path:
            fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
            K = fs.getNode("K").mat()
            D = fs.getNode("D").mat()
            fs.release()
            if K is not None and D is not None:
                self.camera_matrix = np.array(K, dtype=np.float64)
                self.dist_coeffs = np.array(D, dtype=np.float64)
                print(f"Loaded custom camera calibration from {yaml_path}")
            return

    def get_frame(self) -> Optional[np.ndarray]:
        if self.use_zed:
            if self.zed.grab() != self.sl.ERROR_CODE.SUCCESS: return None
            image = self.sl.Mat()
            self.zed.retrieve_image(image, self.sl.VIEW.LEFT)
            frame = image.get_data()
            if frame.shape[-1] == 4: frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            else: frame = frame.copy()
            return frame
        else:
            ret, frame = self.cap.read()
            if not ret: return None
            return frame

    def close(self):
        if self.use_zed and self.zed is not None: self.zed.close()
        if self.cap is not None: self.cap.release()
        cv2.destroyAllWindows()

class ArucoDistanceEstimator:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray, dictionary_name: int = aruco.DICT_6X6_1000):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary_name)
        if hasattr(aruco, "ArucoDetector"):
            self.detector_params = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            self.use_new_detector_api = True
        else:
            self.detector_params = aruco.DetectorParameters_create()
            self.detector = None
            self.use_new_detector_api = False

    def detect_markers(self, frame: np.ndarray, marker_size_m: float) -> Dict[int, MarkerPose]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_new_detector_api: corners, ids, _ = self.detector.detectMarkers(gray)
        else: corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        poses: Dict[int, MarkerPose] = {}
        if ids is None or len(ids) == 0: return poses

        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec = self._estimate_pose(corners[i], marker_size_m)
            if rvec is None or tvec is None: continue
            center = np.mean(corners[i][0], axis=0).astype(int)
            poses[int(marker_id)] = MarkerPose(
                marker_id=int(marker_id), rvec=rvec, tvec=tvec.reshape(3),
                center_px=(int(center[0]), int(center[1])), corners=corners[i][0]
            )
        return poses

    def _estimate_pose(self, corner: np.ndarray, marker_size_m: float):
        half = marker_size_m / 2.0
        object_points = np.array([[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float32)
        image_points = corner.reshape((4, 2)).astype(np.float32)
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success: return None, None
        return rvec, tvec

class UAVCommander:
    def __init__(self):
        self.log_file = "Challenge2_log.txt"

    def log_event(self, text):  # helper to write logs to terminal + text file
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] {text}"
        print(line)
        with open(self.log_file, "a") as f:
            f.write(line + "\n")

    def request_message_streams(self, master):  # asks the flight controller for the messages we care about
        try:
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,
                1,
            )
        except Exception:
            pass

        def set_interval(msg_id, hz):
            try:
                us = int(1e6 / hz)
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    msg_id,
                    us,
                    0, 0, 0, 0, 0,
                )
            except Exception:
                pass

        set_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 15)
        set_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
        set_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 5)
    
    def change_mode(self, master, *mode_names):  # changes the flight controller mode with a couple alias options
        mapping = master.mode_mapping()
        for mode in mode_names:
            if mode in mapping:
                mode_id = mapping[mode]
                master.mav.set_mode_send(
                    master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id,
                )
                self.log_event(f"Mode set: {mode}")
                time.sleep(1)
                return mode
        raise RuntimeError(f"None of these modes were found: {mode_names}. Available: {list(mapping.keys())}")
   

    def disarm_drone(self, master):  # emergency fallback if needed
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )
        self.log_event("Disarm command sent.")

    def set_throttle(self, master, pwm):  # pushes throttle by rc override
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0, 0, pwm, 0, 0, 0, 0, 0,
        )

    def set_rc_override(self, master, pitch = THROTTLE_HOVER, yaw = THROTTLE_HOVER, roll = THROTTLE_HOVER, throttle = THROTTLE_HOVER):  # helper to set multiple channels at once, default to hover for non-throttle channels
        x = throttle if pitch is None else pitch
        y = throttle if yaw is None else yaw
        z = throttle if roll is None else roll
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            x, y, z, 0, 0, 0, 0, 0,
        )


    def clear_rc_override(self, master):  # releases rc override back to the autopilot / radio
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    def get_optical_flow_position_x(self, master): # gets the position data from the optical flow sensor, if available
        msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
        while msg:
            if msg.quality > 0:
                pos_x = msg.flow_x
                return pos_x
            msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
        return None

    def get_optical_flow_position_y(self, master): # gets the position data from the optical flow sensor, if available
        msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
        while msg:
            if msg.quality > 0:
                pos_y = msg.flow_y
                return pos_y
            msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
        return None

    def get_optical_flow_quality(self, master): # gets the quality reading from the optical flow sensor, if available
        msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
        while msg:
            quality = msg.quality
            return quality
        return None

    def get_position_estimate(self,master): # combines the optical flow x and y to get an esitmate of the current position relative to the starting point, if available
        pos_x = self.get_optical_flow_position_x(master)
        pos_y = self.get_optical_flow_position_y(master)
        quality = self.get_optical_flow_quality(master)

        if pos_x is not None and pos_y is not None and quality is not None:
            return pos_x, pos_y, quality
        else:
            return None, None, None


    #Altitude handlers
    def get_rangefinder_alt(self, master):  # tries the downward sensor first
        msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
        while msg:
            current = msg.current_distance / 100.0
            if current > 0.01:
                return current
            msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
        return None

    def get_baro_relative_alt(self, master):  # fallback altitude from global position message
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        while msg:
            rel_alt_m = msg.relative_alt / 1000.0
            return rel_alt_m
        return None


    def get_altitude_m(self, master):  # unified altitude helper
        rng_alt = self.get_rangefinder_alt(master)
        if rng_alt is not None:
            return rng_alt, "rangefinder"

        baro_alt = self.get_baro_relative_alt(master)
        if baro_alt is not None:
            return baro_alt, "baro"

        return None, "none"


    def print_altitude(self, master, prefix="Altitude"):  # prints the current altitude every loop
        alt, source = self.get_altitude_m(master)
        if alt is None:
            print(f"{prefix}: waiting for altitude data...", end="\r", flush=True)
            return None

        print(f"{prefix}: {alt:5.2f} m  (source: {source})", end="\r", flush=True)
        return alt


    def wait_for_good_altitude(self, master, timeout_s=5.0):  # makes sure we actually have altitude data before flying
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            alt, source = self.get_altitude_m(master)
            if alt is not None:
                self.log_event(f"Altitude source ready: {source}, current altitude {alt:.2f} m")
                return
            time.sleep(0.1)
        raise RuntimeError("No altitude data received from rangefinder or barometer.")
    
    def get_relative_alt_m(self, master):
        """Best-effort relative altitude using autopilot telemetry."""
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg is not None and hasattr(msg, 'relative_alt'):
            return float(msg.relative_alt) / 1000.0  # mm -> m

        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg is not None and hasattr(msg, 'z'):
            return float(-msg.z)

        msg = master.recv_match(type='VFR_HUD', blocking=False)
        if msg is not None and hasattr(msg, 'alt'):
            return float(msg.alt)

        return None

    #flight handlers
    def climb_to_target(self, master, target_alt):  # manual climb like mission 4, then settle near target
        self.log_event(f"Climbing to {target_alt:.2f} m...")

        self.set_throttle(master, THROTTLE_IDLE)
        time.sleep(1.0)

        stable_start = None
        while True:
            alt = self.print_altitude(master, prefix="Climb Alt")

            if alt is None:
                self.set_throttle(master, THROTTLE_HOVER)
                time.sleep(CLIMB_LOOP_DT)
                continue

            # first push upward until close to target, then settle gently
            if alt < (target_alt - ALT_TOL):
                self.set_throttle(master, THROTTLE_CLIMB)
                stable_start = None
            else:
                self.set_throttle(master, THROTTLE_HOVER)

                # require the altitude be at or above the target for a short amount of time
                if alt > (target_alt - 0.50):
                    if stable_start is None:
                        stable_start = time.time()
                    elif (time.time() - stable_start) >= 1.2:
                        print()  # move off the carriage-return line cleanly
                        self.log_event(f"Target altitude reached and stabilized: {alt:.2f} m")
                        return
                else:
                    stable_start = None

            time.sleep(CLIMB_LOOP_DT)


    def hover_in_alt_hold(self, master, hover_time_s):  # switches to alt hold and keeps throttle centered
        self.change_mode(master, "ALT_HOLD", "ALTHOLD")
        self.log_event(f"Holding altitude for {hover_time_s:.1f} seconds...")

        start_t = time.time()
        while (time.time() - start_t) < hover_time_s:
            alt = self.print_altitude(master, prefix="Hover Alt")

            # in alt hold, keeping throttle near mid-stick tells the autopilot to maintain altitude
            self.set_throttle(master, THROTTLE_HOVER)

            # tiny trim if it drifts a lot while still keeping the command near mid-stick
            if alt is not None:
                if alt < TARGET_HEIGHT_M - 0.20:
                    self.set_throttle(master, THROTTLE_HOVER + 40)
                elif alt > TARGET_HEIGHT_M + 0.20:
                    self.set_throttle(master, THROTTLE_HOVER - 40)

            time.sleep(HOVER_LOOP_DT)

        print()
        self.log_event("Hover segment complete.")

    def get_yaw(self, master):
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        while msg:
            yaw = msg.hdg / 100.0  # convert from centidegrees to degrees
            return yaw
        return None
    
    def change_yaw(self, master, turnRight = True):
        self.log_event("Turning 90 degrees right...")
        # This is a 'timed' turn since we aren't reading compass/IMU yet
        # Adjust TURN_TIME based on your drone's sensitivity
        TURN_TIME = 0.75 
        TURN_DIRECTION = 1600 if turnRight else 1400  # 1600 for right, 1400 for left (assuming 1500 is neutral)
        start = time.time()
        while (time.time() - start) < TURN_TIME:
            # CH4 is Yaw. 1500 is neutral, 1600 is right rotation
            self.set_rc_override(master, yaw=TURN_DIRECTION, throttle=THROTTLE_HOVER)
            print(f"Current yaw: {self.get_yaw(master):.1f} degrees")
            time.sleep(0.1)
        self.set_rc_override(master, yaw=THROTTLE_HOVER, throttle=THROTTLE_HOVER)

    def move_pitch(self, master, forward = True, seconds = 1.5):
        move_pwm = 1500
        brake_pwm = 1500

        if(forward):
            direction = "forward"
            move_pwm -= MOVEMENT_SPEED
            brake_pwm += MOVEMENT_SPEED
        else:
            direction = "backwards"
            move_pwm += MOVEMENT_SPEED
            brake_pwm -= MOVEMENT_SPEED
            
        # 1. Tilt Forward
        self.log_event(f"Moving {direction}...")
        start_t = time.time()
        while (time.time() - start_t) < seconds:
            self.set_rc_override(master, pitch=move_pwm, throttle=THROTTLE_HOVER)
            time.sleep(0.1)

        # 2. Level Out
        self.log_event("Leveling... (Coasting)")
        self.set_rc_override(master, pitch=1500, throttle=THROTTLE_HOVER)
        time.sleep(1.0) # Drone is still moving forward here!

        # 3. Active Brake (Counter-Pitch)
        self.log_event("Applying brakes...")
        start_t = time.time()
        while (time.time() - start_t) < 0.4:
            self.set_rc_override(master, pitch=brake_pwm, throttle=THROTTLE_HOVER)
            time.sleep(0.1)

        # 4. Final Neutral
        self.set_rc_override(master, pitch=1500, throttle=THROTTLE_HOVER)
        self.log_event("Hovering at destination.")

    # ***
    # ***
    # ***
    # ***
    def send_body_velocity(self, master, vx: float, vy: float, vz: float):
        '''Send a body-frame velocity setpoint via SET_POSITION_TARGET_LOCAL_NED.
        vx = forward (m/s), vy = right (m/s), vz = down (m/s, positive = descend).
        Drone must already be in GUIDED mode for this to have effect.
        type_mask ignores position and acceleration, uses velocity only.'''
        FRAME_BODY_NED = 8
        TYPE_MASK_VEL_ONLY = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            master.target_system,
            master.target_component,
            FRAME_BODY_NED,
            TYPE_MASK_VEL_ONLY,
            0.0, 0.0, 0.0,   # position (ignored)
            vx, vy, vz,       # velocity
            0.0, 0.0, 0.0,   # acceleration (ignored)
            0.0, 0.0,         # yaw, yaw_rate (ignored)
        )

    def land_safely(self, master):
        # switches to land mode and then BLOCKS until the cube's heartbeat confirms
        # that the drone has actually disarmed (motors stopped) - no guessing from altitude.
        # a timeout fallback is still in place so we never hang forever.
        self.log_event("Landing sequence engaged...")
        self.change_mode(master, "LAND")
        self.clear_rc_override(master)  # let land mode fully control the descent

        self.log_event("Waiting for Cube heartbeat to confirm touchdown and auto-disarm...")

        deadline = time.time() + LAND_TIMEOUT_S  # hard fallback so we never hang forever
        while time.time() < deadline:
            # primary confirmation: cube heartbeat says motors are no longer armed
            hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)  # blocking wait up to 2s per beat
            if hb is not None:
                motors_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)  # check armed bit
                if not motors_armed:
                    # the cube itself told us the motors stopped - this is the real touchdown confirmation
                    print()  # move off carriage-return line
                    self.log_event("Cube heartbeat confirmed: motors disarmed. Touchdown complete.")
                    return  # landing confirmed, exit

            # also print altitude while we wait so the operator can see the descent
            self.print_altitude(master, prefix="Land Alt")
            time.sleep(LAND_LOOP_DT)

        # if we hit the deadline without a disarm heartbeat, log a warning but do not crash out
        print()
        self.log_event("[WARNING] Landing timeout reached without cube disarm confirmation. Sending disarm fallback.")
        self.disarm_drone(master)  # one last attempt to stop the motors
        time.sleep(2.0)  # give the cube a moment to process the disarm
        self.log_event("Disarm fallback sent. Assuming landed.")
    
    def connect(self):
        self.log_event("==========================================")
        self.log_event("   UAV MOVEMENT TEST + SAFE LAND MISSION")
        self.log_event("==========================================")

        self.log_event(f"Connecting to Drone: {MAVLINK_CONN}...")
        if MAVLINK_CONN.startswith("udp:") or MAVLINK_CONN.startswith("tcp:"):
            master = mavutil.mavlink_connection(MAVLINK_CONN)
        else:
            master = mavutil.mavlink_connection(MAVLINK_CONN, baud=BAUD_RATE)

        hb = master.wait_heartbeat(timeout=10)
        if not hb:
            self.log_event("No heartbeat received from drone. Check connection and try again.")
            return None
        self.log_event("Drone Heartbeat OK.")

        self.request_message_streams(master)

        return master
    
    #Denis' helpers
    def send_landing_target(self, master,x_b, y_b, z_b):
        """ Secondary: Broadcasts absolute marker target relative to flow """
        master.mav.landing_target_send(
            int(time.time() * 1e6), 0, mavutil.mavlink.MAV_FRAME_BODY_FRD, 0.0, 0.0,
            abs(z_b), 0.0, 0.0, x_b, y_b, abs(z_b), (1.0, 0.0, 0.0, 0.0), 0, 1
        )

    def send_guided_velocity(self, master,vx, vy, vz):
        """ Primary Flight: Smooth, safe, velocity vectors directly forcing the PIDs! """
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0, 0, 0
        )

    def send_rc_override(self, master, roll=1500, pitch=1500, throttle=1500, yaw=1500):
        """ Forcefully bypass ArduPilot's 'Not Flying' safety state for desk testing """
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[0] = int(roll)
        rc_channel_values[1] = int(pitch)
        rc_channel_values[2] = int(throttle)
        rc_channel_values[3] = int(yaw)
        master.mav.rc_channels_override_send(master.target_system, master.target_component, *rc_channel_values)

    def release_rc_override(self, master):
        self.send_rc_override(master, 0, 0, 0, 0)

    def change_mode(self, master, mode_name: str):
        mode_id = master.mode_mapping().get(mode_name)
        if mode_id is None:
            return False
        master.set_mode(mode_id)
        return True


    def wait_for_mode(self, master, mode_name: str, timeout: float = 8.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if master.flightmode == mode_name:
                return True
        return False

    def arm_with_timeout(self, master, timeout=10):
        master.arducopter_arm()
        start_time = time.time()
        while time.time() - start_time < timeout:
            if master.motors_armed():
                print("Motors Armed!")
                return True
            time.sleep(0.1)
        print("Failed to arm within timeout.")
        return False

    def arm_and_takeoff(self, master, alt):
        print("Switching to GUIDED...")
        if not self.change_mode(master, "GUIDED"):
            raise RuntimeError("GUIDED mode is not available on this vehicle")
        if not self.wait_for_mode(master, "GUIDED", timeout=8.0):
            raise RuntimeError(f"Vehicle never entered GUIDED mode (current: {master.flightmode})")

        print("Arming motors...")
        isArmed = self.arm_with_timeout(master, timeout=10)
        if not isArmed:
            raise RuntimeError("Failed to arm motors, cannot takeoff")
        
        print("Armed! Sending takeoff command...")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt
        )


def draw_crosshair(frame: np.ndarray):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)
    cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)


'''Controls the UGV navigation'''
class UGVCommander:
    def __init__(
        self,
        bridge_port: str,
        bridge_baud: int,
        turn_threshold_deg: float,
        stop_distance_m: float,
        step_min_m: float,
        step_max_m: float,
        drive_speed_mps: float,
        marker_timeout_sec: float,
    ):
        self.bridge = v2v_bridge.V2VBridge(bridge_port, baud=bridge_baud, name="UGV-Nav-Bridge")
        self.turn_threshold_deg = turn_threshold_deg
        self.stop_distance_m = stop_distance_m
        self.step_min_m = step_min_m
        self.step_max_m = step_max_m
        self.drive_speed_mps = drive_speed_mps
        self.marker_timeout_sec = marker_timeout_sec

        self.seq = 1
        self.arm_sent = False
        self.last_motion = "idle"
        self.next_drive_time = 0.0
        self.last_seen_time = 0.0
        self.last_status_text = "init"

    ''' Connect to the ESP32 bridge and send an initial message indicating that the UGV navigator is online.'''
    def connect(self):
        self.bridge.connect()
        self.bridge.send_message("aruco ugv navigator online")

    ''' Close the connection to the ESP32 bridge and send a stop command to ensure the UGV is not left moving.'''
    def close(self):
        try:
            self.send_stop(force=True)
        except Exception:
            pass
        self.bridge.stop()

    ''' Generate the next sequence number for commands sent to the bridge. Increments an internal counter and returns the current value.'''
    def _next_seq(self) -> int:
        val = self.seq
        self.seq += 1
        return val

    ''' Makes sure the UGV is armed before sending movement commands.'''
    def ensure_armed(self):
        if self.arm_sent:
            return
        print("[UGV] Sending ARM command over bridge...")
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_ARM, 0)
        self.arm_sent = True
        self.last_status_text = "arming sent"
        time.sleep(0.25)

    '''Tells the UGV to turn left if it is not already turning left.'''
    def send_turn_left(self):
        if self.last_motion == "turn_left":
            return
        print("[UGV] TURN LEFT")
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_TURN_LEFT, 0)
        self.last_motion = "turn_left"
        self.last_status_text = "turning left"

    '''Tells the UGV to turn right if it is not already turning right.'''
    def send_turn_right(self):
        if self.last_motion == "turn_right":
            return
        print("[UGV] TURN RIGHT")
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_TURN_RIGHT, 0)
        self.last_motion = "turn_right"
        self.last_status_text = "turning right"

    '''Tells the UGV to stop if its not already stopped or if the stop if forced.'''
    def send_stop(self, force: bool = False):
        if not force and self.last_motion == "stopped":
            return
        print("[UGV] STOP")
        self.bridge.send_command(self._next_seq(), v2v_bridge.CMD_STOP, 0)
        self.last_motion = "stopped"
        self.last_status_text = "stopped"

    '''Tells the UGV to move forward a certain step in meters.'''
    def send_forward_step(self, step_m: float):
        now = time.time()
        if now < self.next_drive_time:
            return

        step_m = max(self.step_min_m, min(step_m, self.step_max_m))
        duration = max(step_m / max(self.drive_speed_mps, 1e-6), 0.15)
        margin = 0.25

        print(f"[UGV] FORWARD STEP {step_m:.3f} m")
        self.bridge.send_message(f"GOTO:{step_m:.3f},0")
        self.last_motion = "forward"
        self.last_status_text = f"forward {step_m:.2f} m"
        self.next_drive_time = now + duration + margin

    ''' If the markers are lost for longer than the specified timeout, send a stop command to the UGV and update the status text.'''
    def handle_marker_loss(self):
        now = time.time()
        if self.last_seen_time <= 0.0:
            return
        if (now - self.last_seen_time) > self.marker_timeout_sec:
            self.send_stop()
            self.last_status_text = "marker lost -> stopped"


FORWARD_AXIS_MAP = {
    "+x": np.array([1.0, 0.0, 0.0], dtype=np.float64),
    "-x": np.array([-1.0, 0.0, 0.0], dtype=np.float64),
    "+y": np.array([0.0, 1.0, 0.0], dtype=np.float64),
    "-y": np.array([0.0, -1.0, 0.0], dtype=np.float64),
}

''' Calculate the forward direction of a marker in pixel coordinates. 
Returns a 2D vector pointing in the forward direction of the marker or None if the direction cannot be determined.'''
def get_marker_forward_direction_px(
    pose: MarkerPose,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_size_m: float,
    forward_axis_name: str,
) -> Optional[np.ndarray]:
    axis = FORWARD_AXIS_MAP[forward_axis_name]
    tip_local = axis * (marker_size_m * 0.75)

    pts3d = np.array(
        [
            [0.0, 0.0, 0.0],
            tip_local,
        ],
        dtype=np.float32,
    )

    img_pts, _ = cv2.projectPoints(
        pts3d,
        pose.rvec,
        pose.tvec.reshape(3, 1),
        camera_matrix,
        dist_coeffs,
    )
    img_pts = img_pts.reshape(-1, 2)

    center = img_pts[0]
    tip = img_pts[1]
    direction = tip - center

    if np.linalg.norm(direction) < 1e-6:
        return None
    return direction

''' Calculate the signed angle in degrees between two 2D vectors. The angle is positive if vec_b is counterclockwise from vec_a, and negative if clockwise.'''
def signed_angle_deg(vec_a: np.ndarray, vec_b: np.ndarray) -> float:
    a = np.asarray(vec_a, dtype=np.float64).reshape(2)
    b = np.asarray(vec_b, dtype=np.float64).reshape(2)

    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return 0.0

    a /= na
    b /= nb

    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    cross = float(a[0] * b[1] - a[1] * b[0])
    return math.degrees(math.atan2(cross, dot))

''' Pick two markers from the detected poses. If specific marker IDs are provided, try to find those. 
    Otherwise, pick the two markers with the lowest IDs. 
    Returns a tuple of the two MarkerPose objects or None if not enough markers are detected.'''
def pick_two_markers(
    poses: Dict[int, MarkerPose],
    marker1_id: Optional[int],
    marker2_id: Optional[int],
) -> Optional[Tuple[MarkerPose, MarkerPose]]:
    if len(poses) < 2:
        return None

    if marker1_id is not None and marker2_id is not None:
        if marker1_id in poses and marker2_id in poses:
            return poses[marker1_id], poses[marker2_id]
        return None

    ordered_ids = sorted(poses.keys())
    return poses[ordered_ids[0]], poses[ordered_ids[1]]


''' Draw a navigation overlay on the frame showing the UGV and destination markers, the heading error, distance, and status text.'''
def draw_nav_overlay(
    frame: np.ndarray,
    ugv_pose: MarkerPose,
    dest_pose: MarkerPose,
    heading_error_deg: float,
    status_text: str,
    stop_distance_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_size_m: float,
    forward_axis_name: str,
):
    ugv_center = np.array(ugv_pose.center_px, dtype=np.int32)
    dst_center = np.array(dest_pose.center_px, dtype=np.int32)

    fwd_dir = get_marker_forward_direction_px(
        ugv_pose,
        camera_matrix,
        dist_coeffs,
        marker_size_m,
        forward_axis_name,
    )

    cv2.arrowedLine(
        frame,
        tuple(ugv_center),
        tuple(dst_center),
        (0, 255, 255),
        2,
        tipLength=0.08,
    )

    if fwd_dir is not None:
        fwd_tip = ugv_center + np.round(fwd_dir).astype(np.int32)
        cv2.arrowedLine(
            frame,
            tuple(ugv_center),
            tuple(fwd_tip),
            (255, 0, 255),
            2,
            tipLength=0.18,
        )

    diff = dest_pose.tvec.reshape(3) - ugv_pose.tvec.reshape(3)
    dist_m = float(np.linalg.norm(diff))

    lines = [
        f"UGV ID: {ugv_pose.marker_id}",
        f"DEST ID: {dest_pose.marker_id}",
        f"Heading error: {heading_error_deg:+.1f} deg",
        f"Distance: {dist_m:.3f} m",
        f"Stop dist: {stop_distance_m:.3f} m",
        f"State: {status_text}",
    ]

    x0, y0 = 20, 20
    padding = 10
    line_h = 28
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.7
    thickness = 2

    max_w = 0
    for text in lines:
        (tw, _), _ = cv2.getTextSize(text, font, scale, thickness)
        max_w = max(max_w, tw)

    box_w = max_w + 2 * padding
    box_h = len(lines) * line_h + 2 * padding

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (255, 255, 255), -1)
    frame[:] = cv2.addWeighted(overlay, 0.82, frame, 0.18, 0)

    y = y0 + padding + 20
    for text in lines:
        cv2.putText(
            frame,
            text,
            (x0 + padding, y),
            font,
            scale,
            (0, 0, 0),
            thickness,
            cv2.LINE_AA,
        )
        y += line_h

''' Parse command-line arguments for configuring the camera, ArUco detection, and UGV control parameters.'''
def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Measure ArUco marker distance and command the ground vehicle marker to drive toward the destination marker."
        )
    )
    parser.add_argument("--use-zed", action="store_true", help="Use Stereolabs ZED / ZED X camera.")
    parser.add_argument("--camera-index", type=int, default=0, help="Standard camera index for laptop/USB webcam.")
    parser.add_argument("--calibration", type=str, default=None, help="YAML calibration file for standard camera with nodes K and D.")
    parser.add_argument("--marker-size", type=float, default=0.254, help="Marker size in meters. Example: 0.254 for 10 inches.")
    parser.add_argument("--ugv-marker-id", type=int, default=5, help="ArUco ID attached to the ground vehicle.")
    parser.add_argument("--dest-marker-id", type=int, default=0, help="Destination ArUco ID.")
    parser.add_argument("--dict", type=str, default="DICT_6X6_1000", help="ArUco dictionary name.")
    parser.add_argument("--width", type=int, default=1280, help="Standard camera width.")
    parser.add_argument("--height", type=int, default=720, help="Standard camera height.")
    parser.add_argument("--fps", type=int, default=60, help="Camera FPS.")

    parser.add_argument("--bridge-port", type=str, default="/dev/ttyUSB0", help="Local ESP32 bridge serial port used to send commands.")
    parser.add_argument("--bridge-baud", type=int, default=115200, help="ESP32 bridge baud rate.")

    parser.add_argument("--turn-threshold-deg", type=float, default=12.0, help="Heading error magnitude required before issuing turn commands.")
    parser.add_argument("--stop-distance-m", type=float, default=0.28, help="Distance at which the UGV is considered to have reached the destination.")
    parser.add_argument("--step-min-m", type=float, default=0.20, help="Minimum forward body step to command.")
    parser.add_argument("--step-max-m", type=float, default=0.60, help="Maximum forward body step to command.")
    parser.add_argument("--drive-speed-mps", type=float, default=1.5, help="Expected ground-station forward speed used only for cooldown timing.")
    parser.add_argument("--marker-timeout-sec", type=float, default=0.75, help="Stop the UGV if markers disappear longer than this timeout.")
    parser.add_argument(
        "--ugv-forward-axis",
        choices=["+x", "-x", "+y", "-y"],
        default="+y",
        help="Which marker local axis points in the UGV forward direction.",
    )
    return parser.parse_args()

''' Get the ArUco dictionary constant from OpenCV by name. Raises a ValueError if the name is not valid.'''
def get_dictionary_by_name(name: str) -> int:
    if not hasattr(aruco, name):
        valid = [x for x in dir(aruco) if x.startswith("DICT_")]
        raise ValueError(f"Unknown dictionary '{name}'. Valid examples: {valid[:10]}")
    return getattr(aruco, name)

'''Initialize the camera, ArUco estimator, and UGV commander before entering the main functionality'''
def initialize_system(args):
    dictionary = get_dictionary_by_name(args.dict)

    cam = CameraInterface(
        use_zed=args.use_zed,
        camera_index=args.camera_index,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )

    if not args.use_zed:
        cam.load_standard_calibration(args.calibration)

    estimator = ArucoDistanceEstimator(
        camera_matrix=cam.camera_matrix,
        dist_coeffs=cam.dist_coeffs,
        marker_size_m=args.marker_size,
        dictionary_name=dictionary,
    )

    commander = UGVCommander(
        bridge_port=args.bridge_port,
        bridge_baud=args.bridge_baud,
        turn_threshold_deg=args.turn_threshold_deg,
        stop_distance_m=args.stop_distance_m,
        step_min_m=args.step_min_m,
        step_max_m=args.step_max_m,
        drive_speed_mps=args.drive_speed_mps,
        marker_timeout_sec=args.marker_timeout_sec,
    )
    commander.connect()

    return cam, estimator, commander

''' Draw a line between two markers and annotate the distance and relative position. 
    Also displays a status box with detailed information about the markers and their relative pose.'''
def draw_distance_overlay(
    frame: np.ndarray,
    pose_a: MarkerPose,
    pose_b: MarkerPose,
):
    p1 = pose_a.tvec.reshape(3)
    p2 = pose_b.tvec.reshape(3)

    diff = p2 - p1
    dist_m = np.linalg.norm(diff)

    x_diff_mm = diff[0] * 1000.0
    y_diff_mm = diff[1] * 1000.0
    z_diff_mm = diff[2] * 1000.0
    dist_mm = dist_m * 1000.0

    cv2.line(frame, pose_a.center_px, pose_b.center_px, (0, 255, 0), 3)

    mid_x = (pose_a.center_px[0] + pose_b.center_px[0]) // 2
    mid_y = (pose_a.center_px[1] + pose_b.center_px[1]) // 2

    cv2.putText(
        frame,
        f"{dist_mm:.1f} mm",
        (mid_x + 10, mid_y - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    lines = [
        f"Marker A ID: {pose_a.marker_id}",
        f"Marker B ID: {pose_b.marker_id}",
        f"Z Diff is : {z_diff_mm:.2f} [mm]",
        f"Y Diff is : {y_diff_mm:.2f} [mm]",
        f"X Diff is : {x_diff_mm:.2f} [mm]",
        f"Distance is : {dist_mm:.2f} [mm]",
    ]

    padding = 12
    line_h = 34
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.9
    thickness = 2

    max_w = 0
    for text in lines:
        (tw, _), _ = cv2.getTextSize(text, font, scale, thickness)
        max_w = max(max_w, tw)

    box_w = max_w + padding * 2
    box_h = line_h * len(lines) + padding * 2
    x0 = 20
    y0 = frame.shape[0] - box_h - 20

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (255, 255, 255), -1)
    frame[:] = cv2.addWeighted(overlay, 0.82, frame, 0.18, 0)

    y = y0 + padding + 24
    for text in lines:
        cv2.putText(
            frame,
            text,
            (x0 + padding, y),
            font,
            scale,
            (0, 0, 0),
            thickness,
            cv2.LINE_AA,
        )
        y += line_h


'''Main ArUco navigation function'''
def guideUGV(args, commander, estimator, cam):

    print(
        f"Tracking UGV marker {args.ugv_marker_id} toward destination marker {args.dest_marker_id}."
    )

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                print("Failed to read frame.")
                commander.handle_marker_loss()
                continue

            poses = estimator.detect_markers(frame)
            display = frame.copy()

            draw_crosshair(display)
            estimator.draw_markers(display, poses)

            pair = pick_two_markers(poses, args.ugv_marker_id, args.dest_marker_id)
            if pair is not None:
                ugv_pose, dest_pose = pair
                commander.last_seen_time = time.time()
                commander.ensure_armed()

                draw_distance_overlay(display, ugv_pose, dest_pose)

                forward_dir = get_marker_forward_direction_px(
                    ugv_pose,
                    cam.camera_matrix,
                    cam.dist_coeffs,
                    args.marker_size,
                    args.ugv_forward_axis,
                )
                target_dir = np.array(
                    [
                        dest_pose.center_px[0] - ugv_pose.center_px[0],
                        dest_pose.center_px[1] - ugv_pose.center_px[1],
                    ],
                    dtype=np.float64,
                )
                heading_error_deg = signed_angle_deg(forward_dir if forward_dir is not None else target_dir, target_dir)
                dist_m = float(np.linalg.norm(dest_pose.tvec.reshape(3) - ugv_pose.tvec.reshape(3)))

                if dist_m <= args.stop_distance_m:
                    commander.send_stop()
                    commander.last_status_text = "destination reached"
                elif abs(heading_error_deg) > args.turn_threshold_deg:
                    if heading_error_deg > 0.0:
                        commander.send_turn_right()
                    else:
                        commander.send_turn_left()
                else:
                    if commander.last_motion in ("turn_left", "turn_right"):
                        commander.send_stop()
                        time.sleep(0.15)

                    remaining = max(0.0, dist_m - args.stop_distance_m)
                    step_m = max(args.step_min_m, min(remaining * 0.5, args.step_max_m))
                    commander.send_forward_step(step_m)

                draw_nav_overlay(
                    display,
                    ugv_pose,
                    dest_pose,
                    heading_error_deg,
                    commander.last_status_text,
                    args.stop_distance_m,
                    cam.camera_matrix,
                    cam.dist_coeffs,
                    args.marker_size,
                    args.ugv_forward_axis,
                )
            else:
                commander.handle_marker_loss()
                cv2.putText(
                    display,
                    f"Need markers {args.ugv_marker_id} and {args.dest_marker_id}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    display,
                    f"State: {commander.last_status_text}",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow("ArUco Distance + UGV Navigation", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                cv2.imwrite("aruco_distance_ugv_nav_screenshot.png", display)
                print("Saved: aruco_distance_ugv_nav_screenshot.png")

    finally:
        commander.close()
        cam.close()

''' Tells the drone to move to the center of the grid and increase altitude until both markers are visible, then switch to the main navigation loop. This can help in cases where the markers are initially too close or partially out of frame.'''
def moveToCenter(master, UAVcommander, estimator, cam, args, bottomRight=True):
    """
    Commands the drone to fly toward the center of the search area while 
    climbing, using the UAVCommander's guided velocity methods.
    """
    print("[SEARCH] Moving toward center while climbing and searching for markers...")

    try:
        FAIL_COUNT = 0

        # Field math: Distance from corner to center of a 15x15 yard field
        distance_to_center_m = (15.0 / math.sqrt(2.0)) * 0.9144
        forward_speed_mps = 1.5
        total_time = distance_to_center_m / forward_speed_mps
        
        # Timing for the velocity loop (10Hz for smooth control)
        loop_hz = 10.0  
        total_steps = int(total_time * loop_hz)

        for step in range(total_steps):
            frame = cam.get_frame()
            if frame is None:
                print("[SEARCH] Failed to read frame.")
                FAIL_COUNT += 1
                if FAIL_COUNT > 5:
                    return 0
                continue

            # Detection Phase
            poses = estimator.detect_markers(frame, LARGE_MARKER_SIZE)
            pair = pick_two_markers(poses, args.ugv_marker_id, args.dest_marker_id)

            # --- SUCCESS: Both Markers Found ---
            if pair is not None:
                ugv_pose, dest_pose = pair
                print(f">>> Markers detected! UGV={ugv_pose.marker_id}, DEST={dest_pose.marker_id}")
                UAVcommander.log_event(f"Markers found at step {step}")
                
                # Stop movement immediately
                UAVcommander.send_guided_velocity(master, 0.0, 0.0, 0.0)
                return 1

            # --- MOVEMENT CALCULATION ---
            # Get current altitude using the commander's telemetry helper
            current_alt = UAVcommander.get_relative_alt_m(master)
            vz = 0.0
            
            # If we are too low to see both markers, climb slowly while moving
            if current_alt is not None and current_alt < 5.0:
                # vz is negative to move UP in the NED (North-East-Down) frame
                vz = -0.3 
            
            if DESK_TESTING_NO_PROPELLERS:
                if step % 10 == 0: # Status update every 1 second
                    print(f"[DESK TEST] Alt: {current_alt if current_alt else 0:.2f}m | Cmd: VX=1.5, VZ={vz}")
            else:
                # Primary movement: Forward at 1.5m/s and Climbing at 0.3m/s
                UAVcommander.send_guided_velocity(master, vx=forward_speed_mps, vy=0.0, vz=vz)

            time.sleep(1.0 / loop_hz)

        print("[SEARCH] Reached field center point. Markers not seen yet. Climbing vertically...")

        # --- SECONDARY SEARCH: Vertical Ascent at Center ---
        while True:
            frame = cam.get_frame()
            if frame is None: continue

            poses = estimator.detect_markers(frame, LARGE_MARKER_SIZE)
            pair = pick_two_markers(poses, args.ugv_marker_id, args.dest_marker_id)

            if pair is not None:
                print(">>> Markers found during vertical climb!")
                UAVcommander.send_guided_velocity(master, 0.0, 0.0, 0.0)
                return 1

            current_alt = UAVcommander.get_relative_alt_m(master)
            if current_alt is not None and current_alt >= 5.0:
                print("[SEARCH] Max search altitude (5m) reached. Aborting.")
                UAVcommander.send_guided_velocity(master, 0.0, 0.0, 0.0)
                return 0

            if not DESK_TESTING_NO_PROPELLERS:
                # Climb straight up (vz = -0.4) while keeping XY position stationary
                UAVcommander.send_guided_velocity(master, 0.0, 0.0, -0.4)
            
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("[SEARCH] Keyboard interrupt received.")
        return -1
    finally:
        FAIL_COUNT = 0

def endCode(master, UAVcommander, UGVcommander, cam):
    print("Ending program and landing drone safely...")
    time.sleep(2.0)
    UAVcommander.land_safely(master)
    cam.close()
    UGVcommander.close()


def loiter_landing_sequence(master, estimator, UAVcommander, cam, args):
    """
        notes: make the drone land using the code in main
    change the parameters if needed 
    if an error occurs force it go go into regular land mode
        no need to handle errors at first, 
        do it when we know the code is properly integrated
    - Sam

    Challenge 1: Automated Precision Landing Sequence
    This function handles the 'brain' of the landing: centering over the target
    using computer vision and descending once alignment is perfect.
    """
    print(">>> Starting Challenge 1: Precision Landing Sequence")
    
    # Internal state tracking
    state = "APPROACH"
    stable_count = 0  # Used to ensure we aren't reacting to 'flicker' detection
    last_send = 0.0
    
    # Proportional Gain: Higher = more aggressive centering, Lower = smoother
    Kp = 0.8 

    try:
        while True:
            # --- 1. CAPTURE & DETECTION ---
            frame = cam.get_frame()
            if frame is None:
                continue

            # We start by looking for the LARGE marker (ID 5) to get in the ballpark
            poses = estimator.detect_markers(frame, LARGE_MARKER_SIZE)
            
            target_eb = None
            marker_id_to_use = None
            marker_size_m_to_use = None

            # NESTED LOGIC: If we see the small marker (ID 6), immediately prioritize it.
            # Small markers are more accurate for the final 'touchdown' centimeters.
            if SMALL_MARKER_ID in poses:
                poses = estimator.detect_markers(frame, SMALL_MARKER_SIZE)
                if SMALL_MARKER_ID in poses:
                    marker_id_to_use = SMALL_MARKER_ID
                    marker_size_m_to_use = SMALL_MARKER_SIZE
            elif LARGE_MARKER_ID in poses:
                marker_id_to_use = LARGE_MARKER_ID
                marker_size_m_to_use = LARGE_MARKER_SIZE

            # --- 2. COORDINATE TRANSFORMATION ---
            if marker_id_to_use is not None:
                pose = poses[marker_id_to_use]
                
                # ArUco 'tvec' is in Camera Frame: X=Right, Y=Down, Z=Forward
                # ArduPilot expects Body FRD: X=Forward, Y=Right, Z=Down
                # This mapping aligns the camera's view with the drone's physical axes.
                x_b = -pose.tvec[1]  # Camera 'Down' becomes Drone 'Forward'
                y_b =  pose.tvec[0]  # Camera 'Right' stays Drone 'Right'
                z_b =  pose.tvec[2]  # Camera 'Forward' becomes Drone 'Down' (Distance to ground)
                target_eb = (x_b, y_b, z_b)
                
                stable_count += 1
                estimator.draw_markers(frame, poses) # Visual debug
            else:
                # If no marker is seen, reset the stability counter
                stable_count = 0

            # --- 3. TIMED COMMAND SENDING ---
            # We only send MAVLink messages at a specific frequency (SEND_HZ)
            # to avoid flooding the flight controller's serial buffer.
            now = time.time()
            if now - last_send >= (1.0 / SEND_HZ):
                if target_eb:
                    # Calculate total horizontal distance from target center
                    err_m = math.sqrt(target_eb[0]**2 + target_eb[1]**2)
                    
                    # Calculate velocity vectors (v = error * gain)
                    # We cap these at 0.7 m/s for safety.
                    vx = max(-0.7, min(0.7, float(target_eb[0]) * Kp))
                    vy = max(-0.7, min(0.7, float(target_eb[1]) * Kp))
                    vz = 0.0 # Default to no vertical movement while centering

                    # STATE: APPROACH - Moving toward the marker area
                    if state == "APPROACH":
                        if stable_count >= PLND_STABLE_FRAMES:
                            state = "PREC_LOITER"
                            print(">>> Tracking stable. Centering...")

                    # STATE: PREC_LOITER - Fine-tuning the position over the center
                    elif state == "PREC_LOITER":
                        # Once error is less than 20cm, we are 'centered' enough to land
                        if err_m < LAND_LATERAL_ERR_M:
                            state = "LANDING"
                            print(">>> Centered! Descending...")

                    # STATE: LANDING - Direct vertical descent while maintaining XY center
                    elif state == "LANDING":
                        # Check Lidar for true height. If it fails, use a constant 0.4m/s descent.
                        lidar_m = UAVCommander.print_altitude(master)
                        vz = max(0.15, min(0.4, lidar_m * 0.3)) if lidar_m else 0.4
                    
                    # Push the calculated velocities to the motors
                    UAVCommander.send_guided_velocity(master, vx, vy, vz)
                    # Send telemetry update so the Ground Station (Mission Planner) sees the target
                    UAVCommander.send_landing_target(master, target_eb[0], target_eb[1], target_eb[2])
                else:
                    # SAFETY: If we lose the marker, stop moving immediately to prevent drifting
                    UAVCommander.send_guided_velocity(master, 0.0, 0.0, 0.0)
                
                last_send = now

            # --- 4. COMPLETION & TERMINATION ---
            # Check if ArduPilot has disarmed (detected touchdown via pressure sensor/IMU)
            if state == "LANDING" and not master.motors_armed():
                print(">>> Touchdown! Sequence Complete.")
                break

            # Feedback Window
            cv2.putText(frame, f"STATE: {state}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Precision Landing", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        # GLOBAL FAILSAFE: If anything breaks in the code, force a standard RTL or LAND
        print(f"CRITICAL CODE ERROR: {e}")
        UAVCommander.change_mode(master, "LAND")

# ─── MAIN PROGRAM ──────────────────────────────────────────────────────────────
def main():
    # 1. Initialize variables and configurations
    print("\n[STEP 1] Initializing system variables and arguments...")
    args = parse_args()
    
    # Placeholder for the UAV Commander (Assumes it's in a separate file or defined class)
    # Based on your base logic: 
    uav_boss = UAVCommander()
    
    master = None
    cam = None
    estimator = None
    ugv_commander = None

    try:
        # 2. Connect to the Drone
        print(f"[STEP 2] Connecting to UAV at {MAVLINK_CONN}...")
        master = uav_boss.connect()

        if master is not None:
            print(">>> UAV Connection Successful.")
            
            # 3. Takeoff
            print(f"[STEP 3] Initiating Takeoff to {TARGET_HEIGHT_M}m...")
            uav_boss.arm_and_takeoff(master, alt=TARGET_HEIGHT_M)
            
            # 4. Initialize Vision and UGV Bridge
            print("[STEP 4] Initializing Camera and UGV Bridge...")
            cam, estimator, ugv_commander = initialize_system(args)
            
            # 5. Move to Center (Grid Search/Climb logic)
            print("[STEP 5] Running 'moveToCenter' sequence...")
            did_it_move = moveToCenter(master, uav_boss, estimator, cam, args)
            
            if did_it_move == 1:
                print(">>> Successfully moved to center and detected markers.")
                
                # 6. Guide UGV (Command Ground Vehicle)
                print("[STEP 6] Starting UGV Navigation logic...")
                # Note: This function runs until the UGV reaches its destination or 'q' is pressed
                guideUGV(args, ugv_commander, estimator, cam)
                print(">>> UGV Navigation phase completed.")

                # 7. Precision Landing (The final drop)
                print("[STEP 7] Initiating UAV Precision Landing Sequence...")
                loiter_landing_sequence(master, estimator, uav_boss, cam, args)
                print(">>> Precision Landing Sequence completed.")

            elif did_it_move == 0:
                print("[ERROR] Logic Error in moveToCenter: Markers never found.")
                endCode(master, uav_boss, ugv_commander, cam)
                return
                
            elif did_it_move == -1:
                print("[EXIT] User requested exit during moveToCenter.")
                endCode(master, uav_boss, ugv_commander, cam)
                return
        else:
            # Handle connection failure
            if DESK_TESTING_NO_PROPELLERS:
                print("[TESTING] Running in DESK MODE: Skipping drone connection/Takeoff.")
                # You might want to initialize system anyway for vision testing
                cam, estimator, ugv_commander = initialize_system(args)
                guideUGV(args, ugv_commander, estimator, cam)
            else:
                print("[CRITICAL] Failed to connect to drone. Check your serial port/telemetry.")
                return

        print("\nMission complete. Cleaning up resources...")
        endCode(master, uav_boss, ugv_commander, cam)

    except KeyboardInterrupt:
        print("\n[HALT] Keyboard interrupt received. Emergency landing initiated.")
        if master and uav_boss:
            endCode(master, uav_boss, ugv_commander, cam)
        
    except Exception as e:
        print(f"\n[CRITICAL ERROR] An unexpected error occurred: {e}")
        # Final safety failsafe
        if master:
            print("Attempting emergency land due to script crash...")
            UAVCommander.change_mode(master, "LAND")
        if cam:
            cam.close()
        if ugv_commander:
            ugv_commander.close()

if __name__ == "__main__":
    main()
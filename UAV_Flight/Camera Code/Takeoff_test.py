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
            print(f"Hovering... Time remaining: {hover_time_s - (time.time() - start_t):.1f} s", end="\r", flush=True)
            alt, source = self.get_altitude_m(master, prefix="Hover Alt")

            # in alt hold, keeping throttle near mid-stick tells the autopilot to maintain altitude
            self.set_throttle(master, THROTTLE_HOVER)

            # tiny trim if it drifts a lot while still keeping the command near mid-stick
            if alt is not None:
                if alt < TARGET_HEIGHT_M - 0.20:
                    print(f"Altitude low: {alt:.2f} m, rising")
                    self.send_guided_velocity(master, vx=0, vy=0, vz=-0.1)  # small upward velocity command
                elif alt > TARGET_HEIGHT_M + 0.20:
                    print(f"Altitude high: {alt:.2f} m, descending")
                    self.send_guided_velocity(master, vx=0, vy=0, vz=0.1)   # small downward velocity command

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


def main():
    print("UAV Takeoff Test Starting...")
    uav_boss = UAVCommander()
    
    master = uav_boss.connect()
    if master is not None:
        print(">>> UAV Connection Successful.")
        uav_boss.wait_for_good_altitude(master, timeout_s=10.0)
        print(">>> Starting Takeoff Sequence...")
        uav_boss.arm_and_takeoff(master, alt=TARGET_HEIGHT_M)

        alt, source = uav_boss.get_altitude_m(master)
        takeoff_start_time = time.time()

        while alt < (TARGET_HEIGHT_M - TAKEOFF_ALT_TOLERANCE_M):
            alt, source = uav_boss.get_altitude_m(master)
            takeoff_duration = time.time() - takeoff_start_time
            print(f"Taking off... Current altitude: {alt:.2f} m, Time elapsed: {takeoff_duration:.1f}/{TAKEOFF_TIMEOUT_S} s", end="\r", flush=True)
            if(takeoff_duration > TAKEOFF_TIMEOUT_S):
                print("\n>>> Takeoff timeout reached. Landing for safety.")
                uav_boss.land_safely(master)
                return
            time.sleep(0.1)

        print(f">>> Takeoff complete. Current altitude: {alt:.2f} m")
        
        uav_boss.hover_in_alt_hold(master, hover_time_s=HOVER_TIME_S)
        uav_boss.land_safely(master)
    else:
        print(">>> UAV Connection Failed.")


if __name__ == "__main__":
    main()
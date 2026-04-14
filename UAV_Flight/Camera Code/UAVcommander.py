'''Contains the simple movement commands for the UAV, such as takeoff, land, and move to a location.
   '''

from pymavlink import mavutil  # using the confirmed mavlink pattern instead of dronekit
import time  # for timing and sleeps
import math  # for simple comparisons

# connection settings
CONNECTION_STRING = "/dev/ttyACM0"   # change to "udp:127.0.0.1:14551" for SITL if needed
BAUD_RATE = 57600                     # serial speed for hardware connections

# mission params
TARGET_ALT = 2.3          # target hover height in meters
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


class UAVCommander:
    def __init__(self):
        self.log_file = "uav_log.txt"

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
    
    def arm_drone(self, master):  # engages the motors
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        self.log_event("Arming motors...")

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
    
    def climb_to_target(self, master, target_alt):  # manual climb like mission 4, then settle near target
        self.log_event(f"Climbing to {target_alt:.2f} m...")

        self.set_throttle(master, THROTTLE_IDLE)
        time.sleep(1.0)

        stable_start = None
        while True:
            alt = self.print_altitude(master, prefix="Climb Alt")

            if alt is None:
                self.set_throttle(master, THROTTLE_IDLE)
                time.sleep(CLIMB_LOOP_DT)
                continue

            # first push upward until close to target, then settle gently
            if alt < (target_alt - ALT_TOL):
                self.set_throttle(master, THROTTLE_CLIMB)
                stable_start = None
            else:
                self.set_throttle(master, THROTTLE_HOVER)

                # require the altitude to stay near target briefly before switching to alt hold
                if abs(alt - target_alt) <= 0.50:
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
                if alt < TARGET_ALT - 0.20:
                    self.set_throttle(master, THROTTLE_HOVER + 40)
                elif alt > TARGET_ALT + 0.20:
                    self.set_throttle(master, THROTTLE_HOVER - 40)

            time.sleep(HOVER_LOOP_DT)

        print()
        self.log_event("Hover segment complete.")

    def get_yaw(self, master):
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        while msg:
            yaw = math.degrees(msg.hdg / 100.0)  # convert from centidegrees to degrees
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
            print(f"Current yaw: {self.get_yaw(master):.1f} degrees", end="\r", flush=True)
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

        self.log_event(f"Connecting to Drone: {CONNECTION_STRING}...")
        if CONNECTION_STRING.startswith("udp:") or CONNECTION_STRING.startswith("tcp:"):
            master = mavutil.mavlink_connection(CONNECTION_STRING)
        else:
            master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)

        hb = master.wait_heartbeat(timeout=10)
        if not hb:
            return None
        self.log_event("Drone Heartbeat OK.")

        self.request_message_streams(master)

        return master
    
    def takeoff(self, master, target_alt=TARGET_ALT):
        # step 1: start in stabilize like your mission 4 pattern
        self.change_mode(master, "STABILIZE")
        self.arm_drone(master)

        # step 2: climb to the requested height
        self.climb_to_target(master, target_alt)
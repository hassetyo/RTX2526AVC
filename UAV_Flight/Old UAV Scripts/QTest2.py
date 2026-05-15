from pymavlink import mavutil  # using the confirmed mavlink pattern instead of dronekit
import time  # for timing and sleeps
import math  # for simple comparisons

# uav simple altitude mission - arm + climb + hover + move + land
# keeps the overall style close to mission 4 but removes rover/radio pieces
# uses stabilize for liftoff, alt hold for hover, and land for a gentle touchdown

################################# config stuff i setup
# connection settings
CONNECTION_STRING = "/dev/ttyACM0"   # change to "udp:127.0.0.1:14551" for SITL if needed
BAUD_RATE = 57600                     # serial speed for hardware connections

# mission params
TARGET_ALT = 1.3          # target hover height in meters
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


# logging
LOG_FILE = "alt_hold_hover_land_log.txt"

############################ the mavlink helpers i wrote

def log_event(text):  # helper to write logs to terminal + text file
    timestamp = time.strftime("%H:%M:%S")
    line = f"[{timestamp}] {text}"
    print(line)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")


def request_message_streams(master):  # asks the flight controller for the messages we care about
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


def change_mode(master, *mode_names):  # changes the flight controller mode with a couple alias options
    mapping = master.mode_mapping()
    for mode in mode_names:
        if mode in mapping:
            mode_id = mapping[mode]
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            log_event(f"Mode set: {mode}")
            time.sleep(1)
            return mode
    raise RuntimeError(f"None of these modes were found: {mode_names}. Available: {list(mapping.keys())}")


def arm_drone(master):  # engages the motors
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
    log_event("Arming motors...")


def disarm_drone(master):  # emergency fallback if needed
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0,
    )
    log_event("Disarm command sent.")


def set_throttle(master, pwm):  # pushes throttle by rc override
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0,
    )

def set_rc_override(master, x = THROTTLE_HOVER,y = THROTTLE_HOVER,z = THROTTLE_HOVER):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        x, y, z, 0, 0, 0, 0, 0,
    )


def clear_rc_override(master):  # releases rc override back to the autopilot / radio
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0,
    )

def get_optical_flow_position_x(master): # gets the position data from the optical flow sensor, if available
    msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
    while msg:
        if msg.quality > 0:
            pos_x = msg.flow_x
            return pos_x
        msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
    return None

def get_optical_flow_position_y(master): # gets the position data from the optical flow sensor, if available
    msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
    while msg:
        if msg.quality > 0:
            pos_y = msg.flow_y
            return pos_y
        msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
    return None

def get_optical_flow_quality(master): # gets the quality reading from the optical flow sensor, if available
    msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
    while msg:
        quality = msg.quality
        return quality
    return None

def get_position_estimate(master): # combines the optical flow x and y to get an esitmate of the current position relative to the starting point, if available
    pos_x = get_optical_flow_position_x(master)
    pos_y = get_optical_flow_position_y(master)
    quality = get_optical_flow_quality(master)

    if pos_x is not None and pos_y is not None and quality is not None:
        return pos_x, pos_y, quality
    else:
        return None, None, None


def get_rangefinder_alt(master):  # tries the downward sensor first
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    while msg:
        current = msg.current_distance / 100.0
        if current > 0.01:
            return current
        msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    return None


def get_baro_relative_alt(master):  # fallback altitude from global position message
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    while msg:
        rel_alt_m = msg.relative_alt / 1000.0
        return rel_alt_m
    return None


def get_altitude_m(master):  # unified altitude helper
    rng_alt = get_rangefinder_alt(master)
    if rng_alt is not None:
        return rng_alt, "rangefinder"

    baro_alt = get_baro_relative_alt(master)
    if baro_alt is not None:
        return baro_alt, "baro"

    return None, "none"


def print_altitude(master, prefix="Altitude"):  # prints the current altitude every loop
    alt, source = get_altitude_m(master)
    if alt is None:
        print(f"{prefix}: waiting for altitude data...", end="\r", flush=True)
        return None

    print(f"{prefix}: {alt:5.2f} m  (source: {source})", end="\r", flush=True)
    return alt


def wait_for_good_altitude(master, timeout_s=5.0):  # makes sure we actually have altitude data before flying
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        alt, source = get_altitude_m(master)
        if alt is not None:
            log_event(f"Altitude source ready: {source}, current altitude {alt:.2f} m")
            return
        time.sleep(0.1)
    raise RuntimeError("No altitude data received from rangefinder or barometer.")


def climb_to_target(master, target_alt):  # manual climb like mission 4, then settle near target
    log_event(f"Climbing to {target_alt:.2f} m...")

    set_throttle(master, THROTTLE_IDLE)
    time.sleep(1.0)

    stable_start = None
    while True:
        alt = print_altitude(master, prefix="Climb Alt")

        if alt is None:
            set_throttle(master, THROTTLE_IDLE)
            time.sleep(CLIMB_LOOP_DT)
            continue

        # first push upward until close to target, then settle gently
        if alt < (target_alt - ALT_TOL):
            set_throttle(master, THROTTLE_CLIMB)
            stable_start = None
        else:
            set_throttle(master, THROTTLE_HOVER)

            # require the altitude to stay near target briefly before switching to alt hold
            if abs(alt - target_alt) <= 0.20:
                if stable_start is None:
                    stable_start = time.time()
                elif (time.time() - stable_start) >= 1.2:
                    print()  # move off the carriage-return line cleanly
                    log_event(f"Target altitude reached and stabilized: {alt:.2f} m")
                    return
            else:
                stable_start = None

        time.sleep(CLIMB_LOOP_DT)


def hover_in_alt_hold(master, hover_time_s):  # switches to alt hold and keeps throttle centered
    change_mode(master, "ALT_HOLD", "ALTHOLD")
    log_event(f"Holding altitude for {hover_time_s:.1f} seconds...")

    start_t = time.time()
    while (time.time() - start_t) < hover_time_s:
        alt = print_altitude(master, prefix="Hover Alt")

        # in alt hold, keeping throttle near mid-stick tells the autopilot to maintain altitude
        set_throttle(master, THROTTLE_HOVER)

        # tiny trim if it drifts a lot while still keeping the command near mid-stick
        if alt is not None:
            if alt < TARGET_ALT - 0.20:
                set_throttle(master, THROTTLE_HOVER + 40)
            elif alt > TARGET_ALT + 0.20:
                set_throttle(master, THROTTLE_HOVER - 40)

        time.sleep(HOVER_LOOP_DT)

    print()
    log_event("Hover segment complete.")


def move_pitch(master, forward = True, seconds = 1.5):
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
    log_event(f"Moving {direction}...")
    start_t = time.time()
    while (time.time() - start_t) < seconds:
        set_rc_override(master, pitch=move_pwm, throttle=THROTTLE_HOVER)
        time.sleep(0.1)

    # 2. Level Out
    log_event("Leveling... (Coasting)")
    set_rc_override(master, pitch=1500, throttle=THROTTLE_HOVER)
    time.sleep(1.0) # Drone is still moving forward here!

    # 3. Active Brake (Counter-Pitch)
    log_event("Applying brakes...")
    start_t = time.time()
    while (time.time() - start_t) < 0.4:
        set_rc_override(master, pitch=brake_pwm, throttle=THROTTLE_HOVER)
        time.sleep(0.1)

    # 4. Final Neutral
    set_rc_override(master, pitch=1500, throttle=THROTTLE_HOVER)
    log_event("Hovering at destination.")

def land_safely(master, timeout=10):  # lets land mode do a controlled descent and auto-disarm
    """
    Sends a command for the drone to land.

    Args:
        master (mavutil.mavlink_connection): The MAVLink connection to use.
        timeout (int): Time in seconds to wait for an acknowledgment.

    Returns:
        int: mavutil.mavlink.MAV_RESULT enum value.
    """
    clear_rc_override(master)
    time.sleep(0.1)

    # Send a command to land
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 
        0, 0, 0, 0, 0, 0, 0, 0
    )

    # Wait for the acknowledgment
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
    if ack is None:
        print('No acknowledgment received within the timeout period.')
        return None

    return ack.result

#################### the main mission logic

def main():  # the main boss function
    with open(LOG_FILE, "w") as f:
        f.write("")

    log_event("==========================================")
    log_event("   UAV MOVEMENT TEST + SAFE LAND MISSION")
    log_event("==========================================")

    log_event(f"Connecting to Drone: {CONNECTION_STRING}...")
    if CONNECTION_STRING.startswith("udp:") or CONNECTION_STRING.startswith("tcp:"):
        master = mavutil.mavlink_connection(CONNECTION_STRING)
    else:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)

    master.wait_heartbeat()
    log_event("Drone Heartbeat OK.")

    request_message_streams(master)

    try:
        wait_for_good_altitude(master)

        # step 1: start in stabilize like your mission 4 pattern
        change_mode(master, "STABILIZE")
        arm_drone(master)

        # step 2: climb to the requested height
        climb_to_target(master, TARGET_ALT)

        # step 3: maintain altitude using alt hold
        hover_in_alt_hold(master, 5.0)

        #step 4: test if position values are coming through and print them
        log_event("Testing optical flow position readings for 10 seconds...")
        start_t = time.time()
        while (time.time() - start_t) < 10.0:
            pos_x, pos_y, quality = get_position_estimate(master)
            if pos_x is not None and pos_y is not None:
                print(f"Optical Flow Position - X: {pos_x:.2f}, Y: {pos_y:.2f}, Quality: {quality}", end="\r", flush=True)
            else:
                print("Waiting for optical flow data...")
            time.sleep(0.1)

        '''
        #step 4: move forward a few feet
        move_pitch(master, True, 1.0)

        #step 5: move backward a few feet
        move_pitch(master, False, 1.0)
        '''
        # step 5: safely and slowly land
        land_safely(master)

    except KeyboardInterrupt:
        print()
        log_event("[!] Keyboard interrupt received. Switching to LAND now...")
        try:
            land_safely(master)
        except Exception as land_err:
            log_event(f"Landing fallback error: {land_err}")
            try:
                clear_rc_override(master)
                disarm_drone(master)
            except Exception:
                pass

    except Exception as e:
        print()
        log_event(f"Mission error: {e}")
        try:
            change_mode(master, "LAND")
            land_safely(master)
        except Exception:
            pass
        time.sleep(1)

    finally:
        print()
        clear_rc_override(master)
        log_event("Mission finalized.")


if __name__ == "__main__":
    main()

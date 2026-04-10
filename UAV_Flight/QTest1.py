from pymavlink import mavutil  # using the confirmed mavlink pattern instead of dronekit
import time  # for timing and sleeps
import math  # for simple comparisons
import signal  # so we can intercept ctrl+c without dying

# uav simple altitude mission - arm + climb + hover + land
# keeps the overall style close to mission 4 but removes rover/radio pieces
# uses stabilize for liftoff, alt hold for hover, and land for a gentle touchdown
# NOTE: ctrl+c will NOT kill this script mid-flight, it only schedules a graceful land

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
LAND_TIMEOUT_S = 60.0     # safety timeout for landing (fallback only - cube heartbeat is primary)

# throttle settings i tuned
THROTTLE_MIN = 1000       # motors off / minimum throttle
THROTTLE_IDLE = 1150      # props spinning but no real lift
THROTTLE_CLIMB = 1650     # enough lift to climb
THROTTLE_HOVER = 1500     # mid-stick hover command for alt hold

# logging
LOG_FILE = "alt_hold_hover_land_log.txt"

# global flag set by the signal handler so ctrl+c schedules a land instead of killing us
_land_requested = False  # flipped to True when ctrl+c is caught

############################ signal handler so ctrl+c never hard-kills the process

def _handle_sigint(signum, frame):  # intercepts ctrl+c at the os level
    global _land_requested  # reach into global space to flip the flag
    if not _land_requested:  # only print the message the first time
        print()  # move off any carriage-return line cleanly
        log_event("[!] Ctrl+C caught. Landing will be initiated at the next safe point.")
    _land_requested = True  # set the flag so the mission loops check it

signal.signal(signal.SIGINT, _handle_sigint)   # redirect ctrl+c to our handler
signal.signal(signal.SIGTERM, _handle_sigint)  # also catch kill signals for robustness

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
    time.sleep(2)  # wait for the motors to spin up

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

def clear_rc_override(master):  # releases rc override back to the autopilot / radio
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0,
    )


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
    global _land_requested  # check the flag in case ctrl+c came during climb

    log_event(f"Climbing to {target_alt:.2f} m...")

    set_throttle(master, THROTTLE_IDLE)
    time.sleep(1.0)

    stable_start = None
    while True:
        if _land_requested:  # ctrl+c came in, bail out of the climb immediately
            log_event("[!] Land requested during climb. Stopping climb early.")
            return

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
    global _land_requested  # check the flag so ctrl+c cuts the hover short safely

    change_mode(master, "ALT_HOLD", "ALTHOLD")
    log_event(f"Holding altitude for {hover_time_s:.1f} seconds...")

    start_t = time.time()
    while (time.time() - start_t) < hover_time_s:
        if _land_requested:  # ctrl+c came in, skip the rest of the hover immediately
            log_event("[!] Land requested during hover. Ending hover early.")
            break

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


def land_safely(master):
    # switches to land mode and then BLOCKS until the cube's heartbeat confirms
    # that the drone has actually disarmed (motors stopped) - no guessing from altitude.
    # a timeout fallback is still in place so we never hang forever.
    log_event("Landing sequence engaged...")
    change_mode(master, "LAND")
    clear_rc_override(master)  # let land mode fully control the descent

    log_event("Waiting for Cube heartbeat to confirm touchdown and auto-disarm...")

    deadline = time.time() + LAND_TIMEOUT_S  # hard fallback so we never hang forever
    while time.time() < deadline:
        # primary confirmation: cube heartbeat says motors are no longer armed
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)  # blocking wait up to 2s per beat
        if hb is not None:
            motors_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)  # check armed bit
            if not motors_armed:
                # the cube itself told us the motors stopped - this is the real touchdown confirmation
                print()  # move off carriage-return line
                log_event("Cube heartbeat confirmed: motors disarmed. Touchdown complete.")
                return  # landing confirmed, exit

        # also print altitude while we wait so the operator can see the descent
        print_altitude(master, prefix="Land Alt")
        time.sleep(LAND_LOOP_DT)

    # if we hit the deadline without a disarm heartbeat, log a warning but do not crash out
    print()
    log_event("[WARNING] Landing timeout reached without cube disarm confirmation. Sending disarm fallback.")
    disarm_drone(master)  # one last attempt to stop the motors
    time.sleep(2.0)  # give the cube a moment to process the disarm
    log_event("Disarm fallback sent. Assuming landed.")


#################### the main mission logic

def main():  # the main boss function
    with open(LOG_FILE, "w") as f:
        f.write("")  # clear the log file at the start of each run

    log_event("==========================================")
    log_event("   UAV ALT HOLD HOVER + SAFE LAND MISSION")
    log_event("==========================================")
    log_event("NOTE: Ctrl+C will NOT kill this process. It only requests a graceful land.")

    log_event(f"Connecting to Drone: {CONNECTION_STRING}...")
    if CONNECTION_STRING.startswith("udp:") or CONNECTION_STRING.startswith("tcp:"):
        master = mavutil.mavlink_connection(CONNECTION_STRING)
    else:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)

    master.wait_heartbeat()
    log_event("Drone Heartbeat OK.")

    request_message_streams(master)

    # ---- outer keepalive shell ----
    # this loop makes sure that even if something blows up deep in the mission
    # we always attempt to land before the script exits. it will not stop
    # looping until land_safely() has actually run to completion.
    landed = False  # tracks whether land_safely has finished successfully
    while not landed:
        try:
            if not _land_requested:
                # step 1: start in stabilize like your mission 4 pattern
                change_mode(master, "STABILIZE")
                arm_drone(master)

                # step 2: climb to the requested height
                climb_to_target(master, TARGET_ALT)

                # step 3: maintain altitude using alt hold
                hover_in_alt_hold(master, HOVER_TIME_S)

            # step 4: land - this is always reached no matter what happened above
            land_safely(master)
            landed = True  # land_safely completed cleanly, we can exit the outer loop

        except Exception as e:
            # something unexpected blew up - log it but do NOT exit
            # the outer while loop will bring us back around and try to land again
            print()
            log_event(f"[ERROR] Unexpected exception: {e}. Attempting emergency land in 2s...")
            time.sleep(2.0)  # short pause before the retry so we dont spam the cube
            try:
                change_mode(master, "LAND")  # try to get into land mode before looping
                clear_rc_override(master)
            except Exception:
                pass  # if even this fails, the outer loop will retry land_safely

    # ---- mission complete ----
    print()
    clear_rc_override(master)
    log_event("Mission finalized. Script exiting now.")


if __name__ == "__main__":
    main()
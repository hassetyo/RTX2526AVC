from pymavlink import mavutil
import time
import math

# connection args
CONNECTION_STRING = "udp:127.0.0.1:14551"

ALTITUDE_M = 10

LEG_DISTANCE_M = 10
STEP_DISTANCE_M = 3
SPEED_MS = 1.0

POS_TOL_M = 0.25
MOVE_TIMEOUT_S = 90

YAW_TOL_DEG = 2.0
TURN_TIMEOUT_S = 20
YAW_STABLE_TIME_S = 0.5

print(f"Connecting to {CONNECTION_STRING}")
master = mavutil.mavlink_connection(CONNECTION_STRING)
master.wait_heartbeat()
print("Heartbeat found")


# local position try
def request_message_streams():
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  
            1
        )
    except Exception:
        pass

    def set_interval(msg_id, hz):
        us = int(1e6 / hz)
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            us,
            0, 0, 0, 0, 0
        )

    try:
        set_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
        set_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)
    except Exception:
        pass


request_message_streams()


def change_mode(mode: str):
    # mode changer
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise RuntimeError(f"Unknown mode '{mode}'. Available: {list(mapping.keys())}")
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode: {mode}")
    time.sleep(1)


def arm_drone():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming...")
    time.sleep(2)


def takeoff(alt: float):
    print(f"Taking off to {alt}m")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        alt
    )
    time.sleep(10)


def get_local_xy(timeout=0.5):
    # need this to track where we are. If optical flow is off, this returns None
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if not msg:
        return None
    return float(msg.x), float(msg.y)


def get_yaw_rad(timeout=0.5):
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)
    if not msg:
        return None
    return float(msg.yaw)


def wrap_pi(a):
    # maintain angles within -pi to pi
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def send_velocity_local_ned(vx, vy, vz=0.0):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )


def send_stop(duration_s=1.0):
    # brake
    end = time.time() + duration_s
    while time.time() < end:
        send_velocity_local_ned(0.0, 0.0, 0.0)
        time.sleep(0.1)


def move_to_local_xy(xt, yt, speed_ms=SPEED_MS, pos_tol=POS_TOL_M, timeout_s=MOVE_TIMEOUT_S):
    # loop to get target coords
    t_end = time.time() + timeout_s

    while time.time() < t_end:
        pos = get_local_xy(timeout=0.2)
        if pos is None:
            continue

        x, y = pos
        ex, ey = (xt - x), (yt - y)
        dist = math.hypot(ex, ey)

        if dist <= pos_tol:
            break

        # normalize vector and scale
        vx = (ex / dist) * speed_ms
        vy = (ey / dist) * speed_ms

        send_velocity_local_ned(vx, vy, 0.0)
        time.sleep(0.05)

    send_stop(1.0)


def move_by_offset_local(dx, dy, speed_ms=SPEED_MS, pos_tol=POS_TOL_M, timeout_s=MOVE_TIMEOUT_S):
    start = get_local_xy(timeout=1.0)
    if start is None:
        raise RuntimeError("Lost local position. Check Optical Flow.")

    x0, y0 = start
    xt, yt = x0 + dx, y0 + dy
    print(f"Moving offset: {dx}, {dy} -> Target: {xt:.2f}, {yt:.2f}")
    move_to_local_xy(xt, yt, speed_ms=speed_ms, pos_tol=pos_tol, timeout_s=timeout_s)


def turn_relative_stable(deg, tol_deg=YAW_TOL_DEG, stable_time_s=YAW_STABLE_TIME_S, timeout_s=TURN_TIMEOUT_S):
    yaw0 = get_yaw_rad(timeout=1.0)
    if yaw0 is None:
        raise RuntimeError("Lost Attitude data")

    target = wrap_pi(yaw0 + math.radians(deg))
    tol = math.radians(tol_deg)

    print(f"Turning {deg} degrees")

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        abs(deg),
        0,
        1 if deg > 0 else -1,
        1,
        0, 0, 0
    )

    # waits til stbale
    t_end = time.time() + timeout_s
    stable_start = None

    while time.time() < t_end:
        yaw = get_yaw_rad(timeout=0.5)
        if yaw is None:
            continue

        err = abs(wrap_pi(target - yaw))
        if err <= tol:
            if stable_start is None:
                stable_start = time.time()
            elif (time.time() - stable_start) >= stable_time_s:
                return
        else:
            stable_start = None

        time.sleep(0.05)


# actual mission
if __name__ == "__main__":
    change_mode("GUIDED")
    arm_drone()
    takeoff(ALTITUDE_M)

    # home position
    home = get_local_xy(timeout=2.0)
    if home is None:
        raise RuntimeError("No position data. Optical Flow is unresposive.")
    home_x, home_y = home
    print(f"Home set at: {home_x:.2f}, {home_y:.2f}")

    print("Starting Snake Pattern")

    FACE_LEGS = True

    if FACE_LEGS:
        turn_relative_stable(0)
    move_by_offset_local(+LEG_DISTANCE_M, 0)

    if FACE_LEGS:
        turn_relative_stable(90)
    move_by_offset_local(0, +STEP_DISTANCE_M)

    if FACE_LEGS:
        turn_relative_stable(90)
    move_by_offset_local(-LEG_DISTANCE_M, 0)

    if FACE_LEGS:
        turn_relative_stable(-90)
    move_by_offset_local(0, +STEP_DISTANCE_M)

    if FACE_LEGS:
        turn_relative_stable(-90)
    move_by_offset_local(+LEG_DISTANCE_M, 0)

    print("Returning Home")
    move_to_local_xy(home_x, home_y)

    print("Landing.")
    change_mode("LAND")
    print("Mission Complete")

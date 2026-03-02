import socket
import json
import time
import math
from pymavlink import mavutil

# ----------------------------
# Rover SITL connection
# ----------------------------
UGV_CONN = "udp:127.0.0.1:14550"   # <-- CHANGE to your Rover port if different

# ----------------------------
# TCP server (local)
# ----------------------------
TCP_IP = "127.0.0.1"
TCP_PORT = 5005

# ----------------------------
# Control tuning
# ----------------------------
POS_TOL = 0.6          # meters
SLOW_RADIUS = 3.0      # start slowing within this many meters
MAX_VX = 1.0           # m/s
MIN_VX = 0.15          # m/s (only when moving forward)
MAX_YAWRATE = 1.0      # rad/s
K_YAW = 1.8            # yaw-rate gain
TURN_IN_PLACE_THRESH = 1.0  # rad (~57 degrees)

def now_boot_ms():
    # MAVLink expects uint32 time_boot_ms
    return int(time.monotonic() * 1000) & 0xFFFFFFFF

def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def change_mode(master, mode: str):
    mapping = master.mode_mapping()
    if not mapping or mode not in mapping:
        raise RuntimeError(f"Mode '{mode}' not available.")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode]
    )
    time.sleep(1.0)

def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)

def get_xy(master):
    msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)
    if not msg:
        return None
    return float(msg.x), float(msg.y)

def get_yaw(master):
    msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)
    if not msg:
        return None
    return float(msg.yaw)

def send_vel(master, vx, yaw_rate):
    # Velocity control in BODY_NED + yaw_rate
    master.mav.set_position_target_local_ned_send(
        now_boot_ms(),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,   # use vx/vy/vz + yaw_rate
        0, 0, 0,
        vx, 0, 0,
        0, 0, 0,
        0, yaw_rate
    )

def stop_and_hold(master, hold_s=2.0):
    # Keep sending 0-velocity so Rover actually settles
    t_end = time.time() + hold_s
    while time.time() < t_end:
        send_vel(master, 0.0, 0.0)
        time.sleep(0.1)

def drive_to_target(master, tx, ty):
    print(f"[UGV] Driving to target ({tx:.3f},{ty:.3f})")

    while True:
        pos = get_xy(master)
        yaw = get_yaw(master)
        if pos is None or yaw is None:
            continue

        x, y = pos
        dx, dy = (tx - x), (ty - y)
        dist = math.hypot(dx, dy)

        # Stop condition
        if dist < POS_TOL:
            stop_and_hold(master, 2.5)
            print("[UGV] Target reached and stopped.")
            return

        desired = math.atan2(dy, dx)
        yaw_err = wrap_pi(desired - yaw)

        # If we're facing too far away from the target, rotate in place first
        if abs(yaw_err) > TURN_IN_PLACE_THRESH:
            vx = 0.0
        else:
            # Slow down as we approach
            if dist < SLOW_RADIUS:
                vx = max(MIN_VX, MAX_VX * (dist / SLOW_RADIUS))
            else:
                vx = MAX_VX

        yaw_rate = max(-MAX_YAWRATE, min(MAX_YAWRATE, K_YAW * yaw_err))

        send_vel(master, vx, yaw_rate)
        print(f"[UGV] pos=({x:.2f},{y:.2f}) dist={dist:.2f} vx={vx:.2f} yaw_err={yaw_err:.2f}")
        time.sleep(0.1)

def main():
    print("[UGV] Connecting to Rover SITL...")
    master = mavutil.mavlink_connection(UGV_CONN)
    master.wait_heartbeat()
    print("[UGV] Heartbeat OK")

    change_mode(master, "GUIDED")
    arm(master)

    # TCP server waiting for UAV packet
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((TCP_IP, TCP_PORT))
    srv.listen(1)

    print("[UGV] Waiting for UAV target...")
    conn, addr = srv.accept()
    print(f"[UGV] Connection from {addr}")

    with conn:
        data = conn.recv(4096).decode("utf-8", errors="ignore").strip()
        pkt = json.loads(data)
        tx = float(pkt["x"])
        ty = float(pkt["y"])
        print(f"[UGV] Received target ({tx:.3f},{ty:.3f})")

    drive_to_target(master, tx, ty)

if __name__ == "__main__":
    main()
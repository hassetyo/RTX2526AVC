import socket, json, time, math
from pymavlink import mavutil

# ----------------------------
# Rover SITL
# ----------------------------
UGV_CONN = "udp:127.0.0.1:14560"

# ----------------------------
# TCP
# ----------------------------
TCP_IP = "127.0.0.1"
TCP_PORT = 5005

POS_TOL = 0.6

def change_mode(master, mode):
    mapping = master.mode_mapping()
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode]
    )
    time.sleep(1)

def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1)

def get_xy(master):
    msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)
    if not msg:
        return None
    return msg.x, msg.y

def get_yaw(master):
    msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)
    if not msg:
        return None
    return msg.yaw

def send_vel(master, vx, yaw_rate):
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, 0, 0,
        0, 0, 0,
        0, yaw_rate
    )

print("[UGV] Connecting to Rover SITL...")
master = mavutil.mavlink_connection(UGV_CONN)
master.wait_heartbeat()
print("[UGV] Heartbeat OK")

change_mode(master, "GUIDED")
arm(master)

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.bind((TCP_IP, TCP_PORT))
srv.listen(1)
print("[UGV] Waiting for UAV target...")

conn, _ = srv.accept()
data = conn.recv(1024).decode()
pkt = json.loads(data)

tx, ty = pkt["x"], pkt["y"]
print(f"[UGV] Received target ({tx:.1f},{ty:.1f})")

while True:
    pos = get_xy(master)
    yaw = get_yaw(master)
    if not pos or yaw is None:
        continue

    x, y = pos
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)

    if dist < POS_TOL:
        send_vel(master, 0, 0)
        print("[UGV] Target reached")
        break

    desired = math.atan2(dy, dx)
    yaw_err = desired - yaw

    vx = min(1.0, 0.6 * dist)
    yaw_rate = max(-1.0, min(1.0, 1.5 * yaw_err))

    send_vel(master, vx, yaw_rate)
    print(f"[UGV] pos=({x:.1f},{y:.1f}) dist={dist:.2f}")
    time.sleep(0.1)
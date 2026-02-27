import time, math, json, socket
from pymavlink import mavutil

# ----------------------------
# Copter SITL
# ----------------------------
UAV_CONN = "udp:127.0.0.1:14550"

# ----------------------------
# TCP (local)
# ----------------------------
TCP_IP = "127.0.0.1"
TCP_PORT = 5005

TARGET_X = 15.0   # meters north
TARGET_Y = 8.0    # meters east
TARGET_Z = -5.0   # meters (up)

POS_TOL = 0.5

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

def get_local(master):
    msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.5)
    if not msg:
        return None
    return msg.x, msg.y, msg.z

def send_setpoint(master, x, y, z):
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def send_tcp(x, y):
    pkt = {"x": x, "y": y}
    with socket.create_connection((TCP_IP, TCP_PORT)) as s:
        s.sendall((json.dumps(pkt) + "\n").encode())
    print(f"[UAV] Sent target {pkt}")

print("[UAV] Connecting to Copter SITL...")
master = mavutil.mavlink_connection(UAV_CONN)
master.wait_heartbeat()
print("[UAV] Heartbeat OK")

change_mode(master, "GUIDED")
arm(master)

print("[UAV] Flying to target...")
while True:
    send_setpoint(master, TARGET_X, TARGET_Y, TARGET_Z)
    pos = get_local(master)
    if pos:
        x, y, z = pos
        d = math.hypot(TARGET_X - x, TARGET_Y - y)
        print(f"[UAV] pos=({x:.1f},{y:.1f}) dist={d:.2f}")
        if d < POS_TOL:
            break
    time.sleep(0.2)

print("[UAV] Target reached")
x, y, _ = get_local(master)
send_tcp(x, y)
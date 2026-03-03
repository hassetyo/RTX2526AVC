import time, math, json, socket
from pymavlink import mavutil

UAV_CONN = "udp:127.0.0.1:14560"  # Copter port
TCP_IP = "127.0.0.1"
TCP_PORT = 5005

TAKEOFF_ALT_M = 5.0
TARGET_X = 20.0   # go farther out (meters north)
TARGET_Y = 0.0
TARGET_Z = -5.0

POS_TOL = 0.8

def now_boot_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF

def change_mode(master, mode):
    mapping = master.mode_mapping()
    if not mapping or mode not in mapping:
        raise RuntimeError(f"Mode '{mode}' not available.")
    master.mav.set_mode_send(master.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            mapping[mode])
    time.sleep(1)

def arm(master):
    master.mav.command_long_send(master.target_system, master.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(1)

def takeoff(master, alt_m):
    master.mav.command_long_send(master.target_system, master.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                0, 0, 0, 0, 0, 0, 0, alt_m)
    time.sleep(1)

def get_local(master):
    msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
    if not msg:
        return None
    return float(msg.x), float(msg.y), float(msg.z)

def send_local_position_setpoint(master, x, y, z):
    # position control in LOCAL_NED
    master.mav.set_position_target_local_ned_send(
        now_boot_ms(),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,   # position enabled, vel/accel ignored
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def wait_until_alt(master, alt_m, timeout=30):
    target_z = -abs(alt_m)
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos = get_local(master)
        if pos:
            _, _, z = pos
            print(f"[UAV] alt(z)={z:.2f} target={target_z:.2f}")
            if z <= target_z + 0.5:
                return True
        time.sleep(0.2)
    return False

def send_tcp(x, y):
    pkt = {"x": x, "y": y}
    data = (json.dumps(pkt) + "\n").encode()
    for attempt in range(1, 11):
        try:
            with socket.create_connection((TCP_IP, TCP_PORT), timeout=2.0) as s:
                s.sendall(data)
            print(f"[UAV] Sent target {pkt}")
            return True
        except Exception as e:
            print(f"[UAV] TCP send attempt {attempt}/10 failed: {e}")
            time.sleep(0.5)
    return False

print("[UAV] Connecting to Copter SITL...")
master = mavutil.mavlink_connection(UAV_CONN)
master.wait_heartbeat()
print("[UAV] Heartbeat OK")

change_mode(master, "GUIDED")
arm(master)

print(f"[UAV] Takeoff to {TAKEOFF_ALT_M} m...")
takeoff(master, TAKEOFF_ALT_M)

if not wait_until_alt(master, TAKEOFF_ALT_M):
    raise RuntimeError("Altitude not reached.")

print(f"[UAV] Going to LOCAL target x={TARGET_X}, y={TARGET_Y} ...")
while True:
    send_local_position_setpoint(master, TARGET_X, TARGET_Y, TARGET_Z)
    pos = get_local(master)
    if pos:
        x, y, z = pos
        dist = math.hypot(TARGET_X - x, TARGET_Y - y)
        print(f"[UAV] pos=({x:.2f},{y:.2f},{z:.2f}) dist={dist:.2f}")
        if dist < POS_TOL:
            break
    time.sleep(0.2)

# hold a moment
for _ in range(20):
    send_local_position_setpoint(master, TARGET_X, TARGET_Y, TARGET_Z)
    time.sleep(0.1)

pos = get_local(master)
x, y, z = pos
print(f"[UAV] Final pos: x={x:.2f}, y={y:.2f}, z={z:.2f} (NED)")

ok = send_tcp(x, y)
print("[UAV] Done." if ok else "[UAV] Done, but TCP send failed.")
from pymavlink import mavutil  # talk to the flight controller
import time                    # timing
import math                    # (kept, minimal changes)

# -----------------------------
# config
# -----------------------------
CONNECTION_STRING = "127.0.0.1:14560"  # "/dev/ttyACM0"
BAUD_RATE = 115200

DIST_M = 3.048        # move ~10 ft
SPEED_MPS = 0.2       # speed magnitude
TAKEOFF_ALT_M = 3.0   # hover height
MAX_ALT_ALLOWED = 3.5 # safety ceiling
RETURN_SPEED_MPS = 0.15   # slow return speed

print(f"[Mission] Connecting to {CONNECTION_STRING}...")
master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
master.wait_heartbeat()
print("[Mission] Heartbeat found.")

# -----------------------------
# mavlink helpers
# -----------------------------
def change_mode(mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"Unknown mode: {mode}")
        return
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set: {mode}")

def arm_drone():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming...")

def takeoff(alt):
    print(f"Takeoff command: {alt}m")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt
    )
    # Reinforce the climb with an explicit position target (NED: z is negative = up)
    time.sleep(0.5)
    time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,  # position only
        0, 0, -alt,          # z = -alt because NED up is negative
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def get_altitude():
    # RELATIVE altitude above home (meters)
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
    if msg:
        return float(msg.relative_alt) / 1000.0  # mm -> meters
    return 0.0

def send_velocity(vx, vy, vz=0.0):
    time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
    master.mav.set_position_target_local_ned_send(
        time_boot_ms,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # use velocity (vx,vy,vz); yaw_rate=0 below
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )

# -----------------------------
# main mission
# -----------------------------
def main():
    try:
        # step 1: takeoff
        change_mode("GUIDED")
        time.sleep(1)
        arm_drone()
        time.sleep(1)
        takeoff(TAKEOFF_ALT_M)

        print("Climbing...")
        while True:
            alt = get_altitude()
            if alt >= TAKEOFF_ALT_M * 0.9:
                print(f"Altitude reached: {alt:.2f}m")
                break
            time.sleep(0.5)

        # record start position in LOCAL NED
        start_pos = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2.0)
        if not start_pos:
            raise RuntimeError("Could not read LOCAL_POSITION_NED at start.")
        start_x, start_y, start_z = float(start_pos.x), float(start_pos.y), float(start_pos.z)

        # step 2: move BACKWARD (no turning)
        print(f"[Mission] Flying {DIST_M}m backward...")
        duration = DIST_M / SPEED_MPS
        start_t = time.time()

        while (time.time() - start_t) < duration:
            alt = get_altitude()
            if alt > MAX_ALT_ALLOWED:
                print(f"Altitude Warning: {alt:.2f}m. Forced Land.")
                raise RuntimeError("Altitude exceeded safety limit")

            # BACKWARD in BODY_NED = negative vx
            send_velocity(-SPEED_MPS, 0.0, 0.0)
            time.sleep(0.1)

        # step 3: hold briefly
        print("[Mission] Holding...")
        hold_end = time.time() + 2.0
        while time.time() < hold_end:
            send_velocity(0.0, 0.0, 0.0)
            time.sleep(0.1)

        # step 4: return to original point (LOCAL NED position hold)
        print("[Mission] Returning to start position (slow)...")

        # read current position once
        p0 = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2.0)
        if not p0:
            raise RuntimeError("Could not read LOCAL_POSITION_NED before return.")
        cur_x, cur_y, cur_z = float(p0.x), float(p0.y), float(p0.z)

        type_mask = 0b0000111111111000  # position enabled; vel/accel ignored; yaw/yaw_rate ignored
        dt = 0.2                        # command rate
        step = RETURN_SPEED_MPS * dt    # meters per step
        return_deadline = time.time() + 60.0

        while time.time() < return_deadline:
            # compute vector to start
            dx = start_x - cur_x
            dy = start_y - cur_y
            dist_back = math.hypot(dx, dy)

            print(f"[Mission] return dist: {dist_back:.2f} m")

            if dist_back < 0.5:
                print("[Mission] Back at start.")
                break

            # move commanded point a small step toward start
            if dist_back > 1e-6:
                ux = dx / dist_back
                uy = dy / dist_back
            else:
                ux, uy = 0.0, 0.0

            cur_x += ux * min(step, dist_back)
            cur_y += uy * min(step, dist_back)

            time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
            master.mav.set_position_target_local_ned_send(
                time_boot_ms,
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                cur_x, cur_y, cur_z,
                0, 0, 0,
                0, 0, 0,
                0, 0
            )

            time.sleep(dt)

        # step 5: land
        print("[Mission] Landing now...")
        change_mode("LAND")

        while True:
            alt = get_altitude()
            if alt < 0.3:
                print("Landed safely.")
                break
            time.sleep(1)

    except Exception as e:
        print(f"Mission Error: {e}. Attempting EMERGENCY LAND.")
        try:
            change_mode("LAND")
        except Exception:
            pass

if __name__ == "__main__":
    main()

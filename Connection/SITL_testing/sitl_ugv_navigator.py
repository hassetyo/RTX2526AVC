"""
sitl_ugv_navigator.py  —  PC SITL version of rpi_ugv_navigator.py
------------------------------------------------------------------
Replaces the RPi UART with a TCP socket connection to virtual_esp_bridge.py.
Replaces the FC USB cable with a UDP MAVLink connection to ArduRover SITL.

Prerequisites:
    pip install pymavlink

Start ArduRover SITL on a DIFFERENT port from ArduCopter:
    sim_vehicle.py -v Rover --console --map -L CMAC --out udp:127.0.0.1:14560

Then start virtual_esp_bridge.py, then sitl_uav_sender.py, then this script.

Usage:
    python sitl_ugv_navigator.py [--fc udp:127.0.0.1:14560] [--bridge 127.0.0.1:5556] [--speed 0.5]

SITL NOTE on ArduRover GPS-denied:
    In SITL, GPS is available by default. To test GPS-denied navigation faithfully,
    set SITL parameter GPS_TYPE=0 to disable it and rely on LOCAL_POSITION_NED only.
    The navigation logic here is identical to the hardware version.
"""

import socket
import threading
import time
import json
import math
import argparse
import sys
from pymavlink import mavutil

# ─────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────
DEFAULT_FC_STR    = "udp:127.0.0.1:14560"  # ArduRover SITL (different port from UAV)
DEFAULT_BRIDGE    = "127.0.0.1:5556"        # virtual_esp_bridge UGV port
DEFAULT_SPEED     = 0.5                     # m/s
ARRIVE_DIST_M     = 0.4                     # Arrival threshold
CMD_HZ            = 10                      # Velocity command rate
STALE_TIMEOUT_S   = 3.0                     # Stop if no fresh data
MOVE_TIMEOUT_S    = 120
CONTINUOUS_TRACK  = True

# ─────────────────────────────────────────
# SHARED STATE
# ─────────────────────────────────────────
target_lock  = threading.Lock()
target_x     = None
target_y     = None
target_stamp = 0.0

# ─────────────────────────────────────────
# TCP READER THREAD  (replaces serial reader)
# ─────────────────────────────────────────
def tcp_reader(host, port):
    global target_x, target_y, target_stamp

    def try_connect():
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((host, port))
                print(f"[TCP] Connected to bridge at {host}:{port}")
                return s
            except ConnectionRefusedError:
                print(f"[TCP] Bridge not ready — retrying in 2 s...")
                time.sleep(2)

    while True:
        sock = try_connect()
        buf = b""
        try:
            while True:
                data = sock.recv(512)
                if not data:
                    raise ConnectionResetError("Bridge closed connection")
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        payload = json.loads(line.decode('ascii'))
                        x = float(payload["x"])
                        y = float(payload["y"])
                        with target_lock:
                            target_x     = x
                            target_y     = y
                            target_stamp = time.time()
                        print(f"[RX] UAV → x={x:.3f} m N  y={y:.3f} m E")
                    except (json.JSONDecodeError, KeyError) as e:
                        print(f"[WARN] Bad packet: {line!r}  ({e})")
        except Exception as e:
            print(f"[TCP] Connection lost: {e} — reconnecting...")
        finally:
            sock.close()

# ─────────────────────────────────────────
# MAVLink HELPERS
# ─────────────────────────────────────────
def connect_fc(connection_string):
    print(f"[UGV-SITL] Connecting to ArduRover SITL: {connection_string}")
    m = mavutil.mavlink_connection(connection_string)
    m.wait_heartbeat()
    print(f"[UGV-SITL] Heartbeat OK  sys={m.target_system}  comp={m.target_component}")
    return m

def request_streams(master, hz=20):
    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            int(1e6 / hz),
            0, 0, 0, 0, 0
        )
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, hz, 1
        )
    except Exception as e:
        print(f"[UGV-SITL] Stream request warning: {e}")

def change_mode(master, mode):
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise RuntimeError(f"Unknown mode '{mode}'. Available: {list(mapping.keys())}")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode]
    )
    print(f"[UGV-SITL] Mode → {mode}")
    time.sleep(1.0)

def arm(master):
    print("[UGV-SITL] Arming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < 10:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[UGV-SITL] Armed.")
            return
    print("[WARN] Arm not confirmed — continuing.")

def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("[UGV-SITL] Disarmed.")

def get_local_xy(master, timeout=0.15):
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout)
    if msg is None:
        return None
    return float(msg.x), float(msg.y)

def send_velocity_ned(master, vx, vy, vz=0.0):
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE  |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE  |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE  |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0
    )

def stop_ugv(master, duration=1.0):
    end = time.time() + duration
    while time.time() < end:
        send_velocity_ned(master, 0.0, 0.0, 0.0)
        time.sleep(0.1)

# ─────────────────────────────────────────
# NAVIGATION LOOP
# ─────────────────────────────────────────
def navigate(master, speed_mps):
    cmd_interval = 1.0 / CMD_HZ
    t_start = time.time()
    arrived = False

    print("[NAV] Navigation loop started.")
    print(f"[NAV] Speed: {speed_mps} m/s  |  Arrive radius: {ARRIVE_DIST_M} m  |  Stale timeout: {STALE_TIMEOUT_S} s\n")

    while True:
        loop_start = time.time()

        # ── Check target freshness ──────────────────────────────────
        with target_lock:
            tx = target_x
            ty = target_y
            age = time.time() - target_stamp if target_stamp > 0 else 999.0

        if tx is None:
            print("[NAV] Waiting for first UAV position...")
            time.sleep(0.5)
            continue

        if age > STALE_TIMEOUT_S:
            if not arrived:
                print(f"[NAV] Data stale ({age:.1f}s) — stopped.")
            stop_ugv(master, 0.3)
            arrived = False
            time.sleep(0.2)
            continue

        # ── Get UGV position ────────────────────────────────────────
        pos = get_local_xy(master, timeout=0.15)
        if pos is None:
            print("[WARN] No LOCAL_POSITION_NED — retrying...")
            time.sleep(0.2)
            continue
        ux, uy = pos

        # ── Compute distance to target ──────────────────────────────
        ex, ey = tx - ux, ty - uy
        dist = math.hypot(ex, ey)

        status = "TRACKING" if not arrived else "AT_TARGET"
        print(f"[NAV] [{status}] UGV=({ux:6.2f},{uy:6.2f})  UAV=({tx:6.2f},{ty:6.2f})  dist={dist:5.2f}m  age={age:.1f}s")

        # ── Arrival ─────────────────────────────────────────────────
        if dist <= ARRIVE_DIST_M:
            if not arrived:
                print(f"\n[NAV] ✓ ARRIVED — within {ARRIVE_DIST_M} m of UAV position\n")
                stop_ugv(master, 1.0)
                arrived = True
            if not CONTINUOUS_TRACK:
                break
            send_velocity_ned(master, 0.0, 0.0, 0.0)

        else:
            arrived = False
            # Proportional guidance toward target
            vx = (ex / dist) * speed_mps
            vy = (ey / dist) * speed_mps
            send_velocity_ned(master, vx, vy, 0.0)

        # ── Hard timeout ────────────────────────────────────────────
        if (time.time() - t_start) > MOVE_TIMEOUT_S:
            print("[NAV] Mission timeout — stopping.")
            stop_ugv(master, 2.0)
            break

        # ── Pace ────────────────────────────────────────────────────
        elapsed = time.time() - loop_start
        sleep_t = cmd_interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

# ─────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────
def main(args):
    # Parse bridge address
    host, port_str = args.bridge.rsplit(':', 1)
    port = int(port_str)

    # Start TCP reader thread
    reader = threading.Thread(
        target=tcp_reader, args=(host, port), daemon=True, name="tcp_reader"
    )
    reader.start()

    # Connect to ArduRover SITL
    master = connect_fc(args.fc)
    request_streams(master)

    try:
        change_mode(master, "GUIDED")
        arm(master)

        print("\n[INFO] UGV armed and in GUIDED mode.")
        print("[INFO] Waiting for UAV position data before moving...\n")

        navigate(master, args.speed)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
        stop_ugv(master, 2.0)
    except Exception as e:
        print(f"[ERROR] {e}")
        try:
            stop_ugv(master, 2.0)
        except Exception:
            pass
    finally:
        try:
            disarm(master)
        except Exception as e:
            print(f"[WARN] Disarm: {e}")
        print("[INFO] Navigator exited.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SITL UGV navigator")
    parser.add_argument("--fc",     default=DEFAULT_FC_STR, help="MAVLink connection string for ArduRover SITL")
    parser.add_argument("--bridge", default=DEFAULT_BRIDGE, help="virtual_esp_bridge UGV port (host:port)")
    parser.add_argument("--speed",  default=DEFAULT_SPEED,  type=float, help="UGV travel speed m/s")
    args = parser.parse_args()
    main(args)

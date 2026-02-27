"""
sitl_uav_sender.py  —  PC SITL version of jetson_uav_sender.py
---------------------------------------------------------------
Replaces the Jetson UART with a TCP socket connection to virtual_esp_bridge.py.
Replaces the FC USB cable with a UDP MAVLink connection to ArduCopter SITL.

Prerequisites:
    pip install pymavlink

Start ArduCopter SITL first:
    sim_vehicle.py -v ArduCopter --console --map -L CMAC
    (or any location — we only need LOCAL_POSITION_NED which works immediately)

Then start virtual_esp_bridge.py, then run this script.

Usage:
    python sitl_uav_sender.py [--fc udp:127.0.0.1:14550] [--bridge 127.0.0.1:5555] [--hz 2]
"""

import socket
import time
import json
import argparse
from pymavlink import mavutil

# ─────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────
DEFAULT_FC_STR    = "udp:127.0.0.1:14550"   # ArduCopter SITL default MAVLink port
DEFAULT_BRIDGE    = "127.0.0.1:5555"         # virtual_esp_bridge UAV port
DEFAULT_SEND_HZ   = 2                         # Must match server.cpp pacing (500 ms)
DEFAULT_STREAM_HZ = 20                        # LOCAL_POSITION_NED request rate

# ─────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────
def connect_fc(connection_string):
    print(f"[UAV-SITL] Connecting to FC/SITL: {connection_string}")
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print(f"[UAV-SITL] Heartbeat OK  sys={master.target_system}  comp={master.target_component}")
    return master

def request_streams(master, hz):
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
        print(f"[UAV-SITL] Requested LOCAL_POSITION_NED at {hz} Hz")
    except Exception as e:
        print(f"[UAV-SITL] Stream request warning: {e}")

def get_local_position(master, timeout=0.1):
    latest = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            latest = msg
        else:
            time.sleep(0.005)
    return latest

def connect_bridge(host, port, retries=30):
    """Connect to virtual_esp_bridge with retries."""
    for i in range(retries):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((host, port))
            print(f"[UAV-SITL] Connected to bridge at {host}:{port}")
            return s
        except ConnectionRefusedError:
            print(f"[UAV-SITL] Bridge not ready, retry {i+1}/{retries}...")
            time.sleep(1)
    raise RuntimeError(f"Cannot connect to virtual_esp_bridge at {host}:{port}")

# ─────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────
def main(args):
    # Connect to SITL flight controller
    master = connect_fc(args.fc)
    request_streams(master, args.stream_hz)

    # Parse bridge address
    host, port = args.bridge.rsplit(':', 1)
    port = int(port)

    # Connect to virtual bridge
    sock = connect_bridge(host, port)

    interval = 1.0 / args.hz
    last_send = 0.0
    sent_count = 0

    print(f"\n[UAV-SITL] Streaming at {args.hz} Hz. Press Ctrl-C to stop.\n")

    try:
        while True:
            now = time.time()
            if now - last_send < interval:
                time.sleep(0.01)
                continue

            lp = get_local_position(master, timeout=0.08)
            if lp is None:
                print("[UAV-SITL] Waiting for LOCAL_POSITION_NED from SITL...")
                time.sleep(0.5)
                continue

            packet = {
                "x":  round(float(lp.x),  3),
                "y":  round(float(lp.y),  3),
                "z":  round(float(lp.z),  3),
                "vx": round(float(lp.vx), 3),
                "vy": round(float(lp.vy), 3),
                "t":  lp.time_boot_ms
            }

            line = json.dumps(packet, separators=(',', ':')) + '\n'
            try:
                sock.sendall(line.encode('ascii'))
                sent_count += 1
                last_send = now
                print(f"[TX #{sent_count:04d}] {line.strip()}")
            except (BrokenPipeError, ConnectionResetError):
                print("[UAV-SITL] Bridge connection lost — reconnecting...")
                sock.close()
                sock = connect_bridge(host, port)

    except KeyboardInterrupt:
        print("\n[UAV-SITL] Stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SITL UAV position sender")
    parser.add_argument("--fc",        default=DEFAULT_FC_STR,  help="MAVLink connection string for ArduCopter SITL")
    parser.add_argument("--bridge",    default=DEFAULT_BRIDGE,  help="virtual_esp_bridge UAV port (host:port)")
    parser.add_argument("--hz",        default=DEFAULT_SEND_HZ,    type=float, help="Transmit rate Hz")
    parser.add_argument("--stream-hz", default=DEFAULT_STREAM_HZ,  type=float, help="LOCAL_POSITION_NED stream rate Hz")
    args = parser.parse_args()
    main(args)

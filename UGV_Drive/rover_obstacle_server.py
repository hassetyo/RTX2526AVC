"""
rover_obstacle_server.py

- MAVLink rover control (GUIDED) + TCP server for obstacle events.
- When obstacle received:
    1) ACK immediately
    2) Turn right 90 degrees
    3) Go forward 2 feet
    4) Turn left 90 degrees
    5) DONE
- Between obstacles: keeps moving forward.

Run:
  python rover_obstacle_server.py --connect 127.0.0.1:14551 --port 5005
"""

from pymavlink import mavutil
import time
import math
import socket
import argparse

# -----------------------------
# MAVLink Helpers (based on your working script)
# -----------------------------

def change_mode(master, mode: str):
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise RuntimeError(f"Unknown mode '{mode}'. Available: {list(mapping.keys())[:10]} ...")
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"[ROVER] Mode set to: {mode}")
    time.sleep(1.0)

def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("[ROVER] Arming sent...")
    time.sleep(2)

def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("[ROVER] Disarmed.")

def send_velocity(master, vx, yaw_rate, duration):
    """
    vx: forward velocity (m/s)
    yaw_rate: yaw rate (rad/s)
    duration: seconds
    """
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )

    steps = max(1, int(duration / 0.1))
    for _ in range(steps):
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,
            vx, 0, 0,
            0, 0, 0,
            0,
            yaw_rate
        )
        time.sleep(0.1)

def turn_in_place(master, yaw_rate_rad_s, angle_deg):
    """
    Turns using yaw_rate for the duration needed to reach angle_deg (approx).
    angle_deg: positive for left, negative for right
    """
    angle_rad = math.radians(abs(angle_deg))
    dur = angle_rad / max(1e-6, abs(yaw_rate_rad_s))
    # For a rover, keeping vx small but nonzero can help turning; adjust if needed.
    vx = 0.4
    signed_yaw = yaw_rate_rad_s if angle_deg > 0 else -abs(yaw_rate_rad_s)
    print(f"[ROVER] Turning {angle_deg:+.0f} deg (dur ~ {dur:.2f}s, yaw_rate {signed_yaw:+.2f} rad/s)")
    send_velocity(master, vx=vx, yaw_rate=signed_yaw, duration=dur)

def go_forward_distance(master, vx, distance_m):
    dur = distance_m / max(1e-6, abs(vx))
    print(f"[ROVER] Forward {distance_m:.3f} m (dur ~ {dur:.2f}s @ {vx:.2f} m/s)")
    send_velocity(master, vx=vx, yaw_rate=0.0, duration=dur)

# -----------------------------
# TCP Server Helpers
# -----------------------------

def recv_line(conn) -> str:
    data = b""
    while True:
        ch = conn.recv(1)
        if not ch:
            raise ConnectionError("Client disconnected")
        if ch == b"\n":
            return data.decode("utf-8", errors="replace").strip()
        data += ch

def send_line(conn, line: str):
    if not line.endswith("\n"):
        line += "\n"
    conn.sendall(line.encode("utf-8"))

# -----------------------------
# Main
# -----------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="127.0.0.1:14551", help="MAVLink connection string (udp:... / 127.0.0.1:14551 / COMx)")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--port", type=int, default=5005, help="TCP port to listen on for obstacle messages")

    # Maneuver tuning
    parser.add_argument("--cruise-vx", type=float, default=1.0, help="Forward cruise velocity (m/s)")
    parser.add_argument("--yaw-rate", type=float, default=0.6, help="Yaw rate magnitude (rad/s)")
    parser.add_argument("--avoid-feet", type=float, default=2.0, help="Forward distance while avoiding (feet)")
    args = parser.parse_args()

    # MAVLink connect
    print(f"[ROVER] Connecting to {args.connect} ...")
    master = mavutil.mavlink_connection(args.connect, baud=args.baud)
    master.wait_heartbeat()
    print(f"[ROVER] Heartbeat found (sys={master.target_system}, comp={master.target_component})")

    change_mode(master, "GUIDED")
    arm(master)

    # TCP server setup
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)
    print(f"[NET] Rover server listening on 0.0.0.0:{args.port}")

    conn = None
    try:
        print("[NET] Waiting for detector to connect...")
        conn, addr = srv.accept()
        print(f"[NET] Detector connected from {addr}")

        # Start cruising forward
        print("[ROVER] Cruising forward...")
        last_cruise_send = 0.0
        cruise_period = 0.2  # keep sending commands so it continues moving

        while True:
            # Keep rover moving forward while idle
            now = time.time()
            if now - last_cruise_send > cruise_period:
                send_velocity(master, vx=args.cruise_vx, yaw_rate=0.0, duration=0.1)
                last_cruise_send = now

            # Non-blocking check for obstacle message (simple trick: short timeout)
            conn.settimeout(0.01)
            try:
                line = recv_line(conn)
            except socket.timeout:
                continue

            if not line:
                continue

            if line.startswith("OBSTACLE"):
                print(f"[NET] Received: {line}")
                send_line(conn, "ACK")

                # Perform avoidance maneuver
                # 1) turn right 90
                turn_in_place(master, yaw_rate_rad_s=args.yaw_rate, angle_deg=-90)

                # 2) forward 2 feet
                distance_m = args.avoid_feet * 0.3048
                go_forward_distance(master, vx=0.8, distance_m=distance_m)

                # 3) turn left 90
                turn_in_place(master, yaw_rate_rad_s=args.yaw_rate, angle_deg=+90)

                # Back to cruise
                print("[ROVER] Avoidance done. Resuming cruise.")
                send_line(conn, "DONE")

            else:
                print(f"[NET] Unknown command: {line}")

    except KeyboardInterrupt:
        print("\n[ROVER] KeyboardInterrupt - exiting.")
    finally:
        try:
            # Stop rover briefly
            send_velocity(master, 0.0, 0.0, 0.5)
        except Exception:
            pass

        try:
            disarm(master)
        except Exception:
            pass

        if conn:
            try:
                conn.close()
            except Exception:
                pass
        try:
            srv.close()
        except Exception:
            pass

        print("[ROVER] Test complete.")

if __name__ == "__main__":
    main()
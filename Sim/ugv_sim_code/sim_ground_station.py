"""
UGV Ground Station - Webots Simulation Edition
===============================================
This is the simulation equivalent of your real `ground_station.py`.
It runs in your terminal and talks to the Webots UGV via TCP (port 5761).
It communicates with `sim_mission1.py` wirelessly via the local UDP 
V2V bridge on port 9002.

Run this script AFTER you click 'Play' in Webots!
"""

import time
import socket
import json
import sim_v2v_bridge as v2v_bridge

# ============================================================
# CONFIG
# ============================================================
WEBOTS_PORT = 5761
ESP32_PORT = "/dev/ttyUSB0"  # Ignored by sim_v2v_bridge
TELEM_SEND_HZ = 5

print("==========================================")
print("   UGV SIMULATION GROUND STATION")
print("==========================================")


class WebotsUGVLink:
    """TCP link to the Webots SimPixhawkUGV controller"""
    def __init__(self, port=5761):
        self.port = port
        self.sock = None
        self.connect()

    def connect(self):
        while True:
            try:
                if self.sock:
                    self.sock.close()
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2.0)
                self.sock.connect(('127.0.0.1', self.port))
                print(f"[UGV-Link] Connected to Webots UGV on port {self.port}")
                break
            except (ConnectionRefusedError, TimeoutError):
                print(f"[UGV-Link] Waiting for Webots UGV on port {self.port}...")
                time.sleep(1.0)

    def _send_cmd_internal(self, cmd):
        """Internal helper to send a command and handle reconnection."""
        try:
            self.sock.sendall((json.dumps(cmd) + "\n").encode('utf-8'))
        except Exception as e:
            print(f"[UGV-Link] Disconnected from Webots! ({e}). Reconnecting...")
            self.connect()
            try:
                self.sock.sendall((json.dumps(cmd) + "\n").encode('utf-8'))
            except Exception as re_e:
                print(f"[UGV-Link] Failed to send command after reconnect: {re_e}")

    def arm(self):
        cmd = {"action": "arm", "value": True}
        self._send_cmd_internal(cmd)

    def disarm(self):
        cmd = {"action": "arm", "value": False}
        self._send_cmd_internal(cmd)

    def drive(self, command_str, duration=0.0):
        # e.g., command_str: "FORWARD", "TURN_LEFT", "STOP"
        cmd = {"action": "drive", "dir": command_str, "duration": duration}
        self._send_cmd_internal(cmd)

    def stop(self):
        self.drive("STOP")


def broadcast_status(bridge, seq, vehicle_armed):
    armed_val = 1 if vehicle_armed else 0
    t_ms = int(time.time() * 1000) & 0xFFFFFFFF
    
    # Pack safety byte: GUIDED=1, Armable=true (0x10), GPS=True (0x20) => 0x31
    safety_byte = 0x31
    speed = 1.5 if vehicle_armed else 0.0

    bridge.send_telemetry(seq, t_ms, speed, 0.0, armed_val, safety_byte)


def main():
    # Connect to SimPixhawkUGV (Webots)
    try:
        vehicle = WebotsUGVLink(WEBOTS_PORT)
    except Exception as e:
        print(f"[!] Could not connect to Webots UGV. Ensure Webots is PLAYING! {e}")
        return

    # Connect to V2V Network (UDP)
    bridge = v2v_bridge.V2VBridge(ESP32_PORT, name="UGV-GS")
    bridge.send_message("UgV SiMULATION GROUND STATION LIVE")

    seq = 0
    vehicle_armed = False
    mission_active = False

    try:
        while True:
            try:
                # Broadcast our status to the UAV
                broadcast_status(bridge, seq, vehicle_armed)
                seq += 1

                # Check for commands from the UAV
                cmd = bridge.get_command()
                if cmd:
                    cmdSeq, cmdVal, payload = cmd
                    print(f"[Ground] CMD RECEIVED: {cmdVal} (payload={payload})")

                    if cmdVal == v2v_bridge.CMD_STOP and payload == 1:
                        mission_active = False
                        vehicle.stop()
                        vehicle.disarm()
                        vehicle_armed = False
                        print("[Ground] EMERGENCY STOP!")
                        continue

                    if not vehicle_armed:
                        print("[Ground] Auto-Arming UGV...")
                        vehicle.arm()
                        vehicle_armed = True
                        time.sleep(1)

                    # Execute explicit commands
                    if cmdVal == v2v_bridge.CMD_MOVE_FORWARD:
                        print("[Ground] Moving Forward 10s...")
                        vehicle.drive("FORWARD", duration=10.0)
                    elif cmdVal == v2v_bridge.CMD_MOVE_2FT:
                        vehicle.drive("FORWARD", duration=0.8)
                    elif cmdVal == v2v_bridge.CMD_TURN_RIGHT:
                        vehicle.drive("TURN_RIGHT", duration=1.0)
                    elif cmdVal == v2v_bridge.CMD_TURN_LEFT:
                        vehicle.drive("TURN_LEFT", duration=1.0)
                    elif cmdVal == v2v_bridge.CMD_CIRCLE:
                        vehicle.drive("CIRCLE", duration=8.0)
                    elif cmdVal == v2v_bridge.CMD_MISSION_1:
                        print(f"[Ground] Mission 1 Active Mode...")
                        mission_active = True
                        vehicle.drive("FORWARD", duration=0)  # drives forever until stopped
                    elif cmdVal in [v2v_bridge.CMD_MISSION_2, v2v_bridge.CMD_MISSION_3]:
                        print(f"[Ground] Mission {cmdVal-9} Active Mode...")
                        mission_active = True
                        vehicle.drive("FORWARD_SLOW", duration=0)  # drives forever until stopped
                    elif cmdVal == v2v_bridge.CMD_STOP:
                        mission_active = False
                        vehicle.stop()
                
            except Exception as e:
                print(f"[Ground] Loop Error: {e}")
                time.sleep(0.1)

            # The Webots UGV Controller handles continuous driving if duration=0
            time.sleep(1.0 / TELEM_SEND_HZ)

    except KeyboardInterrupt:
        print("\n[Ground] Exiting manually...")
    finally:
        try: vehicle.stop()
        except: pass
        try: bridge.stop()
        except: pass


if __name__ == "__main__":
    main()

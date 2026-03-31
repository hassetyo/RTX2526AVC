"""
Simulation UDP V2V Bridge - ESP-NOW Emulator
=============================================
Replaces the hardware ESP32 serial bridge for local PC simulation.
Uses UDP sockets (localhost:9001 and localhost:9002) to instantly and 
wirelessly transmit packets between your UAV python script and UGV python script.

Has the exact same API as your real v2v_bridge.py!
"""

import struct
import socket
import select
import time

TYPE_TELEM = 1
TYPE_CMD = 2
TYPE_MSG = 3

CMD_ARM          = 1
CMD_DISARM       = 2
CMD_TAKEOFF      = 3
CMD_LAND         = 4
CMD_MOVE_FORWARD = 5 
CMD_MOVE_2FT     = 6
CMD_TURN_RIGHT   = 7
CMD_TURN_LEFT    = 8
CMD_CIRCLE       = 9
CMD_MISSION_1    = 10
CMD_MISSION_2    = 11
CMD_MISSION_3    = 12
CMD_STOP         = 13
CMD_START_MISSION = 14
CMD_LAND_NOW      = 15

MODE_INITIAL  = 0
MODE_GUIDED   = 1
MODE_AUTO     = 2
MODE_LAND     = 3
MODE_DISARMED = 4

TELEM_FMT = "<IIffBB"
CMD_FMT   = "<IBB"

class V2VBridge:
    def __init__(self, port="/dev/sim", baud=115200, name="V2V"):
        self.name = name
        self.latest_telemetry = None
        self.latest_command = None
        self.latest_msg = None

        # UDP "Wireless" ESP-NOW Setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        # Automatically route based on who is using it
        if "UAV" in name.upper():
            self.my_port = 9001
            self.target_port = 9002
        else:
            self.my_port = 9002
            self.target_port = 9001

        self.sock.bind(('127.0.0.1', self.my_port))
        print(f"[{self.name}] SIMULATED ESP-NOW Bridge Ready on UDP {self.my_port}")

    def connect(self):
        # UDP is connectionless
        time.sleep(0.5)

    def stop(self):
        self.sock.close()

    def _send_raw(self, data):
        try:
            self.sock.sendto(data, ('127.0.0.1', self.target_port))
        except BlockingIOError:
            pass

    def send_telemetry(self, seq, t_ms, vx, vy, marker, estop):
        payload = struct.pack(TELEM_FMT, seq, t_ms, vx, vy, marker, estop)
        header = struct.pack("B", TYPE_TELEM)
        self._send_raw(header + payload)

    def send_command(self, cmdSeq, cmd, payload=0):
        # We map 'payload' to the 'estop' field in the real protocol for simplicity
        payload_data = struct.pack(CMD_FMT, cmdSeq, cmd, payload)
        header = struct.pack("B", TYPE_CMD)
        self._send_raw(header + payload_data)

    def send_message(self, text: str):
        data = text[:60].encode('ascii', errors='ignore')
        header = struct.pack("B", TYPE_MSG)
        self._send_raw(header + data)

    def _poll(self):
        """Reads all pending UDP packets internally"""
        while True:
            try:
                ready = select.select([self.sock], [], [], 0)
                if not ready[0]:
                    break
                data, _ = self.sock.recvfrom(1024)
                if len(data) < 2: continue

                msg_type = data[0]
                if msg_type == TYPE_TELEM and len(data) >= 1 + struct.calcsize(TELEM_FMT):
                    self.latest_telemetry = struct.unpack(TELEM_FMT, data[1:1+struct.calcsize(TELEM_FMT)])
                elif msg_type == TYPE_CMD and len(data) >= 1 + struct.calcsize(CMD_FMT):
                    self.latest_command = struct.unpack(CMD_FMT, data[1:1+struct.calcsize(CMD_FMT)])
                elif msg_type == TYPE_MSG:
                    self.latest_msg = data[1:].decode('ascii', errors='ignore')
            except BlockingIOError:
                break
            except Exception:
                break

    def get_telemetry(self):
        self._poll()
        return self.latest_telemetry

    def get_command(self, consume=True):
        self._poll()
        val = self.latest_command
        if consume: self.latest_command = None
        return val

    def get_message(self, consume=True):
        self._poll()
        val = self.latest_msg
        if consume: self.latest_msg = None
        return val

"""
Simulation V2V Bridge - Drop-in replacement for the real v2v_bridge.py
======================================================================
this replaces the hardware serial ESP32 communication with Webots 
Emitter/Receiver nodes. same API as the real v2v_bridge.py so the 
mission scripts dont need to change their calling code.

used by the mission code in src/uav_mission/ and src/ugv_mission/
when running inside the webots simulation.
"""

import struct
import time

# protocol constants (identical to the real v2v_bridge.py)
SOF = 0xAA
TYPE_TELEM = 1
TYPE_CMD = 2
TYPE_MSG = 3

# command codes (identical to real v2v_bridge.py)
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

# mode identifiers (identical to real v2v_bridge.py)
MODE_INITIAL  = 0
MODE_GUIDED   = 1
MODE_AUTO     = 2
MODE_LAND     = 3
MODE_DISARMED = 4

# pack formats (identical to real v2v_bridge.py)
TELEM_FMT = "<IIffBB"
CMD_FMT   = "<IBB"


class SimV2VBridge:
    """
    simulation version of V2VBridge that uses webots Emitter/Receiver
    instead of hardware serial. same API as the real v2v_bridge.py so
    you can swap them without changing mission code.
    
    usage in webots controller:
        from sim_v2v_bridge import SimV2VBridge
        bridge = SimV2VBridge(emitter, receiver, name="UAV-Bridge")
        bridge.send_command(cmdSeq=1, cmd=CMD_MISSION_1, estop=0)
    """

    def __init__(self, emitter, receiver, name="SimBridge"):
        self.emitter = emitter
        self.receiver = receiver
        self.name = name

        # latest received data (same pattern as real bridge)
        self.latest_telemetry = None
        self.latest_command = None
        self.latest_msg = None

        print(f"[{self.name}] Simulation V2V Bridge initialized.")

    def connect(self):
        """no-op in simulation - emitter/receiver are already enabled"""
        print(f"[{self.name}] Sim Bridge Ready (no serial needed).")

    def stop(self):
        """no-op in simulation"""
        print(f"[{self.name}] Sim Bridge stopped.")

    # ============================================================
    # SENDING (uses Webots Emitter)
    # ============================================================
    def send_telemetry(self, seq, t_ms, vx, vy, marker, estop):
        """packs telemetry and sends via emitter (same format as real bridge)"""
        payload = struct.pack(TELEM_FMT, seq, t_ms, vx, vy, marker, estop)
        header = struct.pack("B", TYPE_TELEM)
        self.emitter.send(header + payload)

    def send_command(self, cmdSeq, cmd, estop):
        """packs command and sends via emitter (same format as real bridge)"""
        payload = struct.pack(CMD_FMT, cmdSeq, cmd, estop)
        header = struct.pack("B", TYPE_CMD)
        self.emitter.send(header + payload)

    def send_message(self, text: str):
        """sends raw text via emitter"""
        data = text[:60].encode('ascii', errors='ignore')
        header = struct.pack("B", TYPE_MSG)
        self.emitter.send(header + data)

    # ============================================================
    # RECEIVING (uses Webots Receiver)
    # ============================================================
    def poll(self):
        """
        processes all pending messages from the receiver.
        call this once per timestep to keep the mailboxes fresh.
        """
        while self.receiver.getQueueLength() > 0:
            data = self.receiver.getBytes()
            if len(data) < 2:
                self.receiver.nextPacket()
                continue

            msg_type = data[0]

            if msg_type == TYPE_TELEM and len(data) >= 1 + struct.calcsize(TELEM_FMT):
                self.latest_telemetry = struct.unpack(TELEM_FMT, data[1:1+struct.calcsize(TELEM_FMT)])
            elif msg_type == TYPE_CMD and len(data) >= 1 + struct.calcsize(CMD_FMT):
                self.latest_command = struct.unpack(CMD_FMT, data[1:1+struct.calcsize(CMD_FMT)])
            elif msg_type == TYPE_MSG:
                self.latest_msg = data[1:].decode('ascii', errors='ignore')

            self.receiver.nextPacket()

    def get_telemetry(self):
        """returns the latest telemetry tuple or None"""
        self.poll()
        return self.latest_telemetry

    def get_command(self, consume=True):
        """returns the latest command tuple and optionally clears it"""
        self.poll()
        val = self.latest_command
        if consume:
            self.latest_command = None
        return val

    def get_message(self, consume=True):
        """returns the latest text message and optionally clears it"""
        self.poll()
        val = self.latest_msg
        if consume:
            self.latest_msg = None
        return val

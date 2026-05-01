import serial
import struct
import threading
import time

# ========================================================
# V2V BRIDGE LIBRARY - GPS COMMAND VERSION
# ========================================================
# Supports:
#
# UAV -> UGV:
#   GPS_CMD:12,32.985123,-96.750456
#   FORWARD_CMD:13,5.0
#
# UGV -> UAV:
#   GPS_STATUS:lat,lon,alt,heading,fix,sats
#   CMD_STARTED:12
#   CMD_ACTIVE:12
#   CMD_DONE:12
#   CMD_FAILED:12
#   CMD_BUSY:12
# ========================================================

SOF = 0xAA

TYPE_TELEM = 1
TYPE_CMD = 2
TYPE_MSG = 3

# ----------------------------
# Old command codes
# ----------------------------
CMD_ARM = 1
CMD_DISARM = 2
CMD_TAKEOFF = 3
CMD_LAND = 4
CMD_MOVE_FORWARD = 5
CMD_MOVE_2FT = 6
CMD_TURN_RIGHT = 7
CMD_TURN_LEFT = 8
CMD_CIRCLE = 9
CMD_MISSION_1 = 10
CMD_MISSION_2 = 11
CMD_MISSION_3 = 12
CMD_STOP = 13

# Old parametric command codes
CMD_TURN_ANGLE_RIGHT = 14
CMD_TURN_ANGLE_LEFT = 15
CMD_MOVE_DIST = 16

# ----------------------------
# Mode identifiers
# ----------------------------
MODE_INITIAL = 0
MODE_GUIDED = 1
MODE_AUTO = 2
MODE_LAND = 3
MODE_DISARMED = 4

# ----------------------------
# Packet formats
# ----------------------------
TELEM_FMT = "<IIffBB"
CMD_FMT = "<IBB"


class V2VBridge:
    def __init__(self, port, baud=115200, name="Bridge"):
        self.port = port
        self.baud = baud
        self.name = name
        self.ser = None

        self.latest_telemetry = None
        self.latest_command = None
        self.latest_msg = None

        # Queue keeps CMD_DONE from getting overwritten by GPS_STATUS messages
        self.msg_queue = []

        # Latest parsed UGV GPS data
        self.latest_gps_status = None

        # Latest command response from UGV
        self.latest_cmd_response = None

        # Command ID counter for repeat-safe GPS commands
        self._next_command_id = 1

        self._running = False
        self._lock = threading.Lock()
        self._thread = None

    # =====================================================
    # Connection
    # =====================================================
    def connect(self):
        print(f"[{self.name}] Connecting to {self.port} at {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        print(f"[{self.name}] Bridge Thread Running... Listening for radio.")

    def stop(self):
        self._running = False

        if self._thread:
            self._thread.join(timeout=1.0)

        if self.ser:
            self.ser.close()

    # =====================================================
    # Checksum
    # =====================================================
    def _chk_xor(self, type_b, len_b, payload: bytes) -> int:
        c = (type_b ^ len_b) & 0xFF

        for b in payload:
            c ^= b

        return c & 0xFF

    # =====================================================
    # Read loop
    # =====================================================
    def _read_loop(self):
        while self._running:
            if self.ser.in_waiting == 0:
                time.sleep(0.001)
                continue

            b = self.ser.read(1)

            if not b or b[0] != SOF:
                continue

            hdr = self.ser.read(2)

            if len(hdr) < 2:
                continue

            f_type = hdr[0]
            f_len = hdr[1]

            payload = self.ser.read(f_len)

            if len(payload) < f_len:
                continue

            chk_byte = self.ser.read(1)

            if not chk_byte:
                continue

            if chk_byte[0] != self._chk_xor(f_type, f_len, payload):
                continue

            with self._lock:
                if f_type == TYPE_TELEM and f_len == struct.calcsize(TELEM_FMT):
                    self.latest_telemetry = struct.unpack(TELEM_FMT, payload)

                elif f_type == TYPE_CMD and f_len == struct.calcsize(CMD_FMT):
                    self.latest_command = struct.unpack(CMD_FMT, payload)

                elif f_type == TYPE_MSG:
                    msg = payload.decode("ascii", errors="ignore")

                    self.latest_msg = msg
                    self.msg_queue.append(msg)

                    # Prevent queue from growing forever
                    if len(self.msg_queue) > 100:
                        self.msg_queue.pop(0)

                    self._handle_incoming_text_message(msg)

    # =====================================================
    # Incoming message parsing
    # =====================================================
    def _handle_incoming_text_message(self, msg):
        gps = self._parse_gps_status(msg)

        if gps is not None:
            self.latest_gps_status = gps
            return

        cmd_response = self._parse_command_response(msg)

        if cmd_response is not None:
            self.latest_cmd_response = cmd_response
            return

    def _parse_gps_status(self, msg):
        """
        Parses:
            GPS_STATUS:lat,lon,alt,heading,fix,sats

        Returns:
            {
                "lat": float,
                "lon": float,
                "alt": float,
                "heading": float,
                "fix": int,
                "sats": int,
            }
        """

        if not msg.startswith("GPS_STATUS:"):
            return None

        payload = msg.split(":", 1)[1].strip()

        if payload == "UNAVAILABLE":
            return {
                "available": False,
                "lat": None,
                "lon": None,
                "alt": None,
                "heading": None,
                "fix": 0,
                "sats": 0,
            }

        try:
            lat_str, lon_str, alt_str, heading_str, fix_str, sats_str = payload.split(",")

            return {
                "available": True,
                "lat": float(lat_str),
                "lon": float(lon_str),
                "alt": float(alt_str),
                "heading": float(heading_str),
                "fix": int(float(fix_str)),
                "sats": int(float(sats_str)),
            }

        except Exception:
            return None

    def _parse_command_response(self, msg):
        """
        Parses:
            CMD_STARTED:12
            CMD_ACTIVE:12
            CMD_DONE:12
            CMD_FAILED:12
            CMD_BUSY:12

        Returns:
            {
                "status": "DONE",
                "command_id": 12
            }
        """

        valid_prefixes = [
            "CMD_STARTED:",
            "CMD_ACTIVE:",
            "CMD_DONE:",
            "CMD_FAILED:",
            "CMD_BUSY:",
        ]

        for prefix in valid_prefixes:
            if msg.startswith(prefix):
                try:
                    status = prefix.replace("CMD_", "").replace(":", "")
                    command_id = int(msg.split(":", 1)[1].strip())

                    return {
                        "status": status,
                        "command_id": command_id,
                        "raw": msg,
                    }

                except Exception:
                    return None

        return None

    # =====================================================
    # Basic getters
    # =====================================================
    def get_telemetry(self):
        with self._lock:
            return self.latest_telemetry

    def get_command(self, consume=True):
        with self._lock:
            val = self.latest_command

            if consume:
                self.latest_command = None

            return val

    def get_message(self, consume=True):
        with self._lock:
            val = self.latest_msg

            if consume:
                self.latest_msg = None

            return val

    def get_next_message(self):
        """
        Safer than get_message() when GPS_STATUS messages are coming constantly.
        This returns the oldest unread text message.
        """

        with self._lock:
            if not self.msg_queue:
                return None

            return self.msg_queue.pop(0)

    def get_latest_gps_status(self):
        with self._lock:
            return self.latest_gps_status

    def get_latest_command_response(self):
        with self._lock:
            return self.latest_cmd_response

    def clear_buffers(self):
        with self._lock:
            self.latest_command = None
            self.latest_msg = None
            self.msg_queue.clear()
            self.latest_cmd_response = None

    # =====================================================
    # Basic senders
    # =====================================================
    def send_telemetry(self, seq, t_ms, vx, vy, marker, estop):
        payload = struct.pack(
            TELEM_FMT,
            int(seq),
            int(t_ms),
            float(vx),
            float(vy),
            int(marker),
            int(estop),
        )

        chk = self._chk_xor(TYPE_TELEM, len(payload), payload)

        self.ser.write(
            bytes([SOF, TYPE_TELEM, len(payload)])
            + payload
            + bytes([chk])
        )

        self.ser.flush()

    def send_command(self, cmdSeq, cmd, estop):
        payload = struct.pack(
            CMD_FMT,
            int(cmdSeq),
            int(cmd),
            int(estop),
        )

        chk = self._chk_xor(TYPE_CMD, len(payload), payload)

        self.ser.write(
            bytes([SOF, TYPE_CMD, len(payload)])
            + payload
            + bytes([chk])
        )

        self.ser.flush()

    def send_message(self, text: str):
        payload = text[:60].encode("ascii", errors="ignore")
        chk = self._chk_xor(TYPE_MSG, len(payload), payload)

        self.ser.write(
            bytes([SOF, TYPE_MSG, len(payload)])
            + payload
            + bytes([chk])
        )

        self.ser.flush()

    # =====================================================
    # Old helper commands
    # =====================================================
    def send_dynamic_move(self, distance_meters: float):
        dist_cm = int(abs(distance_meters) * 100.0)
        self.send_command(cmdSeq=dist_cm, cmd=CMD_MOVE_DIST, estop=0)

    def send_dynamic_turn(self, angle_degrees: float):
        angle = int(abs(angle_degrees))

        if angle_degrees > 0:
            self.send_command(
                cmdSeq=angle,
                cmd=CMD_TURN_ANGLE_RIGHT,
                estop=0,
            )
        else:
            self.send_command(
                cmdSeq=angle,
                cmd=CMD_TURN_ANGLE_LEFT,
                estop=0,
            )

    def send_estop(self):
        self.send_command(cmdSeq=0, cmd=CMD_STOP, estop=1)

    # =====================================================
    # New GPS command helpers
    # =====================================================
    def get_new_command_id(self):
        with self._lock:
            command_id = self._next_command_id
            self._next_command_id += 1

            if self._next_command_id > 999999:
                self._next_command_id = 1

            return command_id

    def send_gps_command(self, command_id, lat, lon):
        """
        Sends one GPS coordinate command to the UGV.

        Format:
            GPS_CMD:12,32.985123,-96.750456
        """

        msg = f"GPS_CMD:{int(command_id)},{float(lat):.7f},{float(lon):.7f}"
        self.send_message(msg)

    def send_forward_command(self, command_id, distance_meters):
        """
        Sends one forward-distance GPS command to the UGV.

        Format:
            FORWARD_CMD:13,5.0
        """

        msg = f"FORWARD_CMD:{int(command_id)},{float(distance_meters):.2f}"
        self.send_message(msg)

    def send_gps_command_until_done(
        self,
        lat,
        lon,
        command_id=None,
        resend_interval_s=0.5,
        timeout_s=120,
    ):
        """
        Repeatedly sends a GPS command until the UGV responds with CMD_DONE.

        Returns:
            True if CMD_DONE received
            False if CMD_FAILED or timeout
        """

        if command_id is None:
            command_id = self.get_new_command_id()

        start_t = time.time()

        while time.time() - start_t < timeout_s:
            self.send_gps_command(command_id, lat, lon)

            response = self.wait_for_command_response(
                command_id=command_id,
                desired_statuses=["DONE", "FAILED"],
                timeout_s=resend_interval_s,
            )

            if response is not None:
                status = response["status"]

                if status == "DONE":
                    return True

                if status == "FAILED":
                    return False

            time.sleep(resend_interval_s)

        return False

    def send_forward_command_until_done(
        self,
        distance_meters,
        command_id=None,
        resend_interval_s=0.5,
        timeout_s=120,
    ):
        """
        Repeatedly sends a forward command until the UGV responds with CMD_DONE.

        Returns:
            True if CMD_DONE received
            False if CMD_FAILED or timeout
        """

        if command_id is None:
            command_id = self.get_new_command_id()

        start_t = time.time()

        while time.time() - start_t < timeout_s:
            self.send_forward_command(command_id, distance_meters)

            response = self.wait_for_command_response(
                command_id=command_id,
                desired_statuses=["DONE", "FAILED"],
                timeout_s=resend_interval_s,
            )

            if response is not None:
                status = response["status"]

                if status == "DONE":
                    return True

                if status == "FAILED":
                    return False

            time.sleep(resend_interval_s)

        return False

    def wait_for_command_response(
        self,
        command_id,
        desired_statuses=None,
        timeout_s=5.0,
    ):
        """
        Waits for a command response for a specific command_id.

        Example desired_statuses:
            ["DONE"]
            ["DONE", "FAILED"]
            ["STARTED", "ACTIVE", "DONE"]
        """

        if desired_statuses is None:
            desired_statuses = ["DONE"]

        desired_statuses = set(desired_statuses)
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            msg = self.get_next_message()

            if msg is None:
                time.sleep(0.01)
                continue

            response = self._parse_command_response(msg)

            if response is None:
                continue

            if response["command_id"] != command_id:
                continue

            if response["status"] in desired_statuses:
                return response

        return None

    def wait_for_gps_status(self, timeout_s=5.0):
        """
        Waits until the UGV sends a GPS_STATUS message.
        """

        deadline = time.time() + timeout_s

        while time.time() < deadline:
            gps = self.get_latest_gps_status()

            if gps is not None:
                return gps

            time.sleep(0.05)

        return None
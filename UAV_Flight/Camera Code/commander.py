import time
import v2v_bridge

class UGVCommander:
    def __init__(self, bridge_port, bridge_baud, turn_threshold_deg, stop_distance_m, step_min_m, step_max_m, drive_speed_mps, marker_timeout_sec):
        self.bridge = v2v_bridge.V2VBridge(bridge_port, baud=bridge_baud, name="UGV-Nav-Bridge")
        self.turn_threshold_deg = turn_threshold_deg
        self.stop_distance_m = stop_distance_m
        self.step_min_m, self.step_max_m = step_min_m, step_max_m
        self.drive_speed_mps, self.marker_timeout_sec = drive_speed_mps, marker_timeout_sec
        self.seq, self.arm_sent = 1, False
        self.last_motion, self.next_drive_time, self.last_seen_time = "idle", 0.0, 0.0
        self.last_status_text = "init"

    def connect(self):
        self.bridge.connect()
        self.bridge.send_message("aruco ugv navigator online")

    def ensure_armed(self):
        if not self.arm_sent:
            self.bridge.send_command(self.seq, v2v_bridge.CMD_ARM, 0)
            self.seq += 1
            self.arm_sent = True
            time.sleep(0.25)

    def send_turn(self, direction):
        cmd = v2v_bridge.CMD_TURN_RIGHT if direction == "right" else v2v_bridge.CMD_TURN_LEFT
        if self.last_motion != f"turn_{direction}":
            self.bridge.send_command(self.seq, cmd, 0)
            self.seq += 1
            self.last_motion = f"turn_{direction}"

    def send_stop(self, force=False):
        if force or self.last_motion != "stopped":
            self.bridge.send_command(self.seq, v2v_bridge.CMD_STOP, 0)
            self.seq += 1
            self.last_motion = "stopped"

    def send_forward_step(self, step_m):
        now = time.time()
        if now < self.next_drive_time: return
        step_m = max(self.step_min_m, min(step_m, self.step_max_m))
        duration = max(step_m / max(self.drive_speed_mps, 1e-6), 0.15)
        self.bridge.send_message(f"GOTO:{step_m:.3f},0")
        self.last_motion = "forward"
        self.next_drive_time = now + duration + 0.25

    def close(self):
        self.send_stop(force=True)
        self.bridge.stop()
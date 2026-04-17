"""
VLandArUco – Vertical takeoff, hover 15 s, then track & precision-land on
a moving UGV carrying ArUco marker ID 5.

Flight sequence:
  1. ALT_HOLD takeoff to TARGET_ALT_M using proportional throttle.
  2. Hover for HOVER_WAIT_S (15 s) to stabilise.
  3. Begin ArUco tracking (marker ID 5).  Once the marker is stably held
     for PLND_STABLE_FRAMES, switch to GUIDED and start velocity-based
     centering (Precision Loiter).
  4. When lateral error drops below LAND_LATERAL_ERR_M, begin descending
     while continuing to centre (Precision Land).
  5. Touchdown confirmed by auto-disarm heartbeat -> script exits.

Emergency:  KeyboardInterrupt or RC channel 7 high -> immediate LAND.
"""

from pymavlink import mavutil
import time
import math
import os
import cv2
import cv2.aruco as aruco
import numpy as np
from typing import Dict, Optional, Tuple
from dataclasses import dataclass

# ─── CONNECTION ──────────────────────────────────────────────────────────────────
CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE         = 57600

# ─── FLIGHT PARAMS ───────────────────────────────────────────────────────────────
TARGET_ALT_M       = 2.0              # metres to climb to
HOVER_WAIT_S       = 15.0             # seconds to hover before starting ArUco tracking
ALT_TOLERANCE_M    = 0.25             # how close to target counts as "reached"
CLIMB_LOOP_DT      = 0.10
HOVER_LOOP_DT      = 0.10
LAND_TIMEOUT_S     = 60.0

# ─── THROTTLE TUNING (ALT_HOLD proportional controller) ─────────────────────────
THROTTLE_MIN   = 1000
THROTTLE_HOVER = 1500
THROTTLE_CLIMB = 1650

# ─── RC EMERGENCY LAND SWITCH ───────────────────────────────────────────────────
EMERGENCY_LAND_CH            = 7
EMERGENCY_LAND_PWM_THRESHOLD = 1800

# ─── ARUCO / VISION PARAMS ──────────────────────────────────────────────────────
MARKER_ID         = 5
MARKER_SIZE_M     = 0.3048            # 12 inches in metres
SEND_HZ           = 15.0
PLND_STABLE_FRAMES = 15               # consecutive detections before engaging centering
LAND_LATERAL_ERR_M = 0.20             # lateral error threshold to start descent
DESCENT_SPEED_MPS  = 0.3              # default descent speed when LiDAR unavailable
CENTERING_KP       = 0.8              # proportional gain for lateral velocity

# ─── LOGGING ─────────────────────────────────────────────────────────────────────
LOG_FILE = "vland_aruco_log.txt"


def log_event(text):
    ts = time.strftime("%H:%M:%S")
    line = f"[{ts}] {text}"
    print(line)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")


# ═════════════════════════════════════════════════════════════════════════════════
# CAMERA & ARUCO (from precision_land.py)
# ═════════════════════════════════════════════════════════════════════════════════
@dataclass
class MarkerPose:
    marker_id: int
    rvec: np.ndarray
    tvec: np.ndarray
    center_px: Tuple[int, int]
    corners: np.ndarray


class CameraInterface:
    def __init__(self, use_zed: bool = False, camera_index: int = 0,
                 width: int = 1280, height: int = 720, fps: int = 30):
        self.use_zed = use_zed
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.zed = None
        self.sl = None
        self.camera_matrix = None
        self.dist_coeffs = None
        if self.use_zed:
            self._open_zed()
        else:
            self._open_standard()

    def _open_zed(self):
        import pyzed.sl as sl
        self.sl = sl
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = self.fps
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {err}")
        cam_info = self.zed.get_camera_information()
        calib = cam_info.camera_configuration.calibration_parameters.left_cam
        self.camera_matrix = np.array(
            [[calib.fx, 0.0, calib.cx],
             [0.0, calib.fy, calib.cy],
             [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.array(calib.disto, dtype=np.float64).flatten()
        if dist.size >= 5:
            self.dist_coeffs = dist[:5].reshape(-1, 1)
        elif dist.size > 0:
            self.dist_coeffs = dist.reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def _open_standard(self):
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {self.camera_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def load_standard_calibration(self, yaml_path: Optional[str]):
        if yaml_path:
            fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
            K = fs.getNode("K").mat()
            D = fs.getNode("D").mat()
            fs.release()
            if K is not None and D is not None:
                self.camera_matrix = np.array(K, dtype=np.float64)
                self.dist_coeffs = np.array(D, dtype=np.float64)
                log_event(f"Loaded calibration from {yaml_path}")

    def get_frame(self) -> Optional[np.ndarray]:
        if self.use_zed:
            if self.zed.grab() != self.sl.ERROR_CODE.SUCCESS:
                return None
            image = self.sl.Mat()
            self.zed.retrieve_image(image, self.sl.VIEW.LEFT)
            frame = image.get_data()
            if frame.shape[-1] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            else:
                frame = frame.copy()
            return frame
        else:
            ret, frame = self.cap.read()
            return frame if ret else None

    def close(self):
        if self.use_zed and self.zed is not None:
            self.zed.close()
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


class ArucoDistanceEstimator:
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
                 dictionary_name: int = aruco.DICT_6X6_1000):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.aruco_dict = aruco.getPredefinedDictionary(dictionary_name)
        if hasattr(aruco, "ArucoDetector"):
            self.detector_params = aruco.DetectorParameters()
            self._enhance_params(self.detector_params)
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            self.use_new_api = True
        else:
            self.detector_params = aruco.DetectorParameters_create()
            self._enhance_params(self.detector_params)
            self.detector = None
            self.use_new_api = False

    def _enhance_params(self, params):
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 10
        params.polygonalApproxAccuracyRate = 0.05

    def detect_markers(self, frame: np.ndarray, marker_size_m: float) -> Dict[int, MarkerPose]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params)
        poses: Dict[int, MarkerPose] = {}
        if ids is None or len(ids) == 0:
            return poses
        for i, mid in enumerate(ids.flatten()):
            rvec, tvec = self._estimate_pose(corners[i], marker_size_m)
            if rvec is None or tvec is None:
                continue
            center = np.mean(corners[i][0], axis=0).astype(int)
            poses[int(mid)] = MarkerPose(
                marker_id=int(mid), rvec=rvec, tvec=tvec.reshape(3),
                center_px=(int(center[0]), int(center[1])), corners=corners[i][0])
        return poses

    def _estimate_pose(self, corner: np.ndarray, marker_size_m: float):
        half = marker_size_m / 2.0
        obj_pts = np.array([
            [-half, -half, 0.0], [ half, -half, 0.0],
            [ half,  half, 0.0], [-half,  half, 0.0]], dtype=np.float32)
        img_pts = corner.reshape((4, 2)).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE)
        return (rvec, tvec) if ok else (None, None)


# ═════════════════════════════════════════════════════════════════════════════════
# MAVLINK HELPERS (from VLand.py)
# ═════════════════════════════════════════════════════════════════════════════════
def connect():
    log_event(f"Connecting to {CONNECTION_STRING} ...")
    if CONNECTION_STRING.startswith(("udp:", "tcp:")):
        master = mavutil.mavlink_connection(CONNECTION_STRING)
    else:
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    hb = master.wait_heartbeat(timeout=10)
    if not hb:
        raise RuntimeError("No heartbeat – check connection")
    log_event("Heartbeat received.")
    request_message_streams(master)
    return master


def request_message_streams(master):
    try:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    except Exception:
        pass

    def set_interval(msg_id, hz):
        try:
            us = int(1e6 / hz)
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, us, 0, 0, 0, 0, 0)
        except Exception:
            pass

    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 15)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 5)
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW, 10)


def change_mode(master, *mode_names):
    mapping = master.mode_mapping()
    for mode in mode_names:
        if mode in mapping:
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mapping[mode])
            log_event(f"Mode -> {mode}")
            time.sleep(1)
            return mode
    raise RuntimeError(f"No valid mode in {mode_names}. Available: {list(mapping.keys())}")


def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    log_event("Arm command sent.")
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack:
        log_event(f"Arm ACK result: {ack.result}")


def disarm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)
    log_event("Disarm command sent.")


def set_throttle(master, pwm):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, int(pwm), 0, 0, 0, 0, 0)


def clear_rc_override(master):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0)


def send_guided_velocity(master, vx, vy, vz):
    """Body-frame velocity: vx forward, vy right, vz down (positive = descend)."""
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        float(vx), float(vy), float(vz),
        0, 0, 0, 0, 0)


def send_landing_target(master, x_b, y_b, z_b):
    dist = abs(z_b)
    master.mav.landing_target_send(
        int(time.time() * 1e6), 0,
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        0.0, 0.0,
        dist, 0.0, 0.0,
        x_b, y_b, dist,
        (1.0, 0.0, 0.0, 0.0), 0, 1)


# ─── SENSOR READERS ─────────────────────────────────────────────────────────────
def get_rangefinder_alt(master):
    msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    while msg:
        alt = msg.current_distance / 100.0
        if alt > 0.01:
            return alt
        msg = master.recv_match(type="DISTANCE_SENSOR", blocking=False)
    return None


def get_baro_relative_alt(master):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg:
        return msg.relative_alt / 1000.0
    return None


def get_altitude(master):
    rng = get_rangefinder_alt(master)
    if rng is not None:
        return rng, "lidar"
    baro = get_baro_relative_alt(master)
    if baro is not None:
        return baro, "baro"
    return None, "none"


def get_optical_flow(master):
    msg = master.recv_match(type="OPTICAL_FLOW", blocking=False)
    if msg:
        return msg.flow_x, msg.flow_y, msg.quality
    return None, None, None


def print_sensors(master):
    fx, fy, qual = get_optical_flow(master)
    alt, src = get_altitude(master)
    flow_str = (f"Flow X:{fx:7.2f}  Y:{fy:7.2f}  Q:{qual}"
                if fx is not None else "Flow: no data")
    alt_str = (f"Alt:{alt:5.2f}m ({src})" if alt is not None else "Alt: no data")
    print(f"  {flow_str}  |  {alt_str}", end="\r", flush=True)
    return alt


def check_rc_emergency_switch(master):
    msg = master.recv_match(type='RC_CHANNELS', blocking=False)
    if msg is not None:
        ch_val = getattr(msg, f'chan{EMERGENCY_LAND_CH}_raw', None)
        if ch_val is not None and int(ch_val) > EMERGENCY_LAND_PWM_THRESHOLD:
            return True
    return False


# ─── LANDING ─────────────────────────────────────────────────────────────────────
def land_safely(master):
    log_event("Commanding LAND mode...")
    change_mode(master, "LAND")
    clear_rc_override(master)

    log_event("Waiting for auto-disarm ...")
    deadline = time.time() + LAND_TIMEOUT_S
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
        if hb is not None:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                print()
                log_event("Motors disarmed – touchdown confirmed.")
                return
        print_sensors(master)
        time.sleep(0.25)

    print()
    log_event("[WARN] Land timeout – sending disarm fallback.")
    disarm(master)
    time.sleep(2.0)


# ═════════════════════════════════════════════════════════════════════════════════
# MAIN MISSION
# ═════════════════════════════════════════════════════════════════════════════════
def main():
    master = connect()

    # ── CAMERA INIT ───────────────────────────────────────────────────────
    USE_ZED = True
    yaml_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "calibration_chessboard.yaml")

    try:
        cam = CameraInterface(use_zed=USE_ZED, camera_index=0, fps=30)
        cam.load_standard_calibration(yaml_path)
    except Exception as e:
        log_event(f"Failed to open camera: {e}")
        return

    estimator = ArucoDistanceEstimator(
        cam.camera_matrix, cam.dist_coeffs, aruco.DICT_6X6_1000)

    try:
        # ── ARM & TAKEOFF (ALT_HOLD) ─────────────────────────────────────
        change_mode(master, "ALT_HOLD", "ALTHOLD")
        arm(master)

        log_event(f"Climbing to {TARGET_ALT_M:.2f} m ...")
        stable_start = None

        while True:
            if check_rc_emergency_switch(master):
                log_event("RC EMERGENCY SWITCH – landing immediately")
                land_safely(master)
                cam.close()
                return

            alt = print_sensors(master)

            if alt is not None:
                err = TARGET_ALT_M - alt
                thr = int(THROTTLE_HOVER + 150 * err)
                thr = max(THROTTLE_MIN, min(1800, thr))
            else:
                thr = THROTTLE_CLIMB
            set_throttle(master, thr)

            if alt is not None and alt >= (TARGET_ALT_M - ALT_TOLERANCE_M):
                if stable_start is None:
                    stable_start = time.time()
                elif (time.time() - stable_start) >= 1.0:
                    print()
                    log_event(f"Target altitude reached: {alt:.2f} m")
                    break
            else:
                stable_start = None

            time.sleep(CLIMB_LOOP_DT)

        # ── HOVER WAIT (15 s) before starting ArUco tracking ─────────────
        log_event(f"Hovering for {HOVER_WAIT_S:.0f} s before ArUco tracking ...")
        hover_start = time.time()

        while (time.time() - hover_start) < HOVER_WAIT_S:
            if check_rc_emergency_switch(master):
                log_event("RC EMERGENCY SWITCH – landing immediately")
                land_safely(master)
                cam.close()
                return

            alt = print_sensors(master)

            if alt is not None:
                err = TARGET_ALT_M - alt
                thr = int(THROTTLE_HOVER + 80 * err)
                thr = max(1400, min(1600, thr))
            else:
                thr = THROTTLE_HOVER
            set_throttle(master, thr)

            time.sleep(HOVER_LOOP_DT)

        print()
        log_event("Hover wait complete – switching to GUIDED for ArUco tracking.")

        # ── SWITCH TO GUIDED ──────────────────────────────────────────────
        clear_rc_override(master)
        change_mode(master, "GUIDED")
        
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            print(f"Autopilot Status: {msg.text}")

        # ── ARUCO TRACKING & PRECISION LAND ───────────────────────────────
        state = "SEARCHING"    # SEARCHING -> PREC_LOITER -> LANDING -> LANDED
        stable_count = 0
        last_send = 0.0

        log_event("ArUco tracking active – looking for marker ID " + str(MARKER_ID))

        while True:
            if check_rc_emergency_switch(master):
                log_event("RC EMERGENCY SWITCH – landing immediately")
                land_safely(master)
                cam.close()
                return

            frame = cam.get_frame()
            if frame is None:
                continue

            # Detect marker
            poses = estimator.detect_markers(frame, MARKER_SIZE_M)

            found = False
            target_eb = None

            if MARKER_ID in poses:
                pose = poses[MARKER_ID]
                x_cam = float(pose.tvec[0])
                y_cam = float(pose.tvec[1])
                z_cam = float(pose.tvec[2])

                # Camera frame -> ArduPilot FRD body frame
                x_b = -y_cam   # Forward
                y_b =  x_cam   # Right
                z_b =  z_cam   # Down
                target_eb = (x_b, y_b, z_b)

                stable_count += 1
                found = True

                # Draw overlay
                cam_cx, cam_cy = frame.shape[1] // 2, frame.shape[0] // 2
                cv2.line(frame, (cam_cx, cam_cy), pose.center_px, (0, 0, 255), 4)

                corners_arr = [pose.corners.reshape(1, 4, 2).astype(np.float32)]
                aruco.drawDetectedMarkers(
                    frame, corners_arr, np.array([[MARKER_ID]]))
                cv2.drawFrameAxes(
                    frame, cam.camera_matrix, cam.dist_coeffs,
                    pose.rvec, pose.tvec.reshape(3, 1),
                    MARKER_SIZE_M * 0.5)

                cv2.putText(frame, f"TRACKING ID: {MARKER_ID}",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame,
                    f"XYZ: [{x_b:.2f}, {y_b:.2f}, {z_b:.2f}]",
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Send velocity + landing target at SEND_HZ
                now = time.time()
                if now - last_send >= (1.0 / SEND_HZ):
                    if state in ("SEARCHING", "PREC_LOITER", "LANDING"):
                        vx = max(-0.8, min(0.8, x_b * CENTERING_KP))
                        vy = max(-0.8, min(0.8, y_b * CENTERING_KP))
                        vz = 0.0

                        if state == "LANDING":
                            # Descend while centering
                            lidar_m = get_rangefinder_alt(master)
                            if lidar_m is not None:
                                vz = max(0.15, min(0.4, lidar_m * 0.3))
                                cv2.putText(frame, f"LIDAR: {lidar_m:.2f}m",
                                    (20, 120), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 255, 0), 2)
                            else:
                                vz = DESCENT_SPEED_MPS
                                cv2.putText(frame, "LIDAR N/A - BLIND DESCENT",
                                    (20, 120), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 0, 255), 2)

                        send_guided_velocity(master, vx, vy, vz)
                        send_landing_target(master, x_b, y_b, z_b)
                    last_send = now
            else:
                stable_count = 0
                now = time.time()
                if now - last_send >= (1.0 / SEND_HZ):
                    cv2.putText(frame, "TARGET LOST - HOVERING",
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)
                    send_guided_velocity(master, 0.0, 0.0, 0.0)
                    last_send = now

            # ── STATE MACHINE ─────────────────────────────────────────────
            if state == "SEARCHING":
                if stable_count >= PLND_STABLE_FRAMES:
                    log_event("Marker stably detected – engaging centering.")
                    state = "PREC_LOITER"

            elif state == "PREC_LOITER":
                if stable_count == 0:
                    log_event("Lost marker – reverting to search.")
                    state = "SEARCHING"
                elif found and target_eb is not None:
                    err_m = math.sqrt(target_eb[0]**2 + target_eb[1]**2)
                    cv2.putText(frame, f"Align Err: {err_m:.2f}m",
                        (20, 90), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2)
                    if err_m < LAND_LATERAL_ERR_M:
                        log_event("Aligned! Starting precision descent.")
                        state = "LANDING"

            elif state == "LANDING":
                # Check for disarm (touchdown)
                hb = master.recv_match(type="HEARTBEAT", blocking=False)
                if hb is not None:
                    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if not armed:
                        log_event("TOUCHDOWN AUTO-DISARM – landing complete.")
                        state = "LANDED"
                        break
                if found and target_eb is not None:
                    err_m = math.sqrt(target_eb[0]**2 + target_eb[1]**2)
                    cv2.putText(frame, f"Align Err: {err_m:.2f}m",
                        (20, 90), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2)

            cv2.putText(frame, f"STATE: {state}",
                (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.imshow("VLandArUco", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                log_event("User pressed 'q' – landing.")
                break

        # If we exited the loop without landing, land now
        if state != "LANDED":
            land_safely(master)

    except KeyboardInterrupt:
        print()
        log_event("KEYBOARD INTERRUPT – commanding immediate LAND")
        try:
            land_safely(master)
        except Exception:
            pass
    finally:
        clear_rc_override(master)
        send_guided_velocity(master, 0.0, 0.0, 0.0)
        cam.close()
        log_event("Script finished.")


if __name__ == "__main__":
    main()
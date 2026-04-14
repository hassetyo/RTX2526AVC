import argparse
import time
from typing import Dict

import cv2
import numpy as np
from pymavlink import mavutil

from ArUcoMovement import (
    CameraInterface,
    ArucoDistanceEstimator,
    UGVCommander,
    draw_crosshair,
    draw_distance_overlay,
    draw_nav_overlay,
    get_dictionary_by_name,
    get_marker_forward_direction_px,
    pick_two_markers,
    signed_angle_deg,
)
from UAVcommander import (
    BAUD_RATE,
    CONNECTION_STRING,
    THROTTLE_HOVER,
    UAVCommander,
)


YARDS_TO_METERS = 0.9144
RC_NEUTRAL = 1500


class UAVMissionController:
    """
    Adds the UAV mission stages on top of the existing ArUcoMovement.py UGV flow.

    Mission stages:
      1) connect + takeoff to target altitude
      2) fly from the assumed bottom-right corner toward the assumed field center
      3) stop early if destination marker is seen
      4) climb slowly until both marker 0 and marker 5 are in frame
      5) hold hover while existing UGV code drives the rover
      6) land straight down when the rover reaches the destination
    """

    def __init__(
        self,
        connection_string: str,
        baud_rate: int,
        target_alt_m: float,
        field_size_yd: float,
        center_speed_mps: float,
        forward_pitch_pwm: int,
        left_roll_pwm: int,
        climb_pwm_delta: int,
        max_search_alt_m: float,
    ):
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.target_alt_m = target_alt_m
        self.field_size_yd = field_size_yd
        self.center_speed_mps = center_speed_mps
        self.forward_pitch_pwm = forward_pitch_pwm
        self.left_roll_pwm = left_roll_pwm
        self.climb_pwm_delta = climb_pwm_delta
        self.max_search_alt_m = max_search_alt_m

        self.commander = UAVCommander()
        self.master = None

        half_field_m = (field_size_yd * YARDS_TO_METERS) / 2.0
        self.center_legs = [
            ["left", half_field_m],
            ["forward", half_field_m],
        ]
        self.current_leg_idx = 0

        self.state = "preflight"
        self.status_text = "preflight"
        self.last_alt_m = None
        self.is_landed = False

    def connect(self):
        self.commander.log_event(f"Connecting to drone on {self.connection_string}...")
        self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baud_rate)
        self.master.wait_heartbeat()
        self.commander.log_event("Drone heartbeat OK.")
        self.commander.request_message_streams(self.master)

    def takeoff(self):
        self.commander.change_mode(self.master, "STABILIZE")
        self.commander.arm_drone(self.master)
        time.sleep(2.0)
        self.commander.wait_for_good_altitude(self.master)
        self.commander.climb_to_target(self.master, self.target_alt_m)
        self.commander.change_mode(self.master, "ALT_HOLD", "ALTHOLD")
        self.state = "move_to_center"
        self.status_text = "moving to assumed field center"

    def _get_altitude(self):
        alt_m, _ = self.commander.get_altitude_m(self.master)
        if alt_m is not None:
            self.last_alt_m = alt_m
        return alt_m

    def _hover_throttle(self) -> int:
        alt_m = self._get_altitude()
        if alt_m is None:
            return THROTTLE_HOVER
        if alt_m < self.target_alt_m - 0.08:
            return THROTTLE_HOVER + 55
        if alt_m > self.target_alt_m + 0.08:
            return THROTTLE_HOVER - 45
        return THROTTLE_HOVER

    def _send_rc(self, roll_pwm: int = RC_NEUTRAL, pitch_pwm: int = RC_NEUTRAL, throttle_pwm: int | None = None):
        if throttle_pwm is None:
            throttle_pwm = self._hover_throttle()
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            roll_pwm,
            pitch_pwm,
            throttle_pwm,
            0, 0, 0, 0, 0,
        )

    def hover_step(self):
        self._send_rc(RC_NEUTRAL, RC_NEUTRAL, self._hover_throttle())

    def stop_motion(self):
        self.hover_step()

    def _step_move_to_center(self, dt: float, dest_visible: bool, both_visible: bool):
        if both_visible:
            self.stop_motion()
            self.state = "ugv_guidance"
            self.status_text = "both markers seen; guiding UGV"
            return

        if dest_visible:
            self.stop_motion()
            self.state = "climb_for_both_markers"
            self.status_text = "destination found; climbing for full view"
            return

        if self.current_leg_idx >= len(self.center_legs):
            self.stop_motion()
            self.state = "hold_at_center"
            self.status_text = "holding at assumed center; scanning"
            return

        leg_name, remaining_m = self.center_legs[self.current_leg_idx]
        if leg_name == "left":
            self._send_rc(roll_pwm=self.left_roll_pwm, pitch_pwm=RC_NEUTRAL)
            self.status_text = "moving left toward assumed center"
        else:
            self._send_rc(roll_pwm=RC_NEUTRAL, pitch_pwm=self.forward_pitch_pwm)
            self.status_text = "moving forward toward assumed center"

        self.center_legs[self.current_leg_idx][1] -= max(dt, 0.0) * self.center_speed_mps

        if self.center_legs[self.current_leg_idx][1] <= 0.0:
            self.stop_motion()
            time.sleep(0.30)
            self.current_leg_idx += 1
            if self.current_leg_idx >= len(self.center_legs):
                self.state = "hold_at_center"
                self.status_text = "assumed center reached; scanning"

    def _step_hold_at_center(self, dest_visible: bool, both_visible: bool):
        if both_visible:
            self.stop_motion()
            self.state = "ugv_guidance"
            self.status_text = "both markers seen; guiding UGV"
            return
        if dest_visible:
            self.stop_motion()
            self.state = "climb_for_both_markers"
            self.status_text = "destination found; climbing for full view"
            return
        self.hover_step()
        self.status_text = "holding at assumed center; scanning"

    def _step_climb_for_both_markers(self, both_visible: bool):
        if both_visible:
            self.stop_motion()
            self.state = "ugv_guidance"
            self.status_text = "both markers seen; guiding UGV"
            return

        alt_m = self._get_altitude()
        if alt_m is not None and alt_m >= self.max_search_alt_m:
            self.hover_step()
            self.status_text = "max climb reached; holding for marker view"
            return

        climb_pwm = min(self._hover_throttle() + self.climb_pwm_delta, 1700)
        self._send_rc(RC_NEUTRAL, RC_NEUTRAL, climb_pwm)
        self.status_text = "climbing slowly for both markers"

    def step(self, dt: float, dest_visible: bool, both_visible: bool):
        if self.state in ("landing", "complete"):
            return

        if self.state == "move_to_center":
            self._step_move_to_center(dt, dest_visible, both_visible)
        elif self.state == "hold_at_center":
            self._step_hold_at_center(dest_visible, both_visible)
        elif self.state == "climb_for_both_markers":
            self._step_climb_for_both_markers(both_visible)
        elif self.state == "ugv_guidance":
            self.hover_step()
            self.status_text = "hovering while UGV moves"
        else:
            self.hover_step()

    def land_now(self):
        if self.is_landed:
            return
        self.state = "landing"
        self.status_text = "landing"
        self.commander.land_safely(self.master)
        self.is_landed = True
        self.state = "complete"
        self.status_text = "mission complete"

    def close(self):
        if self.master is None:
            return
        try:
            self.commander.clear_rc_override(self.master)
        except Exception:
            pass


def draw_uav_overlay(frame, uav: UAVMissionController):
    alt_text = "unknown" if uav.last_alt_m is None else f"{uav.last_alt_m:.2f} m"
    lines = [
        f"UAV State: {uav.state}",
        f"UAV Alt: {alt_text}",
        f"UAV Status: {uav.status_text}",
    ]

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.65
    thickness = 2
    padding = 10
    line_h = 26

    max_w = 0
    for text in lines:
        (tw, _), _ = cv2.getTextSize(text, font, scale, thickness)
        max_w = max(max_w, tw)

    box_w = max_w + padding * 2
    box_h = len(lines) * line_h + padding * 2

    x0 = frame.shape[1] - box_w - 20
    y0 = 20

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (255, 255, 255), -1)
    frame[:] = cv2.addWeighted(overlay, 0.82, frame, 0.18, 0)

    y = y0 + padding + 18
    for text in lines:
        cv2.putText(frame, text, (x0 + padding, y), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)
        y += line_h


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Take off, move the UAV toward the assumed field center, acquire ArUco marker 0, "
            "climb until markers 5 and 0 are both visible, then reuse the existing ArUcoMovement "
            "UGV navigation flow and land when the UGV arrives."
        )
    )
    parser.add_argument("--use-zed", action="store_true", help="Use Stereolabs ZED / ZED X camera.")
    parser.add_argument("--camera-index", type=int, default=0, help="Standard camera index for laptop/USB webcam.")
    parser.add_argument("--calibration", type=str, default=None, help="YAML calibration file for standard camera with nodes K and D.")
    parser.add_argument("--marker-size", type=float, default=0.254, help="Marker size in meters. Example: 0.254 for 10 inches.")
    parser.add_argument("--ugv-marker-id", type=int, default=5, help="ArUco ID attached to the ground vehicle.")
    parser.add_argument("--dest-marker-id", type=int, default=0, help="Destination ArUco ID.")
    parser.add_argument("--dict", type=str, default="DICT_6X6_1000", help="ArUco dictionary name.")
    parser.add_argument("--width", type=int, default=1280, help="Standard camera width.")
    parser.add_argument("--height", type=int, default=720, help="Standard camera height.")
    parser.add_argument("--fps", type=int, default=60, help="Camera FPS.")

    parser.add_argument("--bridge-port", type=str, default="/dev/ttyUSB0", help="Local ESP32 bridge serial port used to send commands.")
    parser.add_argument("--bridge-baud", type=int, default=115200, help="ESP32 bridge baud rate.")

    parser.add_argument("--turn-threshold-deg", type=float, default=12.0, help="Heading error magnitude required before issuing turn commands.")
    parser.add_argument("--stop-distance-m", type=float, default=0.28, help="Distance at which the UGV is considered to have reached the destination.")
    parser.add_argument("--step-min-m", type=float, default=0.20, help="Minimum forward body step to command.")
    parser.add_argument("--step-max-m", type=float, default=0.60, help="Maximum forward body step to command.")
    parser.add_argument("--drive-speed-mps", type=float, default=1.5, help="Expected ground-station forward speed used only for cooldown timing.")
    parser.add_argument("--marker-timeout-sec", type=float, default=0.75, help="Stop the UGV if markers disappear longer than this timeout.")
    parser.add_argument(
        "--ugv-forward-axis",
        choices=["+x", "-x", "+y", "-y"],
        default="+y",
        help="Which marker local axis points in the UGV forward direction.",
    )

    parser.add_argument("--uav-connection-string", type=str, default=CONNECTION_STRING, help="UAV MAVLink connection string.")
    parser.add_argument("--uav-baud", type=int, default=BAUD_RATE, help="UAV serial baud rate.")
    parser.add_argument("--target-alt-m", type=float, default=1.3, help="Takeoff / hover altitude in meters.")
    parser.add_argument("--field-size-yd", type=float, default=15.0, help="Field side length in yards.")
    parser.add_argument("--center-speed-mps", type=float, default=0.45, help="Assumed lateral ground speed used for the timed move toward center.")
    parser.add_argument("--forward-pitch-pwm", type=int, default=1450, help="Pitch PWM used for forward motion toward center. Increase above 1500 if your frame moves the wrong way.")
    parser.add_argument("--left-roll-pwm", type=int, default=1450, help="Roll PWM used for left motion toward center. Increase above 1500 if your frame moves the wrong way.")
    parser.add_argument("--climb-pwm-delta", type=int, default=60, help="Additional throttle PWM above hover while climbing for both markers.")
    parser.add_argument("--max-search-alt-m", type=float, default=2.5, help="Safety ceiling while climbing for both markers.")

    return parser.parse_args()


def main():
    args = parse_args()
    dictionary = get_dictionary_by_name(args.dict)

    cam = CameraInterface(
        use_zed=args.use_zed,
        camera_index=args.camera_index,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )

    if not args.use_zed:
        cam.load_standard_calibration(args.calibration)

    estimator = ArucoDistanceEstimator(
        camera_matrix=cam.camera_matrix,
        dist_coeffs=cam.dist_coeffs,
        marker_size_m=args.marker_size,
        dictionary_name=dictionary,
    )

    ugv_commander = UGVCommander(
        bridge_port=args.bridge_port,
        bridge_baud=args.bridge_baud,
        turn_threshold_deg=args.turn_threshold_deg,
        stop_distance_m=args.stop_distance_m,
        step_min_m=args.step_min_m,
        step_max_m=args.step_max_m,
        drive_speed_mps=args.drive_speed_mps,
        marker_timeout_sec=args.marker_timeout_sec,
    )
    ugv_commander.connect()

    uav = UAVMissionController(
        connection_string=args.uav_connection_string,
        baud_rate=args.uav_baud,
        target_alt_m=args.target_alt_m,
        field_size_yd=args.field_size_yd,
        center_speed_mps=args.center_speed_mps,
        forward_pitch_pwm=args.forward_pitch_pwm,
        left_roll_pwm=args.left_roll_pwm,
        climb_pwm_delta=args.climb_pwm_delta,
        max_search_alt_m=args.max_search_alt_m,
    )
    uav.connect()
    uav.takeoff()

    print("Press q to quit.")
    print("Press s to save a screenshot.")
    print(
        f"UAV mission active. Tracking UGV marker {args.ugv_marker_id} toward destination marker {args.dest_marker_id}."
    )

    last_loop_time = time.time()

    try:
        while True:
            now = time.time()
            dt = now - last_loop_time
            last_loop_time = now

            frame = cam.get_frame()
            if frame is None:
                print("Failed to read frame.")
                uav.step(dt, False, False)
                if uav.state == "ugv_guidance":
                    ugv_commander.handle_marker_loss()
                continue

            poses: Dict[int, object] = estimator.detect_markers(frame)
            display = frame.copy()

            draw_crosshair(display)
            estimator.draw_markers(display, poses)

            dest_visible = args.dest_marker_id in poses
            both_visible = args.dest_marker_id in poses and args.ugv_marker_id in poses

            uav.step(dt, dest_visible, both_visible)

            if uav.state == "ugv_guidance":
                pair = pick_two_markers(poses, args.ugv_marker_id, args.dest_marker_id)
                if pair is not None:
                    ugv_pose, dest_pose = pair
                    ugv_commander.last_seen_time = time.time()
                    ugv_commander.ensure_armed()

                    draw_distance_overlay(display, ugv_pose, dest_pose)

                    forward_dir = get_marker_forward_direction_px(
                        ugv_pose,
                        cam.camera_matrix,
                        cam.dist_coeffs,
                        args.marker_size,
                        args.ugv_forward_axis,
                    )
                    target_dir = np.array(
                        [
                            dest_pose.center_px[0] - ugv_pose.center_px[0],
                            dest_pose.center_px[1] - ugv_pose.center_px[1],
                        ],
                        dtype=np.float64,
                    )
                    heading_error_deg = signed_angle_deg(
                        forward_dir if forward_dir is not None else target_dir,
                        target_dir,
                    )
                    dist_m = float(np.linalg.norm(dest_pose.tvec.reshape(3) - ugv_pose.tvec.reshape(3)))

                    if dist_m <= args.stop_distance_m:
                        ugv_commander.send_stop()
                        ugv_commander.last_status_text = "destination reached"
                        draw_nav_overlay(
                            display,
                            ugv_pose,
                            dest_pose,
                            heading_error_deg,
                            ugv_commander.last_status_text,
                            args.stop_distance_m,
                            cam.camera_matrix,
                            cam.dist_coeffs,
                            args.marker_size,
                            args.ugv_forward_axis,
                        )
                        draw_uav_overlay(display, uav)
                        cv2.imshow("ArUco Distance + UAV + UGV Navigation", display)
                        cv2.waitKey(1)
                        uav.land_now()
                        break
                    elif abs(heading_error_deg) > args.turn_threshold_deg:
                        if heading_error_deg > 0.0:
                            ugv_commander.send_turn_right()
                        else:
                            ugv_commander.send_turn_left()
                    else:
                        if ugv_commander.last_motion in ("turn_left", "turn_right"):
                            ugv_commander.send_stop()
                            time.sleep(2.0)

                        remaining = max(0.0, dist_m - args.stop_distance_m)
                        step_m = max(args.step_min_m, min(remaining * 0.5, args.step_max_m))
                        ugv_commander.send_forward_step(step_m)

                    draw_nav_overlay(
                        display,
                        ugv_pose,
                        dest_pose,
                        heading_error_deg,
                        ugv_commander.last_status_text,
                        args.stop_distance_m,
                        cam.camera_matrix,
                        cam.dist_coeffs,
                        args.marker_size,
                        args.ugv_forward_axis,
                    )
                else:
                    ugv_commander.handle_marker_loss()
                    cv2.putText(
                        display,
                        f"Need markers {args.ugv_marker_id} and {args.dest_marker_id}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        display,
                        f"State: {ugv_commander.last_status_text}",
                        (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
            else:
                pair = pick_two_markers(poses, args.ugv_marker_id, args.dest_marker_id)
                if pair is not None:
                    draw_distance_overlay(display, pair[0], pair[1])
                else:
                    if not both_visible:
                        cv2.putText(
                            display,
                            f"UAV phase: {uav.status_text}",
                            (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 255),
                            2,
                            cv2.LINE_AA,
                        )
                        cv2.putText(
                            display,
                            f"Need marker {args.dest_marker_id} or both markers to start UGV guidance",
                            (20, 78),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 0, 255),
                            2,
                            cv2.LINE_AA,
                        )

            draw_uav_overlay(display, uav)
            cv2.imshow("ArUco Distance + UAV + UGV Navigation", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                cv2.imwrite("aruco_uav_ugv_mission_screenshot.png", display)
                print("Saved: aruco_uav_ugv_mission_screenshot.png")

    finally:
        try:
            ugv_commander.send_stop(force=True)
        except Exception:
            pass

        try:
            if not uav.is_landed:
                uav.land_now()
        except Exception:
            pass

        ugv_commander.close()
        uav.close()
        cam.close()


if __name__ == "__main__":
    main()

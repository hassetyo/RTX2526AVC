'''Will tell the drone to move to the center of the 15x15 yard grid and rise until both aruco markers are visible for a set amount of time.'''

import argparse
import math
import time
import cv2
import numpy as np
import cv2.aruco as aruco

# Import our custom modules
from camera_utils import CameraInterface
from aruco_utils import ArucoDistanceEstimator, MarkerPose
from commander import UGVCommander

FORWARD_AXIS_MAP = {
    "+x": np.array([1.0, 0.0, 0.0], dtype=np.float64),
    "-x": np.array([-1.0, 0.0, 0.0], dtype=np.float64),
    "+y": np.array([0.0, 1.0, 0.0], dtype=np.float64),
    "-y": np.array([0.0, -1.0, 0.0], dtype=np.float64),
}

def get_marker_forward_direction_px(pose, camera_matrix, dist_coeffs, marker_size_m, forward_axis_name):
    axis = FORWARD_AXIS_MAP[forward_axis_name]
    tip_local = axis * (marker_size_m * 0.75)
    pts3d = np.array([[0.0, 0.0, 0.0], tip_local], dtype=np.float32)

    img_pts, _ = cv2.projectPoints(pts3d, pose.rvec, pose.tvec.reshape(3, 1), camera_matrix, dist_coeffs)
    img_pts = img_pts.reshape(-1, 2)
    direction = img_pts[1] - img_pts[0]
    return direction if np.linalg.norm(direction) > 1e-6 else None

def signed_angle_deg(vec_a, vec_b):
    a, b = vec_a.reshape(2), vec_b.reshape(2)
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9: return 0.0
    a, b = a/na, b/nb
    return math.degrees(math.atan2(a[0]*b[1] - a[1]*b[0], np.dot(a, b)))

def draw_nav_overlay(frame, ugv_pose, dest_pose, heading_error_deg, status_text, stop_distance_m, cam_m, dist_c, m_size, axis_name):
    ugv_center = np.array(ugv_pose.center_px, dtype=np.int32)
    dst_center = np.array(dest_pose.center_px, dtype=np.int32)
    
    # Draw Navigation Arrow
    cv2.arrowedLine(frame, tuple(ugv_center), tuple(dst_center), (0, 255, 255), 2)
    
    dist_m = float(np.linalg.norm(dest_pose.tvec - ugv_pose.tvec))
    lines = [
        f"Heading error: {heading_error_deg:+.1f} deg",
        f"Distance: {dist_m:.3f} m",
        f"State: {status_text}",
    ]
    
    y = 40
    for text in lines:
        cv2.putText(frame, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        y += 30

def parse_args():
    parser = argparse.ArgumentParser(description="Jetson Nano UGV Nav")
    parser.add_argument("--use-zed", action="store_true")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--calibration", type=str, default=None)
    parser.add_argument("--marker-size", type=float, default=0.254)
    parser.add_argument("--ugv-marker-id", type=int, default=5)
    parser.add_argument("--dest-marker-id", type=int, default=0)
    parser.add_argument("--dict", type=str, default="DICT_6X6_1000")
    parser.add_argument("--bridge-port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--bridge-baud", type=int, default=115200)
    parser.add_argument("--turn-threshold-deg", type=float, default=12.0)
    parser.add_argument("--stop-distance-m", type=float, default=0.28)
    parser.add_argument("--step-min-m", type=float, default=0.20)
    parser.add_argument("--step-max-m", type=float, default=0.60)
    parser.add_argument("--drive-speed-mps", type=float, default=1.5)
    parser.add_argument("--marker-timeout-sec", type=float, default=0.75)
    parser.add_argument("--ugv-forward-axis", choices=["+x", "-x", "+y", "-y"], default="+y")
    return parser.parse_args()

def main():
    args = parse_args()
    aruco_dict_id = getattr(aruco, args.dict)

    # Initialize Components
    cam = CameraInterface(use_zed=args.use_zed, camera_index=args.camera_index)
    if not args.use_zed:
        cam.load_standard_calibration(args.calibration)

    estimator = ArucoDistanceEstimator(cam.camera_matrix, cam.dist_coeffs, args.marker_size, aruco_dict_id)
    commander = UGVCommander(args.bridge_port, args.bridge_baud, args.turn_threshold_deg, 
                             args.stop_distance_m, args.step_min_m, args.step_max_m, 
                             args.drive_speed_mps, args.marker_timeout_sec)
    commander.connect()

    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                commander.send_stop()
                continue

            poses = estimator.detect_markers(frame)
            display = frame.copy()
            estimator.draw_markers(display, poses)

            # Navigation Logic
            if args.ugv_marker_id in poses and args.dest_marker_id in poses:
                u_p, d_p = poses[args.ugv_marker_id], poses[args.dest_marker_id]
                commander.last_seen_time = time.time()
                commander.ensure_armed()

                fwd_dir = get_marker_forward_direction_px(u_p, cam.camera_matrix, cam.dist_coeffs, args.marker_size, args.ugv_forward_axis)
                target_dir = np.array([d_p.center_px[0] - u_p.center_px[0], d_p.center_px[1] - u_p.center_px[1]], dtype=np.float64)
                
                heading_err = signed_angle_deg(fwd_dir if fwd_dir is not None else target_dir, target_dir)
                dist_m = float(np.linalg.norm(d_p.tvec - u_p.tvec))

                if dist_m <= args.stop_distance_m:
                    commander.send_stop()
                    commander.last_status_text = "Reached"
                elif abs(heading_err) > args.turn_threshold_deg:
                    commander.send_turn("right" if heading_err > 0 else "left")
                else:
                    remaining = max(0.0, dist_m - args.stop_distance_m)
                    commander.send_forward_step(max(args.step_min_m, min(remaining * 0.5, args.step_max_m)))

                draw_nav_overlay(display, u_p, d_p, heading_err, commander.last_status_text, args.stop_distance_m, 
                                 cam.camera_matrix, cam.dist_coeffs, args.marker_size, args.ugv_forward_axis)
            else:
                commander.send_stop()

            cv2.imshow("Jetson Nav", display)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        commander.close()
        cam.close()

if __name__ == "__main__":
    main()
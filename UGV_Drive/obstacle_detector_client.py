"""
obstacle_detector_client.py

- Uses webcam + YOLOv8 obstacle detection (center corridor heuristic)
- Sends a TCP message to rover script when obstacle detected
- Handshake:
    Detector -> "OBSTACLE <name>\n"
    Rover    -> "ACK\n"
    Rover    -> "DONE\n"
- Detector PAUSES inference after ACK, and resumes after DONE.
- Includes a "clear-frames" lockout to avoid immediate re-trigger.

Run (webcam):
  python obstacle_detector_client.py --source webcam --webcam-index 0 --rover-host 127.0.0.1 --rover-port 5005

Press 'q' to quit.
"""

import argparse
import time
import socket
import cv2

# -----------------------------
# Camera Abstraction
# -----------------------------

class Camera:
    def __init__(self, source: str, webcam_index: int = 0, width: int = 1280, height: int = 720):
        self.source = source.lower()
        self.width = width
        self.height = height

        self.zed = None
        self.cap = None

        if self.source == "zed":
            self._init_zed()
        elif self.source == "webcam":
            self._init_webcam(webcam_index)
        else:
            raise ValueError("source must be 'zed' or 'webcam'")

    def _init_zed(self):
        print("[INFO] Initializing ZED camera...")
        import pyzed.sl as sl
        self.sl = sl

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.depth_mode = sl.DEPTH_MODE.NONE

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {status}")

        self.zed_image = sl.Mat()
        print("[INFO] ZED camera ready.")

    def _init_webcam(self, webcam_index: int):
        print("[INFO] Initializing webcam...")
        self.cap = cv2.VideoCapture(webcam_index)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open webcam (VideoCapture).")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        print("[INFO] Webcam ready.")

    def read(self):
        if self.source == "zed":
            if self.zed.grab() != self.sl.ERROR_CODE.SUCCESS:
                return None
            self.zed.retrieve_image(self.zed_image, self.sl.VIEW.LEFT)
            frame = self.zed_image.get_data()  # BGRA
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            return frame

        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def close(self):
        if self.zed is not None:
            self.zed.close()
        if self.cap is not None:
            self.cap.release()

# -----------------------------
# YOLOv8 Detector
# -----------------------------

class YOLODetector:
    def __init__(self, model_name: str = "yolov8n.pt", conf: float = 0.35):
        from ultralytics import YOLO
        print(f"[INFO] Loading model: {model_name}")
        self.model = YOLO(model_name)
        self.conf = conf

    def detect(self, frame_bgr):
        results = self.model.predict(frame_bgr, conf=self.conf, verbose=False)

        dets = []
        r0 = results[0]
        if r0.boxes is None:
            return dets

        names = r0.names
        for box in r0.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            dets.append({
                "cls_name": names.get(cls_id, str(cls_id)),
                "conf": conf,
                "x1": int(x1), "y1": int(y1), "x2": int(x2), "y2": int(y2)
            })
        return dets

# -----------------------------
# Obstacle Logic
# -----------------------------

def obstacle_in_path(detections, frame_shape, corridor_width_ratio=0.35, min_area_ratio=0.02):
    h, w = frame_shape[:2]
    img_area = w * h

    corridor_w = int(w * corridor_width_ratio)
    cx1 = (w - corridor_w) // 2
    cx2 = cx1 + corridor_w

    for d in detections:
        x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]

        bbox_area = max(0, x2 - x1) * max(0, y2 - y1)
        if bbox_area < img_area * min_area_ratio:
            continue

        overlap_x1 = max(x1, cx1)
        overlap_x2 = min(x2, cx2)
        overlap_w = max(0, overlap_x2 - overlap_x1)

        if overlap_w > 0:
            return True, (cx1, cx2), d["cls_name"]

    return False, (cx1, cx2), None


def draw_overlay(frame, detections, is_obstacle, corridor_bounds, paused=False):
    h, w = frame.shape[:2]
    cx1, cx2 = corridor_bounds

    cv2.rectangle(frame, (cx1, 0), (cx2, h), (255, 255, 255), 2)

    for d in detections:
        x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
        label = f"{d['cls_name']} {d['conf']:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(frame, label, (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    status = "PAUSED" if paused else ("OBSTACLE" if is_obstacle else "CLEAR")
    cv2.putText(frame, f"STATUS: {status}", (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
    return frame

# -----------------------------
# TCP Client Helpers
# -----------------------------

def tcp_connect(host: str, port: int, timeout_s: float = 5.0) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout_s)
    s.connect((host, port))
    s.settimeout(None)  # blocking afterwards
    return s

def recv_line(sock: socket.socket) -> str:
    data = b""
    while True:
        bch = sock.recv(1)
        if not bch:
            raise ConnectionError("Socket closed")
        if bch == b"\n":
            return data.decode("utf-8", errors="replace").strip()
        data += bch

def send_line(sock: socket.socket, line: str):
    if not line.endswith("\n"):
        line += "\n"
    sock.sendall(line.encode("utf-8"))

# -----------------------------
# Main
# -----------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", choices=["zed", "webcam"], default="webcam")
    parser.add_argument("--webcam-index", type=int, default=0)
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--corridor-width", type=float, default=0.35)
    parser.add_argument("--min-area", type=float, default=0.02)
    parser.add_argument("--maxfps", type=float, default=15.0)

    parser.add_argument("--rover-host", default="127.0.0.1")
    parser.add_argument("--rover-port", type=int, default=5005)

    # Avoid immediate retrigger: require N consecutive "CLEAR" frames after DONE
    parser.add_argument("--clear-frames", type=int, default=12)

    args = parser.parse_args()

    print(f"[NET] Connecting to rover server at {args.rover_host}:{args.rover_port} ...")
    sock = tcp_connect(args.rover_host, args.rover_port)
    print("[NET] Connected.")

    cam = Camera(source=args.source, webcam_index=args.webcam_index)
    det = YOLODetector(model_name=args.model, conf=args.conf)

    cv2.namedWindow("Obstacle Detection", cv2.WINDOW_NORMAL)

    last_time = 0.0
    frame_interval = 1.0 / max(1e-6, args.maxfps)

    paused = False
    need_clear = 0  # how many clear frames still required before re-arming trigger

    try:
        while True:
            # FPS limiting
            now = time.time()
            if now - last_time < frame_interval:
                time.sleep(0.001)
                continue
            last_time = now

            frame = cam.read()
            if frame is None:
                print("[WARN] No frame received.")
                continue

            if paused:
                # While paused, we still show video, but do not run YOLO inference
                out = draw_overlay(frame, [], is_obstacle=False, corridor_bounds=(0, 0), paused=True)
                cv2.imshow("Obstacle Detection", out)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            detections = det.detect(frame)
            is_obs, corridor_bounds, obstacle_name = obstacle_in_path(
                detections, frame.shape,
                corridor_width_ratio=args.corridor_width,
                min_area_ratio=args.min_area
            )

            # Manage re-arm / clear frames lockout
            if need_clear > 0:
                if not is_obs:
                    need_clear -= 1
                # Even if obstacle is still there, we do not trigger until cleared enough
                is_trigger_allowed = (need_clear <= 0)
            else:
                is_trigger_allowed = True

            out = draw_overlay(frame, detections, is_obs, corridor_bounds, paused=False)
            cv2.imshow("Obstacle Detection", out)

            if is_obs and is_trigger_allowed:
                print(f"[DETECT] OBSTACLE: {obstacle_name}. Sending to rover...")

                # Send obstacle message
                send_line(sock, f"OBSTACLE {obstacle_name}")

                # Wait for ACK
                msg = recv_line(sock)
                if msg != "ACK":
                    print(f"[NET] Unexpected reply (expected ACK): {msg}")
                else:
                    print("[NET] Rover ACK received. Pausing detection until DONE...")

                # Pause inference while rover maneuvers
                paused = True

                # Wait for DONE
                done = recv_line(sock)
                print(f"[NET] Rover says: {done}")

                # Resume detection + require several CLEAR frames before triggering again
                paused = False
                need_clear = args.clear_frames

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        try:
            sock.close()
        except Exception:
            pass
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
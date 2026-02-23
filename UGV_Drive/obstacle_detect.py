"""
obstacle_detect.py

- Supports:
  - ZED camera (pyzed.sl)
  - Standard webcam / Windows camera (OpenCV VideoCapture)
- Runs YOLOv8 object detection
- Declares "OBSTACLE" if a detected object is in the center corridor (path)
  and meets confidence + size thresholds.

Run examples:
  Webcam:
    python obstacle_detect.py --source webcam

  ZED:
    python obstacle_detect.py --source zed

Press 'q' to quit.
"""

import argparse
import time
import cv2
import numpy as np

# -----------------------------
# Camera Abstraction
# -----------------------------

class Camera:
    """
    A unified camera wrapper:
      - "zed": Stereolabs ZED (pyzed.sl)
      - "webcam": OpenCV VideoCapture (Windows/USB camera)
    """

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
        init_params.camera_resolution = sl.RESOLUTION.HD720  # good default
        init_params.depth_mode = sl.DEPTH_MODE.NONE          # we aren't using depth here

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {status}")

        # Pre-allocate container for images
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
        """
        Returns: frame (numpy array, BGR)
        """
        if self.source == "zed":
            if self.zed.grab() != self.sl.ERROR_CODE.SUCCESS:
                return None
            self.zed.retrieve_image(self.zed_image, self.sl.VIEW.LEFT)
            frame = self.zed_image.get_data()  # typically BGRA
            # Convert BGRA -> BGR for OpenCV + YOLO pipelines
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            return frame

        # webcam
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
    """
    Wraps Ultralytics YOLO inference.
    """
    def __init__(self, model_name: str = "yolov8n.pt", conf: float = 0.35):
        from ultralytics import YOLO
        print(f"[INFO] Loading model: {model_name}")
        self.model = YOLO(model_name)
        self.conf = conf

    def detect(self, frame_bgr):
        """
        Returns a list of detections:
          each detection = dict with keys:
            - cls_name
            - conf
            - x1,y1,x2,y2  (ints)
        """
        # Ultralytics accepts numpy images (BGR is fine)
        results = self.model.predict(frame_bgr, conf=self.conf, verbose=False)

        dets = []
        r0 = results[0]
        if r0.boxes is None:
            return dets

        names = r0.names  # class index -> class name
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
    """
    Decide if there is an obstacle "in front of the drone".

    Heuristic:
      - Define a vertical corridor in the center of the image (the "path")
      - If any detection overlaps that corridor AND is large enough, call it an obstacle

    corridor_width_ratio: fraction of image width that counts as the path corridor
    min_area_ratio: minimum bbox area relative to image area to count as a real obstruction
    """
    h, w = frame_shape[:2]
    img_area = w * h

    corridor_w = int(w * corridor_width_ratio)
    cx1 = (w - corridor_w) // 2
    cx2 = cx1 + corridor_w

    for d in detections:
        x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]

        # bbox area check
        bbox_area = max(0, x2 - x1) * max(0, y2 - y1)
        if bbox_area < img_area * min_area_ratio:
            continue

        # overlap with corridor (any horizontal overlap counts)
        overlap_x1 = max(x1, cx1)
        overlap_x2 = min(x2, cx2)
        overlap_w = max(0, overlap_x2 - overlap_x1)

        if overlap_w > 0:
            return True, (cx1, cx2)  # obstacle present, return corridor bounds

    return False, (cx1, cx2)


def draw_overlay(frame, detections, is_obstacle, corridor_bounds):
    """
    Draw bounding boxes + obstacle status + corridor visualization.
    """
    h, w = frame.shape[:2]
    cx1, cx2 = corridor_bounds

    # draw corridor
    cv2.rectangle(frame, (cx1, 0), (cx2, h), (255, 255, 255), 2)

    # draw detections
    for d in detections:
        x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
        label = f"{d['cls_name']} {d['conf']:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(frame, label, (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    status = "OBSTACLE" if is_obstacle else "CLEAR"
    cv2.putText(frame, f"STATUS: {status}", (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)

    return frame


# -----------------------------
# Main Loop
# -----------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", choices=["zed", "webcam"], default="webcam",
                        help="Camera source: 'zed' for ZED camera, 'webcam' for Windows/USB camera.")
    parser.add_argument("--webcam-index", type=int, default=0, help="OpenCV camera index for webcam source.")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO model, e.g. yolov8n.pt / yolov8s.pt")
    parser.add_argument("--conf", type=float, default=0.35, help="Min confidence threshold for detections.")
    parser.add_argument("--corridor-width", type=float, default=0.35, help="Center corridor width ratio (0..1).")
    parser.add_argument("--min-area", type=float, default=0.02, help="Min bbox area ratio to count as obstacle.")
    parser.add_argument("--maxfps", type=float, default=15.0, help="Limit processing FPS.")
    args = parser.parse_args()

    cam = Camera(source=args.source, webcam_index=args.webcam_index)
    det = YOLODetector(model_name=args.model, conf=args.conf)

    cv2.namedWindow("Obstacle Detection", cv2.WINDOW_NORMAL)

    last_time = 0.0
    frame_interval = 1.0 / max(1e-6, args.maxfps)

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

            detections = det.detect(frame)

            is_obs, corridor_bounds = obstacle_in_path(
                detections, frame.shape,
                corridor_width_ratio=args.corridor_width,
                min_area_ratio=args.min_area
            )

            out = draw_overlay(frame, detections, is_obs, corridor_bounds)
            cv2.imshow("Obstacle Detection", out)

            # Print a simple console message too (useful for drone logic later)
            if is_obs:
                print("[OBSTACLE] Object detected in path corridor.")

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
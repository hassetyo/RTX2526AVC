import argparse
import cv2

# run with: source .venv/bin/activate
# python3 test_best_camera.py --source webcam --model best.pt
# This script tests the best.pt model (cones, buckets, cardboard boxes)
class Camera:
    def __init__(self, source: str, webcam_index: int = 0, width: int = 1280, height: int = 720):
        self.source = source.lower()
        self.width = width
        self.height = height
        self.cap = None
        self.device = None
        self.q_rgb = None

        if self.source == "oak":
            self._init_oak()
        elif self.source == "webcam":
            self._init_webcam(webcam_index)
        else:
            raise ValueError("source must be 'oak' or 'webcam'")

    def _init_oak(self):
        import depthai as dai

        print("[INFO] Initializing OAK-D-Lite...")
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        xout_rgb = pipeline.create(dai.node.XLinkOut)

        xout_rgb.setStreamName("rgb")
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setPreviewSize(self.width, self.height)
        cam_rgb.setFps(30)

        cam_rgb.preview.link(xout_rgb.input)

        self.device = dai.Device(pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        print("[INFO] OAK-D-Lite ready.")

    def _init_webcam(self, webcam_index: int):
        print("[INFO] Initializing webcam...")
        self.cap = cv2.VideoCapture(webcam_index)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open webcam.")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        print("[INFO] Webcam ready.")

    def read(self):
        if self.source == "oak":
            in_rgb = self.q_rgb.tryGet()
            if in_rgb is None:
                return None
            return in_rgb.getCvFrame()

        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def close(self):
        if self.cap is not None:
            self.cap.release()
        if self.device is not None:
            self.device.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", choices=["oak", "webcam"], default="oak")
    parser.add_argument("--webcam-index", type=int, default=0)
    parser.add_argument("--model", default="best.pt")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    from ultralytics import YOLO

    print(f"[INFO] Loading model: {args.model}")
    model = YOLO(args.model)

    cam = Camera(
        source=args.source,
        webcam_index=args.webcam_index,
        width=args.width,
        height=args.height
    )

    cv2.namedWindow("best.pt Camera Test", cv2.WINDOW_NORMAL)

    try:
        while True:
            frame = cam.read()
            if frame is None:
                continue

            results = model.predict(frame, conf=args.conf, verbose=False)
            annotated = results[0].plot()

            cv2.imshow("best.pt Camera Test", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
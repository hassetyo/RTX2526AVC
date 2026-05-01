import cv2
import depthai as dai
import time

PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 480
CAMERA_FPS = 30


def create_pipeline():
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(PREVIEW_WIDTH, PREVIEW_HEIGHT)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(CAMERA_FPS)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")

    cam_rgb.preview.link(xout_rgb.input)

    return pipeline


def main():
    print("[INFO] Starting OAK-D-Lite camera...")
    print("[INFO] Press q to quit.")

    pipeline = create_pipeline()

    frame_count = 0
    start_time = time.time()
    fps = 0.0

    try:
        with dai.Device(pipeline) as device:
            print("[INFO] OAK-D-Lite connected successfully.")
            print("[INFO] USB speed:", device.getUsbSpeed())

            rgb_queue = device.getOutputQueue(
                name="rgb",
                maxSize=1,
                blocking=False
            )

            while True:
                packet = rgb_queue.tryGet()

                if packet is None:
                    continue

                frame = packet.getCvFrame()

                frame_count += 1
                now = time.time()

                if now - start_time >= 1.0:
                    fps = frame_count / (now - start_time)
                    frame_count = 0
                    start_time = now
                    print(f"[INFO] FPS: {fps:.2f}")

                cv2.putText(
                    frame,
                    f"FPS: {fps:.2f}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2
                )

                cv2.imshow("OAK-D-Lite RGB Camera", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("[INFO] Stopped by user.")

    except RuntimeError as e:
        print("[ERROR] Could not connect to OAK-D-Lite.")
        print(e)

    finally:
        cv2.destroyAllWindows()
        print("[INFO] Camera viewer closed.")


if __name__ == "__main__":
    main()
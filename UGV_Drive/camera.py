import cv2
import depthai as dai
import time

# ==========================================
# OAK-D-Lite Camera Viewer for Raspberry Pi
# UGV
#
# This script:
#   1. Opens the OAK-D-Lite camera
#   2. Shows the RGB camera feed
#   3. Press q to quit
# ==========================================


def create_pipeline():
    pipeline = dai.Pipeline()

    # Create RGB camera node
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(30)

    # Create output stream
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")

    # Link camera preview to output
    cam_rgb.preview.link(xout_rgb.input)

    return pipeline


def main():
    print("==========================================")
    print("        OAK-D-Lite Camera Viewer")
    print("==========================================")
    print("[INFO] Starting OAK-D-Lite camera...")
    print("[INFO] Press 'q' to quit.")

    pipeline = create_pipeline()

    try:
        with dai.Device(pipeline) as device:
            print("[INFO] OAK-D-Lite connected successfully.")

            rgb_queue = device.getOutputQueue(
                name="rgb",
                maxSize=4,
                blocking=False
            )

            while True:
                rgb_frame = rgb_queue.get()
                frame = rgb_frame.getCvFrame()

                cv2.imshow("OAK-D-Lite RGB Camera", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[INFO] Quit key pressed.")
                    break

    except RuntimeError as e:
        print("[ERROR] Could not connect to OAK-D-Lite.")
        print("[ERROR]", e)
        print()
        print("Try these checks:")
        print("1. Make sure the OAK-D-Lite is plugged into the Raspberry Pi.")
        print("2. Try a different USB-C cable.")
        print("3. Make sure depthai is installed:")
        print("   pip install depthai opencv-python")
        print("4. Run:")
        print("   python -c \"import depthai as dai; print(dai.Device.getAllAvailableDevices())\"")

    finally:
        cv2.destroyAllWindows()
        print("[INFO] Camera viewer closed.")


if __name__ == "__main__":
    main()
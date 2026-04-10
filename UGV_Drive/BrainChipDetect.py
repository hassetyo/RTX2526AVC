import cv2
import numpy as np
import depthai as dai
from akida import Model, devices

# This code runs obstacle detection on UGV using Oak D Lite camera and BrainChip AKD1000 M.2 Card
MODEL_PATH = "/home/raspberry/Desktop/UGVCode/akida_model.fbz"
INPUT_W = 224
INPUT_H = 224

# FOMO-style output: [grid_h, grid_w, num_classes]
# Channel 0 is usually background, then your object classes
CLASS_NAMES = ["background", "box", "bucket", "cone"]

# Detection settings
CONF_THRESH = 0.60
CENTER_REGION_ONLY = True          # only treat detections in center path as obstacles
CENTER_REGION_FRAC = 0.33          # middle third of image
MIN_CONSECUTIVE_FRAMES = 2         # require persistence before declaring obstacle

# -----------------------------
# Load and map Akida model
# -----------------------------
model = Model(MODEL_PATH)
devs = devices()
if not devs:
    raise RuntimeError("No Akida device found")

akida_device = devs[0]
print("Using Akida device:", akida_device)
model.map(akida_device)
print("Model mapped OK")

# Get output grid shape once
dummy = np.zeros((1, INPUT_H, INPUT_W, 3), dtype=np.uint8)
dummy_out = model.predict(dummy)[0]
GRID_H, GRID_W, NUM_CLASSES = dummy_out.shape
print(f"Model output grid: {GRID_H}x{GRID_W}x{NUM_CLASSES}")

if NUM_CLASSES != len(CLASS_NAMES):
    print("[WARN] NUM_CLASSES does not match CLASS_NAMES length.")
    print(f"[WARN] Model reports {NUM_CLASSES} channels, CLASS_NAMES has {len(CLASS_NAMES)} names.")

# -----------------------------
# DepthAI pipeline
# -----------------------------
with dai.Pipeline() as pipeline:
    camera = pipeline.create(dai.node.Camera).build()

    output = camera.requestOutput(
        size=(640, 480),
        type=dai.ImgFrame.Type.BGR888p,
        resizeMode=dai.ImgResizeMode.CROP,
        fps=15
    )

    output_q = output.createOutputQueue()

    pipeline.start()

    print("Press q to quit")

    consecutive_obstacle_frames = 0

    while pipeline.isRunning():
        frame = output_q.get().getCvFrame()
        display = frame.copy()
        frame_h, frame_w = display.shape[:2]

        # Define center obstacle region
        center_x1 = int(frame_w * (0.5 - CENTER_REGION_FRAC / 2))
        center_x2 = int(frame_w * (0.5 + CENTER_REGION_FRAC / 2))

        if CENTER_REGION_ONLY:
            cv2.rectangle(display, (center_x1, 0), (center_x2, frame_h), (255, 0, 0), 2)
            cv2.putText(display, "CENTER PATH", (center_x1 + 5, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # Preprocess for model
        resized = cv2.resize(frame, (INPUT_W, INPUT_H))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        x = np.expand_dims(rgb, axis=0)

        # Run BrainChip inference
        out = model.predict(x)[0]   # shape [GRID_H, GRID_W, NUM_CLASSES]

        cell_w = INPUT_W / GRID_W
        cell_h = INPUT_H / GRID_H

        obstacle_in_path = False
        detections = []

        for gy in range(GRID_H):
            for gx in range(GRID_W):
                scores = out[gy, gx]

                cls = int(np.argmax(scores))
                conf = float(scores[cls])

                # Skip background / low confidence
                if cls == 0 or conf < CONF_THRESH:
                    continue

                # Cell box in model-input coordinates
                x1 = int(gx * cell_w)
                y1 = int(gy * cell_h)
                x2 = int((gx + 1) * cell_w)
                y2 = int((gy + 1) * cell_h)

                # Scale back to display frame
                sx1 = int(x1 * frame_w / INPUT_W)
                sy1 = int(y1 * frame_h / INPUT_H)
                sx2 = int(x2 * frame_w / INPUT_W)
                sy2 = int(y2 * frame_h / INPUT_H)

                # Cell center
                cx = (sx1 + sx2) // 2
                cy = (sy1 + sy2) // 2

                label = CLASS_NAMES[cls] if cls < len(CLASS_NAMES) else f"class_{cls}"

                detections.append({
                    "label": label,
                    "conf": conf,
                    "box": (sx1, sy1, sx2, sy2),
                    "center": (cx, cy)
                })

                # Check if detection is in rover path
                in_center = center_x1 <= cx <= center_x2 if CENTER_REGION_ONLY else True
                if in_center:
                    obstacle_in_path = True

        # Persistence filter
        if obstacle_in_path:
            consecutive_obstacle_frames += 1
        else:
            consecutive_obstacle_frames = 0

        confirmed_obstacle = consecutive_obstacle_frames >= MIN_CONSECUTIVE_FRAMES

        # Draw detections
        for det in detections:
            sx1, sy1, sx2, sy2 = det["box"]
            cx, cy = det["center"]
            label = det["label"]
            conf = det["conf"]

            in_center = center_x1 <= cx <= center_x2 if CENTER_REGION_ONLY else True
            color = (0, 0, 255) if in_center else (0, 255, 0)

            cv2.rectangle(display, (sx1, sy1), (sx2, sy2), color, 2)
            cv2.circle(display, (cx, cy), 3, color, -1)
            cv2.putText(display, f"{label} {conf:.2f}", (sx1, max(20, sy1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Status text
        if confirmed_obstacle:
            status_text = "OBSTACLE IN PATH"
            status_color = (0, 0, 255)
        elif obstacle_in_path:
            status_text = "POSSIBLE OBSTACLE"
            status_color = (0, 165, 255)
        else:
            status_text = "CLEAR"
            status_color = (0, 255, 0)

        cv2.putText(display, status_text, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)

        cv2.putText(display, f"Consecutive obstacle frames: {consecutive_obstacle_frames}",
                    (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

        # Console message for rover integration
        if confirmed_obstacle:
            print("STOP: obstacle detected in center path")
        else:
            print("CLEAR")

        cv2.imshow("OAK + Akida detections", display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()


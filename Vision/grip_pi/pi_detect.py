from ultralytics import YOLO
import cv2
import time
import json

# ── config ────────────────────────────────────────────────────────────
MODEL_PATH   = "/home/pi/GRIP/best.onnx"
CONF         = 0.50
CAMERA_ID    = 0          # USB webcam = 0, try 1 if wrong camera
USE_PI_CAM   = True      # set True if using Pi Camera Module 3
BELT_SPEED   = 250        # mm/s
PX_PER_MM    = 3.84       # your calibrated value — update this
SHOW_DISPLAY = True       # set False if running headless (no monitor)
# ──────────────────────────────────────────────────────────────────────

def open_camera(camera_id, use_pi_cam):
    if use_pi_cam:
        cap = cv2.VideoCapture(
            "libcamerasrc ! video/x-raw,width=1280,height=720,"
            "framerate=30/1 ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER
        )
    else:
        cap = cv2.VideoCapture(camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap

if __name__ == '__main__':
    print("🔧 Loading model...")
    model = YOLO(MODEL_PATH, task='detect')
    print("✅ Model loaded")

    cap = open_camera(CAMERA_ID, USE_PI_CAM)

    if not cap.isOpened():
        print("❌ Camera not found. Check CAMERA_ID or USE_PI_CAM setting")
        exit()

    print("✅ Camera opened")
    print("   GREEN = left_glove  (correct)")
    print("   RED   = right_glove (SCARA removes)")
    print("   Press Q to quit")

    fps_start   = time.time()
    fps_display = 0
    frame_count = 0
    start_time  = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Frame read failed")
            break

        # ── inference ─────────────────────────────────────────────
        results = model(frame, conf=CONF, verbose=False)[0]

        # ── belt position estimate ─────────────────────────────────
        elapsed_ms  = (time.time() - start_time) * 1000
        belt_pos_mm = (elapsed_ms / 1000) * BELT_SPEED

        # ── process detections ────────────────────────────────────
        remove_count = 0

        for box in results.boxes:
            cls_id     = int(box.cls[0])
            cls_name   = model.names[cls_id]
            confidence = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            cx_px = (x1 + x2) / 2
            cy_px = (y1 + y2) / 2
            w_mm  = (x2 - x1) / PX_PER_MM
            h_mm  = (y2 - y1) / PX_PER_MM

            should_remove = cls_name == 'right_glove'
            if should_remove:
                remove_count += 1

            # ── SCARA JSON output ──────────────────────────────────
            payload = {
                "timestamp_ms" : int(elapsed_ms),
                "belt_pos_mm"  : round(belt_pos_mm, 1),
                "class"        : cls_name,
                "confidence"   : round(confidence, 3),
                "bbox_px"      : [x1, y1, x2, y2],
                "centroid_mm"  : [round(cx_px/PX_PER_MM, 1),
                                  round(cy_px/PX_PER_MM, 1)],
                "size_mm"      : [round(w_mm, 1), round(h_mm, 1)],
                "remove"       : should_remove,
            }
            print(json.dumps(payload))

            if SHOW_DISPLAY:
                color = (0, 0, 255) if should_remove else (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = f"{cls_name} {confidence:.2f}"
                cv2.putText(frame, label, (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, color, 2)

        # ── FPS display ───────────────────────────────────────────
        frame_count += 1
        if frame_count % 20 == 0:
            fps_display = 20 / (time.time() - fps_start)
            fps_start   = time.time()

        if SHOW_DISPLAY:
            cv2.putText(frame, f"FPS: {fps_display:.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 0), 2)
            if remove_count > 0:
                cv2.putText(frame,
                            f"REMOVE: {remove_count} right glove(s)",
                            (10, 65),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2)

            cv2.imshow("GRIP - Pi Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    if SHOW_DISPLAY:
        cv2.destroyAllWindows()
    print("✅ Stopped.")
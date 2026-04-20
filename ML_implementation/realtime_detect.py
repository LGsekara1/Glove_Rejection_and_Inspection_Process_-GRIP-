from ultralytics import YOLO
import cv2
import time

if __name__ == '__main__':

    # ── config ─────────────────────────────────────────────────────
    MODEL_PATH = r"D:/Abdul Rahman/Engineering_UOM/Personal Projects/GRIP/VsCode implementation/Find_gloves.yolov8/ML_implementation/runs/detect/runs/glove_v3_leftright/weights/best.pt"
    CONF       = 0.50
    CAMERA_ID  = 0
    # ───────────────────────────────────────────────────────────────

    model = YOLO(MODEL_PATH)
    cap   = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("❌ Camera not found. Try CAMERA_ID = 1 or 2")
        exit()

    print("✅ Stage 2 live detection running...")
    print("   GREEN box = left_glove  (correct — stay on belt)")
    print("   RED box   = right_glove (wrong — SCARA removes)")
    print("   Press Q to quit | S to save frame")

    fps_start   = time.time()
    fps_display = 0
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, conf=CONF, verbose=False)[0]

        remove_count = 0

        for box in results.boxes:
            cls_id     = int(box.cls[0])
            cls_name   = model.names[cls_id]
            confidence = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            # GREEN = left_glove (correct)
            # RED   = right_glove (should be removed)
            if cls_name == 'left_glove':
                color  = (0, 255, 0)
                action = "OK"
            else:
                color  = (0, 0, 255)
                action = "REMOVE"
                remove_count += 1

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            label      = f"{cls_name} {confidence:.2f} [{action}]"
            label_size = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0]
            cv2.rectangle(frame,
                          (x1, y1 - label_size[1] - 10),
                          (x1 + label_size[0], y1),
                          color, -1)
            cv2.putText(frame, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 0), 2)

        # FPS
        frame_count += 1
        if frame_count % 30 == 0:
            fps_display = 30 / (time.time() - fps_start)
            fps_start   = time.time()

        cv2.putText(frame, f"FPS: {fps_display:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"Total: {len(results.boxes)}",
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)

        # flash warning if right glove detected
        if remove_count > 0:
            cv2.putText(frame,
                        f"⚠ {remove_count} RIGHT GLOVE DETECTED — REMOVE",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)

        cv2.imshow("GRIP Stage 2 — Left/Right Detection", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = f"capture_{int(time.time())}.jpg"
            cv2.imwrite(fname, frame)
            print(f"📸 Saved: {fname}")

    cap.release()
    cv2.destroyAllWindows()
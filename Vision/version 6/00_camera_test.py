import cv2
import time

CAM_INDEX = 1

# Use DirectShow on Windows
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)

# Force MJPEG format - very important for smooth webcam video
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

# Start with 720p first for testing
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)

# Reduce delay from internal camera buffer
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
print("Actual width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print("Actual height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Actual FPS:", cap.get(cv2.CAP_PROP_FPS))
if not cap.isOpened():
    print("Camera not opened. Try CAM_INDEX = 1 or 2")
    exit()

prev_time = time.time()

while True:
    # grab + retrieve helps reduce old-frame delay
    cap.grab()
    ret, frame = cap.retrieve()

    if not ret:
        print("Frame not received")
        break

    now = time.time()
    fps = 1 / (now - prev_time)
    prev_time = now

    h, w = frame.shape[:2]

    cv2.putText(frame, f"Resolution: {w}x{h}", (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    cv2.putText(frame, f"FPS: {fps:.1f}", (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Low Latency Camera Test", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
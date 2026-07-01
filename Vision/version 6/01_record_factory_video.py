import cv2
import time
import json
from pathlib import Path
from datetime import datetime

CAM_INDEX = 1
WIDTH = 1920
HEIGHT = 1080
FPS = 30
CONVEYOR_SPEED_MPS = 0.254

clip_type = input("Clip label, e.g. left_good_normal: ").strip() or "factory_clip"

out_dir = Path("data/raw_videos")
out_dir.mkdir(parents=True, exist_ok=True)

ts = datetime.now().strftime("%Y%m%d_%H%M%S")
video_path = out_dir / f"{ts}_{clip_type}.mp4"
meta_path = out_dir / f"{ts}_{clip_type}.json"

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

fourcc = cv2.VideoWriter_fourcc(*"mp4v")
writer = cv2.VideoWriter(str(video_path), fourcc, FPS, (WIDTH, HEIGHT))

frame_count = 0
start = time.time()
print("Recording. Press ESC to stop.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame.")
        break

    writer.write(frame)
    frame_count += 1

    preview = cv2.resize(frame, (960, 540))
    cv2.putText(preview, f"REC {clip_type} frame={frame_count}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.imshow("Factory Recording", preview)

    if cv2.waitKey(1) & 0xFF == 27:
        break

duration = time.time() - start
metadata = {
    "clip_type": clip_type,
    "video_path": str(video_path),
    "resolution": [WIDTH, HEIGHT],
    "fps_requested": FPS,
    "frame_count": frame_count,
    "duration_seconds": duration,
    "conveyor_speed_mps": CONVEYOR_SPEED_MPS,
    "notes": input("Notes about lighting/defects/placement: ")
}

with open(meta_path, "w") as f:
    json.dump(metadata, f, indent=4)

cap.release()
writer.release()
cv2.destroyAllWindows()

print("Saved:", video_path)
print("Saved:", meta_path)

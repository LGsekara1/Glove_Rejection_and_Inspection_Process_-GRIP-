import cv2
from pathlib import Path
from tqdm import tqdm

VIDEO_DIR = Path("data/raw_videos")
OUT_DIR = Path("data/extracted_frames")
OUT_DIR.mkdir(parents=True, exist_ok=True)

FRAME_INTERVAL = 10

for video_path in VIDEO_DIR.glob("*.mp4"):
    cap = cv2.VideoCapture(str(video_path))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_idx = 0
    save_idx = 0

    clip_out = OUT_DIR / video_path.stem
    clip_out.mkdir(parents=True, exist_ok=True)

    for _ in tqdm(range(total), desc=video_path.name):
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx % FRAME_INTERVAL == 0:
            out_path = clip_out / f"{video_path.stem}_{save_idx:05d}.jpg"
            cv2.imwrite(str(out_path), frame)
            save_idx += 1

        frame_idx += 1

    cap.release()
    print(f"{video_path.name}: saved {save_idx} frames")

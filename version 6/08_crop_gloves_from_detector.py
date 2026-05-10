from ultralytics import YOLO
import cv2
from pathlib import Path

MODEL_PATH = "runs/detect/train/weights/best.pt"
IMG_DIR = Path("data/filtered_frames/good")
OUT_DIR = Path("data/glove_crops_for_landmarks")
OUT_DIR.mkdir(parents=True, exist_ok=True)

model = YOLO(MODEL_PATH)

for img_path in IMG_DIR.glob("*.jpg"):
    img = cv2.imread(str(img_path))
    if img is None:
        continue

    result = model(img, imgsz=640, conf=0.4)[0]
    for idx, box in enumerate(result.boxes):
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        crop = img[y1:y2, x1:x2]
        if crop.size == 0:
            continue
        out_path = OUT_DIR / f"{img_path.stem}_glove{idx}.jpg"
        cv2.imwrite(str(out_path), crop)

print("Glove crop generation complete. Annotate these for landmarks.")

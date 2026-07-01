import cv2
from pathlib import Path
import albumentations as A

IMG_DIR = Path("dataset_split/images/train")
LAB_DIR = Path("dataset_split/labels/train")
OUT_IMG = Path("dataset_aug/images/train")
OUT_LAB = Path("dataset_aug/labels/train")
OUT_IMG.mkdir(parents=True, exist_ok=True)
OUT_LAB.mkdir(parents=True, exist_ok=True)

AUG_PER_IMAGE = 2

transform = A.Compose([
    A.RandomBrightnessContrast(p=0.6),
    A.GaussNoise(p=0.25),
    A.MotionBlur(blur_limit=5, p=0.20),
    A.Rotate(limit=12, border_mode=cv2.BORDER_CONSTANT, p=0.5),
    A.Affine(scale=(0.9, 1.1), translate_percent=(-0.04, 0.04), p=0.5),
], bbox_params=A.BboxParams(format="yolo", label_fields=["class_labels"]))

def read_yolo_labels(path):
    bboxes, classes = [], []
    if not path.exists():
        return bboxes, classes
    for line in path.read_text().strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        classes.append(int(parts[0]))
        bboxes.append([float(x) for x in parts[1:5]])
    return bboxes, classes

def write_yolo_labels(path, bboxes, classes):
    lines = []
    for cls, box in zip(classes, bboxes):
        x, y, w, h = box
        if w <= 0 or h <= 0:
            continue
        lines.append(f"{cls} {x:.6f} {y:.6f} {w:.6f} {h:.6f}")
    path.write_text("\n".join(lines))

for img_path in IMG_DIR.glob("*.jpg"):
    img = cv2.imread(str(img_path))
    if img is None:
        continue

    label_path = LAB_DIR / f"{img_path.stem}.txt"
    bboxes, classes = read_yolo_labels(label_path)

    cv2.imwrite(str(OUT_IMG / img_path.name), img)
    write_yolo_labels(OUT_LAB / f"{img_path.stem}.txt", bboxes, classes)

    for i in range(AUG_PER_IMAGE):
        try:
            aug = transform(image=img, bboxes=bboxes, class_labels=classes)
            out_name = f"{img_path.stem}_aug{i}.jpg"
            cv2.imwrite(str(OUT_IMG / out_name), aug["image"])
            write_yolo_labels(OUT_LAB / f"{img_path.stem}_aug{i}.txt",
                              aug["bboxes"], aug["class_labels"])
        except Exception as e:
            print("Aug failed:", img_path, e)

print("Detection augmentation complete.")

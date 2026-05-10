import cv2
from pathlib import Path
import albumentations as A

IN_DIR = Path("label_roi_split/train")
OUT_DIR = Path("label_roi_aug/train")
CLASSES = ["non_defect", "defect"]
AUG_PER_IMAGE = 2

transform = A.Compose([
    A.RandomBrightnessContrast(p=0.7),
    A.GaussNoise(p=0.25),
    A.MotionBlur(blur_limit=5, p=0.25),
    A.Rotate(limit=8, border_mode=cv2.BORDER_CONSTANT, p=0.4),
    A.Sharpen(p=0.25),
])

for cls in CLASSES:
    out_cls = OUT_DIR / cls
    out_cls.mkdir(parents=True, exist_ok=True)

    for img_path in (IN_DIR / cls).glob("*.jpg"):
        img = cv2.imread(str(img_path))
        if img is None:
            continue

        cv2.imwrite(str(out_cls / img_path.name), img)

        for i in range(AUG_PER_IMAGE):
            aug = transform(image=img)["image"]
            out_name = f"{img_path.stem}_aug{i}.jpg"
            cv2.imwrite(str(out_cls / out_name), aug)

print("Classification augmentation complete.")

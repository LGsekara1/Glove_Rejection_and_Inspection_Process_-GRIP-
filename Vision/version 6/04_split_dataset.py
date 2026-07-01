from pathlib import Path
import random
import shutil

RAW_IMG_DIR = Path("dataset_raw/images")
RAW_LABEL_DIR = Path("dataset_raw/labels")
OUT_DIR = Path("dataset_split")

TRAIN_RATIO = 0.70
VAL_RATIO = 0.20
SEED = 42

random.seed(SEED)
images = sorted(list(RAW_IMG_DIR.glob("*.jpg")) + list(RAW_IMG_DIR.glob("*.png")))
random.shuffle(images)

n = len(images)
n_train = int(n * TRAIN_RATIO)
n_val = int(n * VAL_RATIO)

splits = {
    "train": images[:n_train],
    "val": images[n_train:n_train+n_val],
    "test": images[n_train+n_val:]
}

for split, files in splits.items():
    img_out = OUT_DIR / "images" / split
    lab_out = OUT_DIR / "labels" / split
    img_out.mkdir(parents=True, exist_ok=True)
    lab_out.mkdir(parents=True, exist_ok=True)

    for img_path in files:
        label_path = RAW_LABEL_DIR / f"{img_path.stem}.txt"
        shutil.copy(str(img_path), str(img_out / img_path.name))
        if label_path.exists():
            shutil.copy(str(label_path), str(lab_out / label_path.name))
        else:
            print("Missing label:", label_path)

print("Split complete:")
for split, files in splits.items():
    print(split, len(files))

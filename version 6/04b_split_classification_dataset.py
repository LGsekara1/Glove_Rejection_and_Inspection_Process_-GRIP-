from pathlib import Path
import random
import shutil

RAW_DIR = Path("label_roi_raw")
OUT_DIR = Path("label_roi_split")
CLASSES = ["non_defect", "defect"]

TRAIN_RATIO = 0.70
VAL_RATIO = 0.20
SEED = 42
random.seed(SEED)

for cls in CLASSES:
    files = list((RAW_DIR / cls).glob("*.jpg")) + list((RAW_DIR / cls).glob("*.png"))
    random.shuffle(files)

    n = len(files)
    n_train = int(n * TRAIN_RATIO)
    n_val = int(n * VAL_RATIO)

    split_files = {
        "train": files[:n_train],
        "val": files[n_train:n_train+n_val],
        "test": files[n_train+n_val:]
    }

    for split, split_list in split_files.items():
        out = OUT_DIR / split / cls
        out.mkdir(parents=True, exist_ok=True)
        for f in split_list:
            shutil.copy(str(f), str(out / f.name))

print("Classification split complete.")

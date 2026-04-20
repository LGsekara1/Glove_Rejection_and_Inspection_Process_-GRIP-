import os
import shutil
import random
from pathlib import Path

# ── YOUR EXACT PATHS ──────────────────────────────────────────────────
LEFT_TRAIN_IMAGES  = r"D:/Abdul Rahman/Engineering_UOM/Personal Projects/Grip v2/Left_Data_set/train/images"
LEFT_TRAIN_LABELS  = r"D:/Abdul Rahman/Engineering_UOM/Personal Projects/Grip v2/Left_Data_set/train/labels"
LEFT_VALID_IMAGES  = r"D:/Abdul Rahman/Engineering_UOM/Personal Projects/Grip v2/Left_Data_set/valid/images"
LEFT_VALID_LABELS  = r"D:/Abdul Rahman/Engineering_UOM/Personal Projects/Grip v2/Left_Data_set/valid/labels"

RIGHT_TRAIN_IMAGES = r"D:/Abdul Rahman/Engineering_UOM/Personal Projects/Grip v2/Right_Data_set/train/images"
RIGHT_TRAIN_LABELS = r"D:/Abdul Rahman/Engineering_UOM/Personal Projects/Grip v2/Right_Data_set/train/labels"

OUTPUT_ROOT        = r"D:/Abdul Rahman/Engineering_UOM/Personal Projects/GRIP/VsCode implementation/Find_gloves.yolov8/Training_v2"
# ──────────────────────────────────────────────────────────────────────

def make_dirs(root):
    for split in ['train', 'valid']:
        Path(f"{root}/{split}/images").mkdir(parents=True, exist_ok=True)
        Path(f"{root}/{split}/labels").mkdir(parents=True, exist_ok=True)

def copy_images(src_dir, dst_dir, prefix=""):
    src = Path(src_dir)
    if not src.exists():
        print(f"⚠️  Not found: {src_dir}")
        return 0
    count = 0
    for img in list(src.glob("*.jpg")) + list(src.glob("*.png")):
        shutil.copy(img, Path(dst_dir) / (prefix + img.name))
        count += 1
    return count

def copy_labels_keep(src_dir, dst_dir, prefix=""):
    src = Path(src_dir)
    if not src.exists():
        print(f"⚠️  Not found: {src_dir}")
        return 0
    count = 0
    for lbl in src.glob("*.txt"):
        shutil.copy(lbl, Path(dst_dir) / (prefix + lbl.name))
        count += 1
    return count

def copy_labels_remap(src_dir, dst_dir, old_cls, new_cls, prefix=""):
    src = Path(src_dir)
    if not src.exists():
        print(f"⚠️  Not found: {src_dir}")
        return 0
    count = 0
    for lbl in src.glob("*.txt"):
        lines = lbl.read_text().strip().splitlines()
        new_lines = []
        for line in lines:
            if not line.strip(): continue
            parts = line.strip().split()
            if int(parts[0]) == old_cls:
                parts[0] = str(new_cls)
            new_lines.append(" ".join(parts))
        (Path(dst_dir) / (prefix + lbl.name)).write_text("\n".join(new_lines))
        count += 1
    return count

def split_right_to_valid(train_images, train_labels, valid_images, valid_labels,
                          ratio=0.2, prefix="right_"):
    """Split 20% of right glove train images into valid set"""
    all_imgs = (list(Path(train_images).glob("*.jpg")) +
                list(Path(train_images).glob("*.png")))
    random.seed(42)
    random.shuffle(all_imgs)
    valid_count = int(len(all_imgs) * ratio)
    valid_list  = all_imgs[:valid_count]

    moved = 0
    for img in valid_list:
        lbl = Path(train_labels) / (img.stem + ".txt")
        shutil.copy(img, Path(valid_images) / (prefix + img.name))
        if lbl.exists():
            lines = lbl.read_text().strip().splitlines()
            new_lines = []
            for line in lines:
                if not line.strip(): continue
                parts = line.strip().split()
                parts[0] = "1"   # remap 0 → 1
                new_lines.append(" ".join(parts))
            (Path(valid_labels) / (prefix + lbl.name)).write_text(
                "\n".join(new_lines))
        moved += 1
    return moved

if __name__ == '__main__':
    print("🔧 Building merged dataset...")
    make_dirs(OUTPUT_ROOT)

    # ── TRAIN ──────────────────────────────────────────────────────────
    print("\n── Train set ──")
    n = copy_images(LEFT_TRAIN_IMAGES,  f"{OUTPUT_ROOT}/train/images", "left_")
    print(f"  Left  images : {n}")
    n = copy_labels_keep(LEFT_TRAIN_LABELS, f"{OUTPUT_ROOT}/train/labels", "left_")
    print(f"  Left  labels : {n}  (class 0 = left_glove ✓)")

    n = copy_images(RIGHT_TRAIN_IMAGES, f"{OUTPUT_ROOT}/train/images", "right_")
    print(f"  Right images : {n}")
    n = copy_labels_remap(RIGHT_TRAIN_LABELS, f"{OUTPUT_ROOT}/train/labels",
                          old_cls=0, new_cls=1, prefix="right_")
    print(f"  Right labels : {n}  (remapped 0→1 = right_glove ✓)")

    # ── VALID ──────────────────────────────────────────────────────────
    print("\n── Valid set ──")
    n = copy_images(LEFT_VALID_IMAGES,  f"{OUTPUT_ROOT}/valid/images", "left_")
    print(f"  Left  images : {n}")
    n = copy_labels_keep(LEFT_VALID_LABELS, f"{OUTPUT_ROOT}/valid/labels", "left_")
    print(f"  Left  labels : {n}")

    n = split_right_to_valid(
        RIGHT_TRAIN_IMAGES, RIGHT_TRAIN_LABELS,
        f"{OUTPUT_ROOT}/valid/images",
        f"{OUTPUT_ROOT}/valid/labels",
        ratio=0.2, prefix="right_"
    )
    print(f"  Right images : {n}  (20% split from train)")
    print(f"  Right labels : {n}  (remapped 0→1 ✓)")

    # ── data.yaml ──────────────────────────────────────────────────────
    yaml = f"""path: {OUTPUT_ROOT.replace(chr(92), '/')}
train: train/images
val:   valid/images

nc: 2
names: ['left_glove', 'right_glove']
"""
    (Path(OUTPUT_ROOT) / "data.yaml").write_text(yaml)

    # ── summary ────────────────────────────────────────────────────────
    train_total = len(list(Path(f"{OUTPUT_ROOT}/train/images").glob("*.jpg")))
    valid_total = len(list(Path(f"{OUTPUT_ROOT}/valid/images").glob("*.jpg")))

    print(f"\n📊 Final merged dataset:")
    print(f"   Train : {train_total} images")
    print(f"   Valid : {valid_total} images")
    print(f"   Classes: 0=left_glove, 1=right_glove")
    print(f"\n✅ data.yaml saved to {OUTPUT_ROOT}")
    print(f"🎯 Now run train_stage2.py")
from pathlib import Path
import random
import shutil
import yaml
from collections import defaultdict, Counter

# ============================================================
# CHANGE ONLY THIS IF YOUR FOLDER NAME IS DIFFERENT
# ============================================================

DATASET_ROOT = Path(r"D:\GRIP_Yolo_pose\YOLO_pose_MVP_dataset")

ANNOTATED_IMAGES = DATASET_ROOT / "annotated" / "images"
ANNOTATED_LABELS = DATASET_ROOT / "annotated" / "labels"

OUT_ROOT = DATASET_ROOT / "yolo_pose_70_20_10"

IMAGE_EXTS = [".jpg", ".jpeg", ".png", ".bmp", ".webp"]

CLASS_NAMES = {
    0: "glove"
}

KEYPOINT_NAMES = [
    "wrist_center",
    "palm_center",
    "thumb_tip",
    "index_tip",
    "middle_tip",
    "ring_tip",
    "pinky_tip",
]

RANDOM_SEED = 42

TRAIN_RATIO = 0.70
VAL_RATIO = 0.20
TEST_RATIO = 0.10


def detect_source_group(img_path):
    """
    This is NOT the YOLO class.
    This is only used to split the dataset evenly.

    The model still has only one class:
    0 = glove
    """

    name = img_path.name.lower()

    if "left_glove" in name or "left" in name:
        return "left_source"

    if "right_glove" in name or "right" in name:
        return "right_source"

    return "unknown_source"


def validate_pose_label(label_path):
    """
    YOLO-pose expected format for 7 keypoints:

    class xc yc w h x1 y1 v1 x2 y2 v2 ... x7 y7 v7

    Total values:
    1 class + 4 bbox + 7*3 keypoint values = 26
    """

    lines = label_path.read_text(encoding="utf-8").splitlines()

    if len(lines) == 0:
        return False

    for line in lines:
        parts = line.strip().split()

        if len(parts) != 26:
            print(f"  Expected 26 values, got {len(parts)} in {label_path.name}")
            print(f"  Line: {line}")
            return False

        try:
            cls = int(float(parts[0]))
            values = list(map(float, parts[1:]))
        except ValueError:
            return False

        if cls != 0:
            print(f"  Expected class 0 only, got class {cls} in {label_path.name}")
            return False

        bbox_values = values[:4]

        if any(v < 0 or v > 1 for v in bbox_values):
            print(f"  BBox values outside 0 to 1 in {label_path.name}")
            return False

        keypoint_values = values[4:]

        for i in range(0, len(keypoint_values), 3):
            x = keypoint_values[i]
            y = keypoint_values[i + 1]
            v = int(keypoint_values[i + 2])

            if v not in [0, 1, 2]:
                print(f"  Bad visibility value {v} in {label_path.name}")
                return False

            if v > 0:
                if x < 0 or x > 1 or y < 0 or y > 1:
                    print(f"  Keypoint outside 0 to 1 in {label_path.name}")
                    return False

    return True


def find_image_label_pairs():
    grouped_pairs = defaultdict(list)

    for img_path in sorted(ANNOTATED_IMAGES.iterdir()):
        if img_path.suffix.lower() not in IMAGE_EXTS:
            continue

        label_path = ANNOTATED_LABELS / f"{img_path.stem}.txt"

        if not label_path.exists():
            print(f"[NO LABEL] {img_path.name}")
            continue

        if not validate_pose_label(label_path):
            print(f"[BAD LABEL] {label_path.name}")
            continue

        group = detect_source_group(img_path)

        grouped_pairs[group].append((img_path, label_path))

    return grouped_pairs


def clean_output():
    if OUT_ROOT.exists():
        print(f"Deleting old split folder: {OUT_ROOT}")
        shutil.rmtree(OUT_ROOT)

    for split in ["train", "val", "test"]:
        (OUT_ROOT / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUT_ROOT / "labels" / split).mkdir(parents=True, exist_ok=True)


def split_group(pairs):
    random.shuffle(pairs)

    n = len(pairs)

    n_train = int(n * TRAIN_RATIO)
    n_val = int(n * VAL_RATIO)

    train_pairs = pairs[:n_train]
    val_pairs = pairs[n_train:n_train + n_val]
    test_pairs = pairs[n_train + n_val:]

    return train_pairs, val_pairs, test_pairs


def copy_pairs(pairs, split):
    for img_path, label_path in pairs:
        shutil.copy2(
            img_path,
            OUT_ROOT / "images" / split / img_path.name
        )

        shutil.copy2(
            label_path,
            OUT_ROOT / "labels" / split / label_path.name
        )


def write_data_yaml():
    data = {
        "path": str(OUT_ROOT).replace("\\", "/"),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",

        # 7 keypoints, each with x, y, visibility
        "kpt_shape": [7, 3],

        # We will train with fliplr=0.0 and flipud=0.0.
        # This identity list is kept only for compatibility.
        "flip_idx": [0, 1, 2, 3, 4, 5, 6],

        # IMPORTANT:
        # Only one YOLO class.
        # Left/right is decided later from keypoint geometry.
        "names": CLASS_NAMES,
    }

    yaml_path = OUT_ROOT / "data.yaml"

    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)

    print("Saved data.yaml:", yaml_path)


def write_keypoint_docs():
    docs_dir = DATASET_ROOT / "docs"
    docs_dir.mkdir(parents=True, exist_ok=True)

    text = "YOLO-Pose keypoint order:\n\n"

    for i, name in enumerate(KEYPOINT_NAMES):
        text += f"{i}: {name}\n"

    text += """
Label format:
class xc yc w h x1 y1 v1 x2 y2 v2 ... x7 y7 v7

Visibility:
0 = missing
1 = occluded / uncertain
2 = visible

Important:
YOLO class is only:
0 = glove

Left/right/wrong-glove decision is NOT stored as YOLO classes.
It will be computed later from the predicted keypoint geometry.

Training note:
Use fliplr=0.0 and flipud=0.0.
Do not use single horizontal/vertical flip because finger-side geometry matters.
"""

    (docs_dir / "keypoint_order_and_training_notes.txt").write_text(
        text,
        encoding="utf-8"
    )


def write_split_report(split_counts):
    report_path = OUT_ROOT / "split_report.txt"

    lines = []
    lines.append("YOLO-Pose Split Report")
    lines.append("======================")
    lines.append("")
    lines.append("YOLO classes:")
    lines.append("0 = glove")
    lines.append("")
    lines.append("Source groups are only for balancing the split.")
    lines.append("They are NOT YOLO classes.")
    lines.append("")

    for split in ["train", "val", "test"]:
        lines.append(f"{split.upper()}:")
        for group, count in split_counts[split].items():
            lines.append(f"  {group}: {count}")
        lines.append("")

    report_path.write_text("\n".join(lines), encoding="utf-8")

    print("Saved split report:", report_path)


def main():
    if not ANNOTATED_IMAGES.exists():
        raise FileNotFoundError(f"Annotated images folder not found: {ANNOTATED_IMAGES}")

    if not ANNOTATED_LABELS.exists():
        raise FileNotFoundError(f"Annotated labels folder not found: {ANNOTATED_LABELS}")

    grouped_pairs = find_image_label_pairs()

    total_valid = sum(len(v) for v in grouped_pairs.values())

    if total_valid == 0:
        raise RuntimeError("No valid annotated image-label pairs found.")

    print("Valid annotated pairs found:", total_valid)

    print("\nSource groups found:")
    for group, pairs in grouped_pairs.items():
        print(f"{group}: {len(pairs)}")

    random.seed(RANDOM_SEED)

    clean_output()

    split_counts = {
        "train": Counter(),
        "val": Counter(),
        "test": Counter(),
    }

    total_train = 0
    total_val = 0
    total_test = 0

    for group, pairs in grouped_pairs.items():
        train_pairs, val_pairs, test_pairs = split_group(pairs)

        copy_pairs(train_pairs, "train")
        copy_pairs(val_pairs, "val")
        copy_pairs(test_pairs, "test")

        split_counts["train"][group] += len(train_pairs)
        split_counts["val"][group] += len(val_pairs)
        split_counts["test"][group] += len(test_pairs)

        total_train += len(train_pairs)
        total_val += len(val_pairs)
        total_test += len(test_pairs)

    write_data_yaml()
    write_keypoint_docs()
    write_split_report(split_counts)

    print("\n==============================")
    print("YOLO-POSE STRATIFIED SPLIT COMPLETE")
    print("==============================")
    print("Output:", OUT_ROOT)
    print(f"Total: {total_valid}")
    print(f"Train: {total_train}")
    print(f"Val:   {total_val}")
    print(f"Test:  {total_test}")

    print("\nSplit by source group:")
    for split in ["train", "val", "test"]:
        print(f"\n{split.upper()}:")
        for group, count in split_counts[split].items():
            print(f"  {group}: {count}")

    print("\nRemember:")
    print("data.yaml has only one class: 0 = glove")
    print("left/right decision will come later from keypoint geometry.")


if __name__ == "__main__":
    main()
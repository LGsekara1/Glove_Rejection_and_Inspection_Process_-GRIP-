import csv
import random
import shutil
import subprocess
import sys
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import cv2
import numpy as np

try:
    from PIL import Image, ImageTk, ImageDraw
except ImportError:
    raise SystemExit("Pillow is required. Install with: pip install pillow")


# ============================================================
# DEFAULT PROJECT SETTINGS
# ============================================================
# Project root:
# D:\GRIP_Dataset\factory_check

# Source video root:
# D:\GRIP_Dataset\factory_check\RECOVERED_BATCH
DEFAULT_PROJECT_ROOT = Path(r"D:\GRIP_Dataset\factory_check")
DEFAULT_OUTPUT_NAME = "YOLO_pose_12kp_dataset"

VIDEO_EXTS = {".mp4", ".avi", ".mov", ".mkv", ".wmv"}
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}

CLASS_NAMES = ["left_glove", "right_glove", "unclear_glove"]

# ============================================================
# 12-KEYPOINT GLOVE POSE SCHEMA
# ============================================================
# Reduced version for faster annotation.
# The names use ANATOMICAL glove sides, not image left/right.
# "thumb_side" means the side where the thumb is located.
# "pinky_side" means the opposite outer side.
#
# Removed to save annotation time:
# - cuff points (old 0, 1, 2)
# - palm center (old 7)
# - palm_upper_thumb_corner (old 5), because it overlaps visually with index_base
# - palm_upper_pinky_corner (old 6), because it overlaps visually with pinky_base
#
# Keep this order fixed forever after you start annotating.
KEYPOINT_NAMES = [
    "palm_lower_thumb_corner",  # 0
    "palm_lower_pinky_corner",  # 1

    "thumb_base_or_web",        # 2
    "thumb_tip",                # 3

    "index_base",               # 4
    "index_tip",                # 5

    "middle_base",              # 6
    "middle_tip",               # 7

    "ring_base",                # 8
    "ring_tip",                 # 9

    "pinky_base",               # 10
    "pinky_tip",                # 11
]

# For preview drawing only.
# This draws a simple lower-palm line, a finger-base arc, and finger chains.
SKELETON_EDGES = [
    # lower palm edge
    (0, 1),

    # thumb
    (0, 2), (2, 3),

    # finger-base arc
    (2, 4), (4, 6), (6, 8), (8, 10), (10, 1),

    # fingers
    (4, 5), (6, 7), (8, 9), (10, 11),

    # useful inner palm diagonals for visual guidance
    (0, 6), (1, 6),
]

KEYPOINT_COLORS = [
    (255, 180, 0),    # 0 palm lower thumb
    (255, 180, 0),    # 1 palm lower pinky

    (255, 0, 255),    # 2 thumb base
    (255, 0, 255),    # 3 thumb tip

    (0, 255, 0),      # 4 index base
    (0, 255, 0),      # 5 index tip

    (255, 255, 0),    # 6 middle base
    (255, 255, 0),    # 7 middle tip

    (255, 128, 0),    # 8 ring base
    (255, 128, 0),    # 9 ring tip

    (0, 128, 255),    # 10 pinky base
    (0, 128, 255),    # 11 pinky tip
]


# ============================================================
# PATH HELPERS
# ============================================================

def get_output_paths(project_root, output_name):
    out_root = Path(project_root) / output_name

    return {
        "out_root": out_root,
        "frames": out_root / "frames_to_label",
        "rejected": out_root / "rejected_frames",
        "manual": out_root / "manual_review",
        "annotated_images": out_root / "annotated" / "images",
        "annotated_labels": out_root / "annotated" / "labels",
        "previews": out_root / "annotated" / "previews",
        "contact_sheets": out_root / "contact_sheets",
        "reports": out_root / "reports",
        "docs": out_root / "docs",
        "split": out_root / "yolo_pose_70_20_10",
    }


def ensure_dirs(paths):
    for key, path in paths.items():
        path.mkdir(parents=True, exist_ok=True)


def open_folder(path):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)

    if sys.platform.startswith("win"):
        subprocess.Popen(f'explorer "{path}"')
    elif sys.platform.startswith("darwin"):
        subprocess.Popen(["open", str(path)])
    else:
        subprocess.Popen(["xdg-open", str(path)])


# ============================================================
# DOCS + YAML
# ============================================================

def write_pose_data_yaml(split_root):
    yaml_path = Path(split_root) / "data.yaml"

    names_lines = "\n".join([f"  {i}: {name}" for i, name in enumerate(CLASS_NAMES)])

    # flip_idx identity because we will train with fliplr=0.0.
    # Do not mirror left/right glove images automatically for this project.
    flip_idx = list(range(len(KEYPOINT_NAMES)))

    content = (
        f"path: {Path(split_root).as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n"
        "\n"
        f"kpt_shape: [{len(KEYPOINT_NAMES)}, 3]\n"
        f"flip_idx: {flip_idx}\n"
        "\n"
        "names:\n"
        f"{names_lines}\n"
    )

    yaml_path.write_text(content, encoding="utf-8")


def write_docs(paths):
    docs = paths["docs"]
    docs.mkdir(parents=True, exist_ok=True)

    keypoint_doc = docs / "keypoints_order.txt"
    keypoint_doc.write_text(
        "\n".join([f"{i}: {name}" for i, name in enumerate(KEYPOINT_NAMES)]),
        encoding="utf-8"
    )

    class_doc = docs / "classes.txt"
    class_doc.write_text(
        "\n".join([f"{i}: {name}" for i, name in enumerate(CLASS_NAMES)]),
        encoding="utf-8"
    )

    kp_lines = "\n".join([f"{i} = {name}" for i, name in enumerate(KEYPOINT_NAMES)])
    class_lines = "\n".join([f"{i} = {name}" for i, name in enumerate(CLASS_NAMES)])

    readme = docs / "README_YOLO_POSE_DATASET.md"
    readme.write_text(
        f"""# GRIP YOLO-Pose 12-Keypoint Dataset

## Object classes

{class_lines}

Use `unclear_glove` for crumpled, folded, heavily occluded, ambiguous, or manual-inspection cases.

## Keypoint order

{kp_lines}

## Main annotation rule

Draw one tight bounding box around each visible glove.

Then place all visible keypoints in the fixed order above.

The keypoint names use anatomy, not image direction:

- `thumb_side` means the side where the thumb is.
- `pinky_side` means the opposite outer side.
- If a point is hidden but you can reasonably estimate it, mark visibility = 1.
- If you cannot estimate it consistently, mark it missing with visibility = 0.
- If a point is clearly visible, mark visibility = 2.

## Visibility values

YOLO-pose labels use:

0 = not labelled / missing  
1 = labelled but occluded / uncertain  
2 = visible  

## Important training note

Do not use horizontal or vertical flipping for this dataset at first.

Use:

fliplr=0.0  
flipud=0.0  

For this project, automatic flipping can convert left-glove geometry into right-glove geometry while keeping the wrong label, unless labels and keypoints are transformed carefully.

## Output folders

- frames_to_label: extracted/imported frames ready to annotate
- annotated/images: saved annotated images
- annotated/labels: YOLO-pose labels
- annotated/previews: visual previews
- yolo_pose_70_20_10: final YOLO-pose train/val/test split
""",
        encoding="utf-8"
    )


# ============================================================
# FRAME EXTRACTION HELPERS
# ============================================================

def blur_score(gray):
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def glove_color_ratio_bgr(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    lower = np.array([30, 20, 35])
    upper = np.array([110, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    return np.count_nonzero(mask) / mask.size


def is_near_duplicate(frame, previous_small, threshold=2.5):
    small = cv2.resize(frame, (160, 90))
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

    if previous_small is None:
        return False, gray

    diff = cv2.absdiff(gray, previous_small)
    mean_diff = diff.mean()

    return mean_diff < threshold, gray


def resize_keep_aspect(img, width=260):
    h, w = img.shape[:2]
    scale = width / w
    new_w = width
    new_h = int(h * scale)
    return cv2.resize(img, (new_w, new_h))


def make_contact_sheet(image_paths, output_path, title, samples=50, cols=5):
    if not image_paths:
        return

    selected = image_paths[:samples]
    thumbs = []

    for p in selected:
        img = cv2.imread(str(p))
        if img is None:
            continue

        img = resize_keep_aspect(img, width=260)

        label = p.name[:35]
        cv2.putText(
            img,
            label,
            (5, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        thumbs.append(img)

    if not thumbs:
        return

    rows = int(np.ceil(len(thumbs) / cols))
    thumb_h = max(t.shape[0] for t in thumbs)
    thumb_w = max(t.shape[1] for t in thumbs)

    sheet = np.ones((rows * thumb_h + 55, cols * thumb_w, 3), dtype=np.uint8) * 255

    cv2.putText(
        sheet,
        title,
        (10, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 0, 0),
        2,
        cv2.LINE_AA,
    )

    for idx, thumb in enumerate(thumbs):
        r = idx // cols
        c = idx % cols

        y = 55 + r * thumb_h
        x = c * thumb_w

        h, w = thumb.shape[:2]
        sheet[y:y + h, x:x + w] = thumb

    cv2.imwrite(str(output_path), sheet)


def find_videos(source_root, output_root):
    source_root = Path(source_root)
    output_root = Path(output_root)

    videos = []

    for p in source_root.rglob("*"):
        if not p.is_file():
            continue

        if p.suffix.lower() not in VIDEO_EXTS:
            continue

        try:
            if output_root in p.parents:
                continue
        except Exception:
            pass

        videos.append(p)

    return sorted(videos)


def import_existing_images(source_root, paths):
    ensure_dirs(paths)
    write_docs(paths)

    source_root = Path(source_root)
    output_root = paths["out_root"]

    if not source_root.exists():
        messagebox.showerror("Folder not found", f"Source root not found:\n{source_root}")
        return

    image_paths = []

    for p in source_root.rglob("*"):
        if not p.is_file():
            continue

        if p.suffix.lower() not in IMAGE_EXTS:
            continue

        try:
            if output_root in p.parents:
                continue
        except Exception:
            pass

        image_paths.append(p)

    image_paths = sorted(image_paths)

    if not image_paths:
        messagebox.showerror("No images", f"No image files found in:\n{source_root}")
        return

    answer = messagebox.askyesno(
        "Import existing images",
        f"Found {len(image_paths)} images.\n\n"
        f"Copy them into:\n{paths['frames']}\n\n"
        "Continue?"
    )

    if not answer:
        return

    copied = 0

    for img_path in image_paths:
        parent_tag = img_path.parent.name.replace(" ", "_")
        filename = f"{parent_tag}_{img_path.stem}{img_path.suffix.lower()}"
        dst = paths["frames"] / filename

        # Avoid overwriting if the same name already exists.
        if dst.exists():
            base = dst.stem
            suffix = dst.suffix
            n = 1

            while dst.exists():
                dst = paths["frames"] / f"{base}_{n:03d}{suffix}"
                n += 1

        shutil.copy2(img_path, dst)
        copied += 1

    messagebox.showinfo(
        "Import complete",
        f"Imported {copied} images into:\n{paths['frames']}\n\n"
        "Now open the YOLO-Pose Annotator."
    )


def extract_frames_from_videos(
    source_root,
    paths,
    extract_every_seconds=0.5,
    blur_threshold=8.0,
    use_color_check=True,
    min_color_ratio=0.0015,
    reject_near_duplicates=True,
    max_frames_per_video=0,
):
    ensure_dirs(paths)
    write_docs(paths)

    videos = find_videos(source_root, paths["out_root"])

    if not videos:
        messagebox.showerror("No videos", f"No videos found in:\n{source_root}")
        return

    report_path = paths["reports"] / "frame_extraction_report.csv"

    rows = []

    for video_path in videos:
        cap = cv2.VideoCapture(str(video_path))

        if not cap.isOpened():
            print("Could not open:", video_path)
            continue

        fps = cap.get(cv2.CAP_PROP_FPS)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        if fps <= 0:
            fps = 25

        interval = max(1, int(fps * extract_every_seconds))

        source_tag = video_path.parent.name.replace(" ", "_")
        video_stem = video_path.stem.replace(" ", "_")

        frame_idx = 0
        saved_idx = 0
        accepted_count = 0
        rejected_count = 0
        previous_small = None

        print("\nProcessing:", video_path)
        print("FPS:", fps, "Total frames:", total_frames, "Interval:", interval)

        while True:
            ret, frame = cap.read()

            if not ret:
                break

            if frame_idx % interval == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                score = blur_score(gray)

                color_ratio = glove_color_ratio_bgr(frame) if use_color_check else 1.0

                too_similar, current_small = is_near_duplicate(frame, previous_small)

                accepted = True
                reason = "accepted"

                if score < blur_threshold:
                    accepted = False
                    reason = "very_blurry"

                if use_color_check and color_ratio < min_color_ratio:
                    accepted = False
                    reason = "no_glove_color"

                if accepted and reject_near_duplicates and too_similar:
                    accepted = False
                    reason = "near_duplicate"

                filename = (
                    f"{source_tag}_{video_stem}_"
                    f"f{frame_idx:07d}_s{saved_idx:05d}.jpg"
                )

                if accepted:
                    out_path = paths["frames"] / filename
                    previous_small = current_small
                    accepted_count += 1
                else:
                    out_path = paths["rejected"] / filename
                    rejected_count += 1

                cv2.imwrite(
                    str(out_path),
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 95]
                )

                rows.append({
                    "video": str(video_path),
                    "source_folder": source_tag,
                    "frame_index": frame_idx,
                    "time_sec": frame_idx / fps,
                    "filename": filename,
                    "saved_path": str(out_path),
                    "blur_score": score,
                    "glove_color_ratio": color_ratio,
                    "accepted": accepted,
                    "reason": reason,
                })

                saved_idx += 1

                if max_frames_per_video > 0 and accepted_count >= max_frames_per_video:
                    break

            frame_idx += 1

        cap.release()

        print("Accepted:", accepted_count)
        print("Rejected:", rejected_count)

    with open(report_path, "w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "video",
            "source_folder",
            "frame_index",
            "time_sec",
            "filename",
            "saved_path",
            "blur_score",
            "glove_color_ratio",
            "accepted",
            "reason",
        ]

        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    accepted = sorted(paths["frames"].glob("*.jpg"))
    rejected = sorted(paths["rejected"].glob("*.jpg"))

    random.shuffle(accepted)
    random.shuffle(rejected)

    make_contact_sheet(
        accepted,
        paths["contact_sheets"] / "accepted_frames_contact_sheet.jpg",
        "Accepted frames",
    )

    make_contact_sheet(
        rejected,
        paths["contact_sheets"] / "rejected_frames_contact_sheet.jpg",
        "Rejected frames",
    )

    messagebox.showinfo(
        "Extraction complete",
        f"Frames extracted.\n\nAccepted: {len(accepted)}\nRejected: {len(rejected)}\n\nOutput:\n{paths['out_root']}"
    )


# ============================================================
# YOLO-POSE ANNOTATOR
# ============================================================

class PoseAnnotator:
    def __init__(self, root, paths):
        self.root = root
        self.paths = paths
        self.root.title("GRIP YOLO-Pose Annotator")

        self.image_paths = sorted([
            p for p in paths["frames"].iterdir()
            if p.is_file() and p.suffix.lower() in IMAGE_EXTS
        ])

        if not self.image_paths:
            messagebox.showerror("No frames", f"No frames found in:\n{paths['frames']}")
            root.destroy()
            return

        ensure_dirs(paths)
        write_docs(paths)

        self.index = 0
        self.mode = tk.StringVar(value="bbox")
        self.selected_object_index = None
        self.selected_keypoint = tk.IntVar(value=0)
        self.visibility_value = tk.IntVar(value=2)
        self.current_class_id = tk.IntVar(value=0)
        self.auto_advance_var = tk.BooleanVar(value=True)

        self.original_img = None
        self.tk_img = None
        self.display_img = None

        self.img_w = 0
        self.img_h = 0
        self.scale = 1.0
        self.offset_x = 0
        self.offset_y = 0

        self.annotations = []

        self.start_img_xy = None
        self.temp_rect_id = None

        self.build_ui()
        self.bind_events()
        self.load_image()

    def build_ui(self):
        self.root.geometry("1600x950")

        self.main = ttk.Frame(self.root)
        self.main.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(self.main, bg="#1e1e1e", cursor="crosshair")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Right-side control panel with vertical scrollbar.
        # This fixes the issue where the Save / Next buttons are hidden on smaller screens.
        self.side_outer = ttk.Frame(self.main, width=430)
        self.side_outer.pack(side=tk.RIGHT, fill=tk.Y)
        self.side_outer.pack_propagate(False)

        self.side_canvas = tk.Canvas(self.side_outer, highlightthickness=0)
        self.side_scrollbar = ttk.Scrollbar(
            self.side_outer,
            orient=tk.VERTICAL,
            command=self.side_canvas.yview
        )

        self.side_canvas.configure(yscrollcommand=self.side_scrollbar.set)

        self.side_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.side_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.side = ttk.Frame(self.side_canvas)
        self.side_window = self.side_canvas.create_window(
            (0, 0),
            window=self.side,
            anchor="nw"
        )

        def _update_side_scroll_region(event=None):
            self.side_canvas.configure(scrollregion=self.side_canvas.bbox("all"))

        def _resize_side_window(event):
            self.side_canvas.itemconfig(self.side_window, width=event.width)

        def _mousewheel_scroll(event):
            self.side_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        self.side.bind("<Configure>", _update_side_scroll_region)
        self.side_canvas.bind("<Configure>", _resize_side_window)

        # Mouse wheel works when the cursor is over the right-side control panel.
        self.side_canvas.bind("<Enter>", lambda e: self.side_canvas.bind_all("<MouseWheel>", _mousewheel_scroll))
        self.side_canvas.bind("<Leave>", lambda e: self.side_canvas.unbind_all("<MouseWheel>"))

        self.file_label = ttk.Label(
            self.side,
            text="",
            wraplength=360,
            font=("Segoe UI", 10, "bold")
        )
        self.file_label.pack(anchor="w", padx=10, pady=(10, 3))

        self.progress_label = ttk.Label(self.side, text="")
        self.progress_label.pack(anchor="w", padx=10, pady=(0, 10))

        ttk.Separator(self.side).pack(fill=tk.X, pady=5)

        ttk.Label(self.side, text="Mode", font=("Segoe UI", 10, "bold")).pack(anchor="w", padx=10)

        ttk.Radiobutton(
            self.side,
            text="BBox mode: drag box for each glove",
            variable=self.mode,
            value="bbox"
        ).pack(anchor="w", padx=20)

        ttk.Radiobutton(
            self.side,
            text="Keypoint mode: click point for selected glove",
            variable=self.mode,
            value="keypoint"
        ).pack(anchor="w", padx=20)

        ttk.Separator(self.side).pack(fill=tk.X, pady=10)

        ttk.Label(self.side, text="Class for selected/new glove", font=("Segoe UI", 10, "bold")).pack(
            anchor="w", padx=10
        )

        for class_id, class_name in enumerate(CLASS_NAMES):
            ttk.Radiobutton(
                self.side,
                text=f"{class_id}: {class_name}",
                variable=self.current_class_id,
                value=class_id
            ).pack(anchor="w", padx=20, pady=1)

        ttk.Button(
            self.side,
            text="Apply class to selected glove",
            command=self.apply_class_to_selected
        ).pack(fill=tk.X, padx=10, pady=4)

        ttk.Separator(self.side).pack(fill=tk.X, pady=10)

        ttk.Label(self.side, text="Gloves on this image", font=("Segoe UI", 10, "bold")).pack(
            anchor="w", padx=10
        )

        self.object_list = tk.Listbox(self.side, height=8)
        self.object_list.pack(fill=tk.X, padx=10, pady=5)
        self.object_list.bind("<<ListboxSelect>>", self.on_object_select)

        button_frame = ttk.Frame(self.side)
        button_frame.pack(fill=tk.X, padx=10, pady=4)

        ttk.Button(button_frame, text="Delete selected glove", command=self.delete_selected_object).pack(fill=tk.X, pady=2)
        ttk.Button(button_frame, text="Clear all", command=self.clear_all).pack(fill=tk.X, pady=2)

        ttk.Separator(self.side).pack(fill=tk.X, pady=10)

        ttk.Label(self.side, text="Keypoints", font=("Segoe UI", 10, "bold")).pack(anchor="w", padx=10)

        kp_frame = ttk.Frame(self.side)
        kp_frame.pack(fill=tk.X, padx=10, pady=5)

        self.keypoint_list = tk.Listbox(kp_frame, height=12, exportselection=False)
        self.keypoint_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        kp_scroll = ttk.Scrollbar(kp_frame, orient=tk.VERTICAL, command=self.keypoint_list.yview)
        kp_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.keypoint_list.config(yscrollcommand=kp_scroll.set)

        for i, name in enumerate(KEYPOINT_NAMES):
            self.keypoint_list.insert(tk.END, f"{i + 1}. {name}")

        self.keypoint_list.selection_set(0)
        self.keypoint_list.bind("<<ListboxSelect>>", self.on_keypoint_select)

        ttk.Checkbutton(
            self.side,
            text="Auto-select next keypoint after click",
            variable=self.auto_advance_var
        ).pack(anchor="w", padx=20, pady=(2, 4))

        ttk.Separator(self.side).pack(fill=tk.X, pady=10)

        ttk.Label(self.side, text="Keypoint visibility", font=("Segoe UI", 10, "bold")).pack(anchor="w", padx=10)

        ttk.Radiobutton(
            self.side,
            text="Visible = 2",
            variable=self.visibility_value,
            value=2
        ).pack(anchor="w", padx=20)

        ttk.Radiobutton(
            self.side,
            text="Occluded / uncertain = 1",
            variable=self.visibility_value,
            value=1
        ).pack(anchor="w", padx=20)

        ttk.Button(self.side, text="Mark selected keypoint missing", command=self.mark_keypoint_missing).pack(
            fill=tk.X, padx=10, pady=5
        )

        ttk.Separator(self.side).pack(fill=tk.X, pady=10)

        ttk.Button(self.side, text="Save Annotated + Next", command=self.save_and_next).pack(
            fill=tk.X, padx=10, pady=4
        )
        ttk.Button(self.side, text="Move to Manual Review + Next", command=self.move_to_manual).pack(
            fill=tk.X, padx=10, pady=4
        )
        ttk.Button(self.side, text="Skip", command=self.skip_image).pack(
            fill=tk.X, padx=10, pady=4
        )
        ttk.Button(self.side, text="Previous", command=self.previous_image).pack(
            fill=tk.X, padx=10, pady=4
        )
        ttk.Button(self.side, text="Next", command=self.next_image).pack(
            fill=tk.X, padx=10, pady=4
        )

        ttk.Separator(self.side).pack(fill=tk.X, pady=10)

        help_text = (
            "Shortcuts:\n"
            "B = bbox mode\n"
            "K = keypoint mode\n"
            "1-9 = choose keypoints 1-9\n"
            "Use list for keypoints 10-12\n"
            "Left drag = draw bbox in bbox mode\n"
            "Left click = place keypoint in keypoint mode\n"
            "S = save + next\n"
            "Space = skip\n"
            "Right/Left arrow = next/previous\n\n"
            f"YOLO-pose label format:\n"
            f"class xc yc w h x1 y1 v1 ... x{len(KEYPOINT_NAMES)} y{len(KEYPOINT_NAMES)} v{len(KEYPOINT_NAMES)}"
        )

        ttk.Label(self.side, text=help_text, justify=tk.LEFT, wraplength=360).pack(
            anchor="w", padx=10, pady=5
        )

        self.output_label = ttk.Label(
            self.side,
            text=f"Output:\n{self.paths['out_root']}",
            wraplength=360
        )
        self.output_label.pack(anchor="w", padx=10, pady=10)

    def bind_events(self):
        self.canvas.bind("<ButtonPress-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)
        self.canvas.bind("<ButtonPress-3>", self.cancel_temp_box)
        self.canvas.bind("<Configure>", self.on_resize)

        self.root.bind("b", lambda e: self.mode.set("bbox"))
        self.root.bind("B", lambda e: self.mode.set("bbox"))
        self.root.bind("k", lambda e: self.mode.set("keypoint"))
        self.root.bind("K", lambda e: self.mode.set("keypoint"))

        # Number shortcuts choose keypoints 1-9.
        # For keypoints 10-12, use the keypoint list on the right panel.
        for i in range(min(9, len(KEYPOINT_NAMES))):
            self.root.bind(str(i + 1), lambda e, idx=i: self.set_selected_keypoint(idx))

        self.root.bind("s", lambda e: self.save_and_next())
        self.root.bind("S", lambda e: self.save_and_next())
        self.root.bind("<space>", lambda e: self.skip_image())
        self.root.bind("<Right>", lambda e: self.next_image())
        self.root.bind("<Left>", lambda e: self.previous_image())
        self.root.bind("<Escape>", self.cancel_temp_box)

    def set_selected_keypoint(self, idx):
        if idx < 0 or idx >= len(KEYPOINT_NAMES):
            return

        self.selected_keypoint.set(idx)

        if hasattr(self, "keypoint_list"):
            self.keypoint_list.selection_clear(0, tk.END)
            self.keypoint_list.selection_set(idx)
            self.keypoint_list.see(idx)

    def on_keypoint_select(self, event):
        if not hasattr(self, "keypoint_list"):
            return

        selection = self.keypoint_list.curselection()

        if selection:
            self.selected_keypoint.set(int(selection[0]))

    def apply_class_to_selected(self):
        if self.selected_object_index is None:
            messagebox.showinfo("No glove selected", "Select or draw a glove bbox first.")
            return

        if 0 <= self.selected_object_index < len(self.annotations):
            self.annotations[self.selected_object_index]["class_id"] = int(self.current_class_id.get())
            self.redraw()

    def new_empty_annotation(self, x1, y1, x2, y2):
        return {
            "class_id": int(self.current_class_id.get()),
            "x1": float(x1),
            "y1": float(y1),
            "x2": float(x2),
            "y2": float(y2),
            "keypoints": [
                {"x": None, "y": None, "v": 0}
                for _ in KEYPOINT_NAMES
            ]
        }

    def load_image(self):
        img_path = self.image_paths[self.index]

        self.original_img = Image.open(img_path).convert("RGB")
        self.img_w, self.img_h = self.original_img.size

        self.annotations = self.load_existing_pose_labels(img_path)

        if self.annotations:
            self.selected_object_index = 0
            self.current_class_id.set(int(self.annotations[0].get("class_id", 0)))
        else:
            self.selected_object_index = None

        self.start_img_xy = None
        self.temp_rect_id = None

        self.file_label.config(text=img_path.name)
        self.progress_label.config(text=f"Frame {self.index + 1} of {len(self.image_paths)}")

        self.redraw()

    def load_existing_pose_labels(self, img_path):
        label_path = self.paths["annotated_labels"] / f"{img_path.stem}.txt"

        if not label_path.exists():
            return []

        annotations = []

        try:
            lines = label_path.read_text(encoding="utf-8").splitlines()

            for line in lines:
                parts = line.strip().split()

                if len(parts) < 5 + len(KEYPOINT_NAMES) * 3:
                    continue

                class_id = int(float(parts[0]))
                xc, yc, bw, bh = map(float, parts[1:5])

                x1 = (xc - bw / 2) * self.img_w
                y1 = (yc - bh / 2) * self.img_h
                x2 = (xc + bw / 2) * self.img_w
                y2 = (yc + bh / 2) * self.img_h

                ann = self.new_empty_annotation(x1, y1, x2, y2)
                ann["class_id"] = class_id

                kp_values = parts[5:]

                for i in range(len(KEYPOINT_NAMES)):
                    px = float(kp_values[i * 3])
                    py = float(kp_values[i * 3 + 1])
                    v = int(float(kp_values[i * 3 + 2]))

                    if v > 0:
                        ann["keypoints"][i] = {
                            "x": px * self.img_w,
                            "y": py * self.img_h,
                            "v": v
                        }

                annotations.append(ann)

        except Exception as exc:
            print("Could not load label:", label_path, exc)

        return annotations

    def on_resize(self, event):
        if self.original_img is not None:
            self.redraw()

    def redraw(self):
        if self.original_img is None:
            return

        self.canvas.delete("all")

        canvas_w = max(self.canvas.winfo_width(), 10)
        canvas_h = max(self.canvas.winfo_height(), 10)

        scale_x = canvas_w / self.img_w
        scale_y = canvas_h / self.img_h
        self.scale = min(scale_x, scale_y)

        display_w = int(self.img_w * self.scale)
        display_h = int(self.img_h * self.scale)

        self.offset_x = (canvas_w - display_w) // 2
        self.offset_y = (canvas_h - display_h) // 2

        self.display_img = self.original_img.resize((display_w, display_h), Image.LANCZOS)
        self.tk_img = ImageTk.PhotoImage(self.display_img)

        self.canvas.create_image(self.offset_x, self.offset_y, anchor=tk.NW, image=self.tk_img)

        for idx, ann in enumerate(self.annotations):
            self.draw_annotation(ann, selected=(idx == self.selected_object_index))

        self.update_object_list()

    def draw_annotation(self, ann, selected=False):
        x1, y1 = self.image_to_canvas(ann["x1"], ann["y1"])
        x2, y2 = self.image_to_canvas(ann["x2"], ann["y2"])

        outline = "#00ff66" if not selected else "#ffcc00"
        width = 2 if not selected else 3

        self.canvas.create_rectangle(x1, y1, x2, y2, outline=outline, width=width)

        class_id = int(ann.get("class_id", 0))
        label = CLASS_NAMES[class_id] if 0 <= class_id < len(CLASS_NAMES) else f"class_{class_id}"
        label_w = max(120, min(230, 10 * len(label) + 20))

        self.canvas.create_rectangle(x1, y1 - 22, x1 + label_w, y1, fill=outline, outline=outline)
        self.canvas.create_text(
            x1 + 4,
            y1 - 11,
            text=label,
            anchor=tk.W,
            fill="black",
            font=("Segoe UI", 9, "bold")
        )

        # Skeleton lines
        for a, b in SKELETON_EDGES:
            kp_a = ann["keypoints"][a]
            kp_b = ann["keypoints"][b]

            if kp_a["v"] > 0 and kp_b["v"] > 0:
                ax, ay = self.image_to_canvas(kp_a["x"], kp_a["y"])
                bx, by = self.image_to_canvas(kp_b["x"], kp_b["y"])

                self.canvas.create_line(ax, ay, bx, by, fill="#ffffff", width=2)

        # Keypoints
        for i, kp in enumerate(ann["keypoints"]):
            if kp["v"] <= 0:
                continue

            cx, cy = self.image_to_canvas(kp["x"], kp["y"])

            color_rgb = KEYPOINT_COLORS[i]
            color_hex = "#%02x%02x%02x" % color_rgb

            r = 5 if kp["v"] == 2 else 4

            self.canvas.create_oval(
                cx - r,
                cy - r,
                cx + r,
                cy + r,
                fill=color_hex,
                outline="black",
                width=1
            )

            self.canvas.create_text(
                cx + 7,
                cy - 7,
                text=str(i + 1),
                fill=color_hex,
                font=("Segoe UI", 9, "bold")
            )

    def update_object_list(self):
        self.object_list.delete(0, tk.END)

        for i, ann in enumerate(self.annotations):
            visible_count = sum(1 for kp in ann["keypoints"] if kp["v"] > 0)
            class_id = int(ann.get("class_id", 0))
            class_name = CLASS_NAMES[class_id] if 0 <= class_id < len(CLASS_NAMES) else f"class_{class_id}"
            text = f"{i + 1}. {class_name} | keypoints {visible_count}/{len(KEYPOINT_NAMES)}"

            self.object_list.insert(tk.END, text)

        if self.selected_object_index is not None and self.selected_object_index < len(self.annotations):
            self.object_list.selection_clear(0, tk.END)
            self.object_list.selection_set(self.selected_object_index)

    def on_object_select(self, event):
        selection = self.object_list.curselection()

        if selection:
            self.selected_object_index = int(selection[0])
            ann = self.annotations[self.selected_object_index]
            self.current_class_id.set(int(ann.get("class_id", 0)))
            self.redraw()

    def image_to_canvas(self, x, y):
        cx = self.offset_x + x * self.scale
        cy = self.offset_y + y * self.scale
        return cx, cy

    def canvas_to_image(self, cx, cy):
        x = (cx - self.offset_x) / self.scale
        y = (cy - self.offset_y) / self.scale

        x = max(0, min(self.img_w - 1, x))
        y = max(0, min(self.img_h - 1, y))

        return x, y

    def is_inside_image(self, cx, cy):
        x_min = self.offset_x
        y_min = self.offset_y
        x_max = self.offset_x + self.img_w * self.scale
        y_max = self.offset_y + self.img_h * self.scale

        return x_min <= cx <= x_max and y_min <= cy <= y_max

    def on_mouse_down(self, event):
        if not self.is_inside_image(event.x, event.y):
            return

        if self.mode.get() == "bbox":
            self.start_img_xy = self.canvas_to_image(event.x, event.y)

            if self.temp_rect_id is not None:
                self.canvas.delete(self.temp_rect_id)
                self.temp_rect_id = None

        elif self.mode.get() == "keypoint":
            self.place_keypoint(event.x, event.y)

    def on_mouse_drag(self, event):
        if self.mode.get() != "bbox":
            return

        if self.start_img_xy is None:
            return

        end_img_xy = self.canvas_to_image(event.x, event.y)

        x1, y1 = self.image_to_canvas(*self.start_img_xy)
        x2, y2 = self.image_to_canvas(*end_img_xy)

        if self.temp_rect_id is not None:
            self.canvas.delete(self.temp_rect_id)

        self.temp_rect_id = self.canvas.create_rectangle(
            x1,
            y1,
            x2,
            y2,
            outline="#ffcc00",
            width=2,
            dash=(4, 2)
        )

    def on_mouse_up(self, event):
        if self.mode.get() != "bbox":
            return

        if self.start_img_xy is None:
            return

        end_img_xy = self.canvas_to_image(event.x, event.y)

        x1, y1 = self.start_img_xy
        x2, y2 = end_img_xy

        x1, x2 = sorted([x1, x2])
        y1, y2 = sorted([y1, y2])

        if self.temp_rect_id is not None:
            self.canvas.delete(self.temp_rect_id)
            self.temp_rect_id = None

        self.start_img_xy = None

        if (x2 - x1) < 10 or (y2 - y1) < 10:
            return

        ann = self.new_empty_annotation(x1, y1, x2, y2)
        self.annotations.append(ann)
        self.selected_object_index = len(self.annotations) - 1
        self.mode.set("keypoint")

        self.redraw()

    def place_keypoint(self, canvas_x, canvas_y):
        if self.selected_object_index is None:
            messagebox.showinfo("No glove selected", "Draw or select a glove bbox first.")
            return

        if not self.is_inside_image(canvas_x, canvas_y):
            return

        img_x, img_y = self.canvas_to_image(canvas_x, canvas_y)

        kp_idx = self.selected_keypoint.get()
        v = self.visibility_value.get()

        self.annotations[self.selected_object_index]["keypoints"][kp_idx] = {
            "x": img_x,
            "y": img_y,
            "v": v
        }

        if self.auto_advance_var.get() and kp_idx < len(KEYPOINT_NAMES) - 1:
            self.set_selected_keypoint(kp_idx + 1)

        self.redraw()

    def mark_keypoint_missing(self):
        if self.selected_object_index is None:
            return

        kp_idx = self.selected_keypoint.get()

        self.annotations[self.selected_object_index]["keypoints"][kp_idx] = {
            "x": None,
            "y": None,
            "v": 0
        }

        self.redraw()

    def cancel_temp_box(self, event=None):
        if self.temp_rect_id is not None:
            self.canvas.delete(self.temp_rect_id)
            self.temp_rect_id = None

        self.start_img_xy = None

    def delete_selected_object(self):
        if self.selected_object_index is None:
            return

        if 0 <= self.selected_object_index < len(self.annotations):
            self.annotations.pop(self.selected_object_index)

        if self.annotations:
            self.selected_object_index = min(self.selected_object_index, len(self.annotations) - 1)
        else:
            self.selected_object_index = None

        self.redraw()

    def clear_all(self):
        if not self.annotations:
            return

        answer = messagebox.askyesno("Clear annotations", "Remove all annotations from this image?")

        if answer:
            self.annotations = []
            self.selected_object_index = None
            self.redraw()

    def save_current(self):
        img_path = self.image_paths[self.index]

        if not self.annotations:
            messagebox.showinfo("Nothing saved", "No annotations on this image.")
            return False

        out_img_path = self.paths["annotated_images"] / img_path.name
        shutil.copy2(img_path, out_img_path)

        label_path = self.paths["annotated_labels"] / f"{img_path.stem}.txt"

        lines = []

        for ann in self.annotations:
            x1 = max(0, min(self.img_w - 1, ann["x1"]))
            y1 = max(0, min(self.img_h - 1, ann["y1"]))
            x2 = max(0, min(self.img_w - 1, ann["x2"]))
            y2 = max(0, min(self.img_h - 1, ann["y2"]))

            x1, x2 = sorted([x1, x2])
            y1, y2 = sorted([y1, y2])

            bw = x2 - x1
            bh = y2 - y1

            if bw <= 0 or bh <= 0:
                continue

            xc = x1 + bw / 2
            yc = y1 + bh / 2

            parts = [
                str(ann["class_id"]),
                f"{xc / self.img_w:.6f}",
                f"{yc / self.img_h:.6f}",
                f"{bw / self.img_w:.6f}",
                f"{bh / self.img_h:.6f}",
            ]

            for kp in ann["keypoints"]:
                v = int(kp["v"])

                if v <= 0 or kp["x"] is None or kp["y"] is None:
                    parts.extend(["0.000000", "0.000000", "0"])
                else:
                    px = max(0, min(self.img_w - 1, kp["x"]))
                    py = max(0, min(self.img_h - 1, kp["y"]))

                    parts.extend([
                        f"{px / self.img_w:.6f}",
                        f"{py / self.img_h:.6f}",
                        str(v)
                    ])

            lines.append(" ".join(parts))

        if not lines:
            messagebox.showinfo("Nothing saved", "No valid annotations found.")
            return False

        label_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

        self.save_preview(img_path)

        print("Saved:", out_img_path)
        print("Saved:", label_path)

        return True

    def save_preview(self, img_path):
        preview = self.original_img.copy()
        draw = ImageDraw.Draw(preview)

        for ann in self.annotations:
            x1 = ann["x1"]
            y1 = ann["y1"]
            x2 = ann["x2"]
            y2 = ann["y2"]

            class_id = int(ann.get("class_id", 0))
            label = CLASS_NAMES[class_id] if 0 <= class_id < len(CLASS_NAMES) else f"class_{class_id}"
            label_w = max(120, min(230, 10 * len(label) + 20))

            draw.rectangle([x1, y1, x2, y2], outline=(0, 255, 0), width=3)
            draw.rectangle([x1, max(0, y1 - 24), x1 + label_w, y1], fill=(0, 255, 0))
            draw.text((x1 + 4, max(0, y1 - 22)), label, fill=(0, 0, 0))

            for a, b in SKELETON_EDGES:
                kp_a = ann["keypoints"][a]
                kp_b = ann["keypoints"][b]

                if kp_a["v"] > 0 and kp_b["v"] > 0:
                    draw.line(
                        [kp_a["x"], kp_a["y"], kp_b["x"], kp_b["y"]],
                        fill=(255, 255, 255),
                        width=3
                    )

            for i, kp in enumerate(ann["keypoints"]):
                if kp["v"] <= 0:
                    continue

                x = kp["x"]
                y = kp["y"]

                color = KEYPOINT_COLORS[i]
                r = 6 if kp["v"] == 2 else 4

                draw.ellipse([x - r, y - r, x + r, y + r], fill=color, outline=(0, 0, 0))
                draw.text((x + 7, y - 7), str(i + 1), fill=color)

        preview_path = self.paths["previews"] / f"{img_path.stem}_preview.jpg"
        preview.save(preview_path, quality=95)

    def save_and_next(self):
        saved = self.save_current()

        if saved:
            self.next_image()

    def move_to_manual(self):
        img_path = self.image_paths[self.index]

        dst = self.paths["manual"] / img_path.name
        shutil.copy2(img_path, dst)

        print("Moved to manual review:", dst)

        self.next_image()

    def skip_image(self):
        if self.annotations:
            answer = messagebox.askyesno(
                "Skip without saving?",
                "This image has annotations. Skip without saving?"
            )
            if not answer:
                return

        self.next_image()

    def next_image(self):
        if self.index >= len(self.image_paths) - 1:
            messagebox.showinfo("Finished", "You are at the last image.")
            return

        self.index += 1
        self.load_image()

    def previous_image(self):
        if self.index <= 0:
            messagebox.showinfo("First image", "You are at the first image.")
            return

        self.index -= 1
        self.load_image()


# ============================================================
# SPLIT CREATION
# ============================================================

def create_70_20_10_split(paths):
    src_images = paths["annotated_images"]
    src_labels = paths["annotated_labels"]
    split_root = paths["split"]

    if split_root.exists():
        answer = messagebox.askyesno(
            "Overwrite split?",
            f"Split folder already exists:\n{split_root}\n\nOverwrite it?"
        )

        if not answer:
            return

        shutil.rmtree(split_root)

    for split in ["train", "val", "test"]:
        (split_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (split_root / "labels" / split).mkdir(parents=True, exist_ok=True)

    pairs = []

    for img_path in sorted(src_images.iterdir()):
        if img_path.suffix.lower() not in IMAGE_EXTS:
            continue

        label_path = src_labels / f"{img_path.stem}.txt"

        if label_path.exists():
            pairs.append((img_path, label_path))

    if not pairs:
        messagebox.showerror("No annotations", "No annotated image-label pairs found.")
        return

    random.seed(42)
    random.shuffle(pairs)

    n = len(pairs)
    n_train = int(n * 0.70)
    n_val = int(n * 0.20)

    split_pairs = {
        "train": pairs[:n_train],
        "val": pairs[n_train:n_train + n_val],
        "test": pairs[n_train + n_val:],
    }

    for split, items in split_pairs.items():
        for img_path, label_path in items:
            shutil.copy2(img_path, split_root / "images" / split / img_path.name)
            shutil.copy2(label_path, split_root / "labels" / split / label_path.name)

    write_pose_data_yaml(split_root)

    write_docs(paths)

    msg = (
        f"YOLO-pose split created.\n\n"
        f"Total: {n}\n"
        f"Train: {len(split_pairs['train'])}\n"
        f"Val: {len(split_pairs['val'])}\n"
        f"Test: {len(split_pairs['test'])}\n\n"
        f"Output:\n{split_root}"
    )

    messagebox.showinfo("Split complete", msg)


# ============================================================
# MAIN UI
# ============================================================

class MainApp:
    def __init__(self, root):
        self.root = root
        self.root.title("GRIP YOLO-Pose Dataset Builder")
        self.root.geometry("950x760")
        self.root.resizable(True, True)

        self.project_root_var = tk.StringVar(value=str(DEFAULT_PROJECT_ROOT))
        self.source_video_root_var = tk.StringVar(value=str(DEFAULT_PROJECT_ROOT))
        self.output_name_var = tk.StringVar(value=DEFAULT_OUTPUT_NAME)

        self.extract_every_var = tk.StringVar(value="0.5")
        self.blur_threshold_var = tk.StringVar(value="8.0")
        self.color_check_var = tk.BooleanVar(value=True)
        self.min_color_ratio_var = tk.StringVar(value="0.0015")
        self.near_dup_var = tk.BooleanVar(value=True)
        self.max_frames_var = tk.StringVar(value="0")

        self.build_ui()

    def build_ui(self):
        title = ttk.Label(
            self.root,
            text="GRIP YOLO-Pose Dataset Builder",
            font=("Segoe UI", 16, "bold")
        )
        title.pack(pady=12)

        frame = ttk.Frame(self.root)
        frame.pack(fill=tk.X, padx=20, pady=5)

        ttk.Label(frame, text="Project root:", width=22).grid(row=0, column=0, sticky="w", pady=6)
        ttk.Entry(frame, textvariable=self.project_root_var, width=78).grid(row=0, column=1, pady=6)
        ttk.Button(frame, text="Browse", command=self.browse_project_root).grid(row=0, column=2, padx=5)

        ttk.Label(frame, text="Source video root:", width=22).grid(row=1, column=0, sticky="w", pady=6)
        ttk.Entry(frame, textvariable=self.source_video_root_var, width=78).grid(row=1, column=1, pady=6)
        ttk.Button(frame, text="Browse", command=self.browse_source_root).grid(row=1, column=2, padx=5)

        ttk.Label(frame, text="Output dataset folder:", width=22).grid(row=2, column=0, sticky="w", pady=6)
        ttk.Entry(frame, textvariable=self.output_name_var, width=30).grid(row=2, column=1, sticky="w", pady=6)

        ttk.Separator(self.root).pack(fill=tk.X, padx=20, pady=12)

        settings = ttk.Frame(self.root)
        settings.pack(fill=tk.X, padx=20)

        ttk.Label(settings, text="Extract every seconds:", width=24).grid(row=0, column=0, sticky="w", pady=4)
        ttk.Entry(settings, textvariable=self.extract_every_var, width=12).grid(row=0, column=1, sticky="w", pady=4)

        ttk.Label(settings, text="Blur threshold:", width=24).grid(row=1, column=0, sticky="w", pady=4)
        ttk.Entry(settings, textvariable=self.blur_threshold_var, width=12).grid(row=1, column=1, sticky="w", pady=4)

        ttk.Checkbutton(settings, text="Use glove color check", variable=self.color_check_var).grid(
            row=2, column=0, sticky="w", pady=4
        )

        ttk.Label(settings, text="Min glove color ratio:", width=24).grid(row=3, column=0, sticky="w", pady=4)
        ttk.Entry(settings, textvariable=self.min_color_ratio_var, width=12).grid(row=3, column=1, sticky="w", pady=4)

        ttk.Checkbutton(settings, text="Reject near-duplicate frames", variable=self.near_dup_var).grid(
            row=4, column=0, sticky="w", pady=4
        )

        ttk.Label(settings, text="Max accepted frames/video:", width=24).grid(row=5, column=0, sticky="w", pady=4)
        ttk.Entry(settings, textvariable=self.max_frames_var, width=12).grid(row=5, column=1, sticky="w", pady=4)
        ttk.Label(settings, text="0 = no limit").grid(row=5, column=2, sticky="w", padx=5)

        ttk.Separator(self.root).pack(fill=tk.X, padx=20, pady=12)

        buttons = ttk.Frame(self.root)
        buttons.pack(fill=tk.X, padx=20, pady=8)

        ttk.Button(buttons, text="1. Extract Frames from Videos", command=self.extract_frames).pack(fill=tk.X, pady=4)
        ttk.Button(buttons, text="1A. Import Existing Image Folders", command=self.import_images).pack(fill=tk.X, pady=4)
        ttk.Button(buttons, text="2. OPEN YOLO-POSE ANNOTATOR", command=self.open_annotator).pack(fill=tk.X, pady=5)
        ttk.Button(buttons, text="3. Create 70/20/10 YOLO-Pose Split", command=self.create_split).pack(fill=tk.X, pady=4)
        ttk.Button(buttons, text="Open Output Folder", command=self.open_output).pack(fill=tk.X, pady=4)

        info = (
            "Workflow:\n"
            "1. Extract frames from raw videos OR import your existing 1000-image folders.\n"
            "2. Annotate each glove with class + bbox + 12 keypoints.\n"
            "3. Create train/val/test split for YOLO-pose.\n\n"
            "For final testing, keep the mixed-glove folder separate from training."
        )

        ttk.Label(self.root, text=info, justify=tk.LEFT).pack(anchor="w", padx=20, pady=8)

    def browse_project_root(self):
        path = filedialog.askdirectory(title="Select project root folder")

        if path:
            self.project_root_var.set(path)

    def browse_source_root(self):
        path = filedialog.askdirectory(title="Select source video root folder")

        if path:
            self.source_video_root_var.set(path)

    def get_paths(self):
        project_root = Path(self.project_root_var.get().strip())
        output_name = self.output_name_var.get().strip()

        if not output_name:
            output_name = DEFAULT_OUTPUT_NAME

        paths = get_output_paths(project_root, output_name)
        ensure_dirs(paths)
        write_docs(paths)

        return paths

    def extract_frames(self):
        try:
            extract_every = float(self.extract_every_var.get())
            blur_threshold = float(self.blur_threshold_var.get())
            min_color_ratio = float(self.min_color_ratio_var.get())
            max_frames = int(self.max_frames_var.get())
        except ValueError:
            messagebox.showerror("Invalid settings", "Check numeric extraction settings.")
            return

        source_root = Path(self.source_video_root_var.get().strip())
        paths = self.get_paths()

        if not source_root.exists():
            messagebox.showerror("Folder not found", f"Source root not found:\n{source_root}")
            return

        extract_frames_from_videos(
            source_root=source_root,
            paths=paths,
            extract_every_seconds=extract_every,
            blur_threshold=blur_threshold,
            use_color_check=self.color_check_var.get(),
            min_color_ratio=min_color_ratio,
            reject_near_duplicates=self.near_dup_var.get(),
            max_frames_per_video=max_frames,
        )

    def import_images(self):
        source_root = Path(self.source_video_root_var.get().strip())
        paths = self.get_paths()

        import_existing_images(source_root, paths)

    def open_annotator(self):
        paths = self.get_paths()

        win = tk.Toplevel(self.root)
        PoseAnnotator(win, paths)

    def create_split(self):
        paths = self.get_paths()
        create_70_20_10_split(paths)

    def open_output(self):
        paths = self.get_paths()
        open_folder(paths["out_root"])


def main():
    root = tk.Tk()
    MainApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
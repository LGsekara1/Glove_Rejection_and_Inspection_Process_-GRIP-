# 🧤 GRIP - YOLO26s-Pose Workflow Experiment

## 📌 Purpose of This Folder

This folder documents the **28th July workflow using YOLO26s-pose** for the GRIP project.

The goal of this experiment is not yet to build the final production model. The goal is to test whether a **pose/keypoint-based approach** is more suitable than direct left/right classification for identifying wrongly placed gloves on a conveyor belt.

The current conclusion is:

```text
YOLO-pose bbox detection works well.
YOLO-pose keypoints are not reliable yet.
More clean and consistent pose annotations are needed before KEEP / REJECT logic can be trusted.
```

---

## 🧠 Why We Moved to YOLO-Pose

Earlier approaches were tested before this pose workflow.

### 1. Normal YOLO left/right detection

A YOLO model was trained to directly detect:

```text
left_glove
right_glove
unclear_glove
```

It performed well on validation data, but it did not generalise reliably on mixed real conveyor videos. The main problems were orientation mismatch, domain shift, motion blur, and cases where the same glove was detected inconsistently.

### 2. YOLO + CNN crop classifier

A second approach used:

```text
YOLO detects glove bbox
↓
CNN classifies cropped glove as left/right/unclear
```

This worked on a small test set but failed on new mixed real videos. The CNN depended too much on crop quality, glove pose, and whether the visible crop contained enough orientation information.

### 3. YOLO-pose approach

The current approach uses:

```text
YOLO-pose detects glove bbox + anatomical keypoints
↓
future geometry logic decides KEEP / REJECT / MANUAL
```

This approach is more explainable because the system can eventually reject a glove based on finger-side geometry rather than only a black-box class label.

---

## 🏭 MVP Assumption

For the MVP, the focus is on **one lane first**.

The factory setup being considered:

```text
Right lane only for MVP
Expected glove on right lane = left glove
Wrong glove on right lane    = right glove
```

The camera is placed over one lane, so the middle line between two lanes is not visible in the camera frame. Therefore, the first pose experiment does not use a lane-center-line rule. Instead, the goal is to learn glove anatomy from keypoints and later use geometry to decide whether the glove orientation matches the expected lane pattern.

---

## 🔬 Current Pose Dataset

This is only a **sanity-test dataset**.

| Item | Value |
|------|-------|
| Total annotated pose images | 52 |
| Left-source images | 26 |
| Right-source images | 26 |
| YOLO class count | 1 |
| YOLO class name | `glove` |
| Keypoints | 7 |
| Split | 70 / 20 / 10 |
| Train images | 36 |
| Validation images | 10 |
| Test images | 6 |

The split was created using `03_create_yolo_pose_split.py`.

Important: `left_source` and `right_source` are only used to balance the split. They are **not YOLO classes**.

---

## 🦴 Keypoint Order Used in This Experiment

The YOLO-pose model was trained with one object class:

```yaml
names:
  0: glove
```

The keypoint order is:

| Index | Keypoint Name | Purpose |
|------:|---------------|---------|
| 0 | `wrist_center` | cuff/wrist reference |
| 1 | `palm_center` | central palm reference |
| 2 | `thumb_tip` | optional, often hidden in palm-down gloves |
| 3 | `index_tip` | finger-side geometry |
| 4 | `middle_tip` | finger order reference |
| 5 | `ring_tip` | finger order reference |
| 6 | `pinky_tip` | opposite side reference |

YOLO-pose label format:

```text
class xc yc w h x1 y1 v1 x2 y2 v2 ... x7 y7 v7
```

Visibility values:

```text
0 = missing
1 = occluded / uncertain
2 = visible
```

Training must avoid single-image flips for now:

```text
fliplr = 0.0
flipud = 0.0
```

Single horizontal or vertical flips can corrupt finger-side geometry unless the keypoint flip mapping is carefully defined and verified.

---

## 📁 Recommended Folder Structure

This experiment folder should be organised like this:

```text
28th July workflow using YOLO26s-pose/
│
├── README.md
├── Yolo26s_pose_GRIP.ipynb
├── requirements.txt
│
├── scripts/
│   ├── 03_create_yolo_pose_split.py
│   ├── test_yolo_pose_video_ui.py
│   └── grip_yolo_pose_dataset_ui.py
│
├── docs/
│   ├── annotation_rules.md
│   ├── keypoint_order.md
│   ├── experiment_log.md
│   ├── failed_attempts.md
│   └── next_steps.md
│
├── results/
│   ├── training_metrics.md
│   ├── pose_video_test_observations.md
│   ├── bbox_good_keypoints_unstable_001.png
│   └── screenshots/
│
├── dataset_notes/
│   ├── dataset_structure.md
│   ├── split_summary.txt
│   └── data_yaml_example.yaml
│
└── models/
    ├── model_info.md
    └── best_pt_drive_link.txt
```

Do not commit raw videos, extracted datasets, large `.pt` weights, or full training run folders directly to GitHub.

---

## 🧪 What Was Tested

### Dataset creation

The initial pose dataset was created from manually annotated frames using a custom Python annotation UI and then split into train/validation/test using a stratified 70/20/10 script.

Current dataset split:

```text
Total: 52
Train: 36
Val:   10
Test:  6
```

### Colab training

The training notebook uses:

```text
Model: YOLO26s-pose
Task: pose
Image size: 768
Batch size: 8
Epochs: 80 for sanity test
Augmentation: mild geometry and brightness changes
Flip augmentation: disabled
```

### Video inference test

The trained `best.pt` was tested on conveyor video using `test_yolo_pose_video_ui.py`.

Observation:

```text
Bounding box detection was strong.
Keypoint placement was not yet accurate enough.
```

---

## 📊 Current Result Summary

| Component | Current Result | Status |
|----------|----------------|--------|
| Glove bbox detection | Detects glove boxes well | ✅ promising |
| Pose keypoints | Keypoints not consistently correct | ⚠️ needs more data |
| KEEP / REJECT logic | Not implemented yet | ⏳ waiting for stable keypoints |
| Robot integration | Not tested in this workflow | ⏳ future step |

Current conclusion:

```text
The pipeline works technically, but the pose model is not yet reliable enough for rejection decisions.
```

This is expected because only 52 pose images were used. The current model is a **proof-of-pipeline sanity test**, not a final model.

---

## 🐛 What Failed or Did Not Work Well

### Direct YOLO left/right classification

The direct class-based model looked good on validation metrics, but failed on mixed real video because the training distribution did not match the real test video well enough.

Main issues:

```text
- orientation/domain shift
- partial gloves
- motion blur
- inconsistent confidence
- same glove sometimes detected in multiple confusing ways
```

### CNN crop classifier

The CNN classifier worked on prepared cropped data but failed on new mixed video.

Main issue:

```text
The CNN depended too much on crop quality and did not robustly understand glove geometry.
```

### Initial YOLO-pose test

The pose model learned glove localisation quickly, but keypoints were unstable.

Main issue:

```text
52 annotated images are not enough for reliable wrist/palm/finger keypoint learning.
```

---

## ⚠️ Known Limitations

1. **Dataset is too small**  
   52 images is enough to test the pipeline but not enough for reliable pose estimation.

2. **Thumb is often hidden**  
   In palm-down factory placement, the thumb may not be visible. This makes `thumb_tip` unreliable for MVP decision logic.

3. **Finger folding affects keypoints**  
   Folded or crumpled gloves should not be forced into normal keypoint labels. They should become `MANUAL` cases later.

4. **Keypoint confidence is not enough**  
   A keypoint can be confidently predicted but semantically wrong. Geometry validation is still required.

5. **No final KEEP / REJECT rule yet**  
   Rejection logic should only be added after keypoint predictions become stable.

---

## ✅ Annotation Rules Going Forward

For each visible glove:

```text
1. Draw tight bbox around the whole glove.
2. Place wrist/palm/finger keypoints consistently.
3. Do not guess hidden finger tips.
4. Mark uncertain or hidden points as occluded/missing.
5. Skip or manual-label crumpled gloves if finger identity is unclear.
```

Most important rule:

```text
Wrong keypoints are worse than missing keypoints.
```

---

## 🔁 Possible Improved Keypoint Design

The current 7-keypoint design includes thumb tip:

```text
wrist_center, palm_center, thumb_tip, index_tip, middle_tip, ring_tip, pinky_tip
```

However, because the thumb is often hidden in palm-down gloves, a better MVP design may be:

```text
0 = wrist_left_edge
1 = wrist_right_edge
2 = palm_center
3 = index_tip
4 = middle_tip
5 = ring_tip
6 = pinky_tip
```

This gives the model a better palm/wrist reference while removing the unreliable thumb point.

Another possible design:

```text
0 = wrist_left_edge
1 = wrist_right_edge
2 = palm_center
3 = index_base_side
4 = pinky_base_side
5 = index_tip
6 = middle_tip
7 = ring_tip
8 = pinky_tip
```

This may improve geometry but requires more annotation effort.

---

## 🚀 Next Steps

### Immediate next steps

- [ ] Annotate 150 to 250 more clean pose images.
- [ ] Keep left-source and right-source examples balanced.
- [ ] Add difficult cases separately, but do not force unclear keypoints.
- [ ] Retrain YOLO26s-pose with `fliplr=0.0`.
- [ ] Test again on video.
- [ ] Compare 7-keypoint design vs improved palm-anchor design.

### Target dataset before judging pose model

| Dataset Stage | Target |
|--------------|--------|
| Minimum useful test | 150 to 250 images |
| Better MVP test | 300 to 600 images |
| Serious model | 1000+ images |

### Only after keypoints improve

- [ ] Implement geometry validation.
- [ ] Add KEEP / REJECT / MANUAL decision logic.
- [ ] Add ByteTrack or another tracker to prevent duplicate decisions.
- [ ] Add robot pick queue timing.
- [ ] Integrate STM32H7 serial communication later.

---

## ▶️ How to Run This Workflow

### 1. Prepare dataset split locally

```bash
python scripts/03_create_yolo_pose_split.py
```

Expected output:

```text
YOLO_pose_MVP_dataset/
└── yolo_pose_70_20_10/
    ├── images/
    │   ├── train/
    │   ├── val/
    │   └── test/
    ├── labels/
    │   ├── train/
    │   ├── val/
    │   └── test/
    └── data.yaml
```

### 2. Zip dataset and upload to Colab

Zip only:

```text
yolo_pose_70_20_10/
```

Upload it to Colab and run `Yolo26s_pose_GRIP.ipynb`.

### 3. Fix `data.yaml` path in Colab

If the YAML still contains a Windows path such as `D:/...`, rewrite it to the extracted Colab path before training.

Correct example:

```yaml
path: /content/grip_pose_dataset/yolo_pose_70_20_10
train: images/train
val: images/val
test: images/test
kpt_shape: [7, 3]
flip_idx: [0, 1, 2, 3, 4, 5, 6]
names:
  0: glove
```

### 4. Train YOLO26s-pose

Example training configuration:

```python
from ultralytics import YOLO

model = YOLO("yolo26s-pose.pt")

model.train(
    data="/content/grip_pose_dataset/yolo_pose_70_20_10/data.yaml",
    epochs=80,
    imgsz=768,
    batch=8,
    patience=20,
    fliplr=0.0,
    flipud=0.0,
    mosaic=0.0,
    mixup=0.0,
    project="/content/grip_pose_runs",
    name="grip_yolo_pose_mvp_test"
)
```

### 5. Test on video locally

```bash
python scripts/test_yolo_pose_video_ui.py
```

Use the downloaded `best.pt` from Colab.

---

## 📦 Model Handling

Do not commit large model files directly.

Recommended:

```text
models/
├── model_info.md
└── best_pt_drive_link.txt
```

`model_info.md` should include:

```text
Model: YOLO26s-pose
Dataset: 52 annotated images
Train/Val/Test: 36/10/6
Purpose: sanity test only
Result: bbox good, keypoints unstable
```

---

## 🧹 GitHub Hygiene

Add this to `.gitignore`:

```gitignore
# raw data and videos
*.mp4
*.avi
*.mov
*.mkv
*.zip
*.rar
raw_videos/
frames_to_label/
annotated/images/
annotated/labels/
yolo_pose_70_20_10/images/
yolo_pose_70_20_10/labels/

# model weights
*.pt
*.onnx
*.engine
*.keras
*.h5

# training outputs
runs/
grip_pose_runs/
wandb/
__pycache__/
.ipynb_checkpoints/
```

Keep only documentation, code, example YAML files, small result screenshots, and result summaries in GitHub.

---

## 🧭 Final Current Conclusion

This workflow proves that YOLO-pose can detect glove bounding boxes on conveyor video, but the current annotated dataset is too small for reliable keypoint estimation.

The project should continue with YOLO-pose, but the next milestone should be:

```text
Build a larger, cleaner, more consistent pose dataset before implementing robot rejection decisions.
```

Current status:

```text
Detection: promising
Pose keypoints: not reliable yet
Robot decision logic: not ready yet
Next action: improve annotations and retrain
```

---

<div align="center">

**Group MOSFET · ENTC, University of Moratuwa · 2026**

*GRIP - Glove Rejection and Inspection Process*  
*28th July workflow using YOLO26s-pose*

</div>

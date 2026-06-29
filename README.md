# 🧤 GRIP - Glove Rejection and Inspection Process

<div align="center">

![GRIP Banner](https://img.shields.io/badge/GRIP-Glove%20Rejection%20%26%20Inspection%20Process-0D9488?style=for-the-badge&labelColor=0F172A)

[![Status](https://img.shields.io/badge/Status-Active%20MVP%20Research-F59E0B?style=flat-square)](.)
[![Current Direction](https://img.shields.io/badge/Current%20Direction-YOLO--Pose%20Keypoints-2563EB?style=flat-square)](.)
[![Dataset](https://img.shields.io/badge/Dataset-Factory%20Video%20Frames-0D9488?style=flat-square)](.)
[![Platform](https://img.shields.io/badge/Platform-PC%20→%20Jetson%20Orin%20Nano-7C3AED?style=flat-square)](.)
[![University](https://img.shields.io/badge/ENTC-University%20of%20Moratuwa-1B3A6B?style=flat-square)](.)

**Automated Computer Vision + SCARA Robotic Quality Control System**  
*Real-time glove orientation inspection, defect detection, and belt-synchronised rejection*

---

### Group MOSFET · ENTC, University of Moratuwa · 2026

| Index | Name |
|-------|------|
| 230212H | L.U.A. Gunasekara |
| 230171E | C.D. Elapatha |
| 230470U | T.S.R. Peiris |
| 230318M | J.H.D. Kariyawasam |
| 230507R | M.F.A. Rahman |

</div>

---

## 📌 Project Status

GRIP is an **ongoing engineering prototype**. The project has gone through multiple computer vision approaches, including normal YOLO object detection, left/right object classification, CNN crop classification, and the current YOLO-pose/keypoint-based approach.

The latest direction is:

```text
YOLO-pose detects glove + anatomical keypoints
↓
geometry-based logic decides KEEP / REJECT / MANUAL
↓
robot pick command is queued only for confident reject cases
```

The current model is **not production-ready yet**. Detection bounding boxes are working well, but keypoint prediction still needs more consistent annotated data before reliable rejection logic can be added.

---

## 🎯 Project Overview

GRIP, short for **Glove Rejection and Inspection Process**, is a real-time quality inspection system for nitrile glove manufacturing conveyor lines. The system uses a top-down camera to inspect gloves moving on a conveyor and identifies gloves that should be rejected or sent for manual inspection.

The long-term objective is to integrate:

```text
Camera
↓
Computer vision model
↓
Decision logic
↓
SCARA robot
↓
Pneumatic/vacuum gripper
↓
Reject/manual inspection bin
```

The system is designed so that normal gloves continue on the belt, while wrong-hand gloves, label defects, and uncertain cases are removed or flagged without stopping the conveyor.

---

## ❗ Problem Being Solved

| Problem | Effect on Production |
|--------|----------------------|
| Left/right glove mix-up | Packaging errors and customer complaints |
| Label defects | Smudged, missing, or unclear printed information |
| Manual visual inspection | Fatigue, inconsistency, and missed defects |
| High belt speed | Difficult to inspect accurately by hand |
| No automatic defect logging | Hard to track defect rate and improve process |

The initial production target is not to solve every defect class at once. The MVP focuses on **wrong-hand glove rejection** first, then expands to label defect detection and size verification.

---

## 🏭 Factory MVP Scenario

The real conveyor setup has two lanes on a wide conveyor. For the first MVP, the system focuses on **one lane only**.

```text
Right-side lane only
Expected glove: left glove
Wrong glove: right glove
Camera: top-down, fixed over one lane
Glove orientation: palm-down as much as possible
```

The centre line between the two conveyor lanes is **not visible** in the camera view because the camera is mounted over only one lane. Therefore, the current approach does not depend on a visible centre line.

### Current data limitation

Early right-glove data was collected from the **left-side lane**, not from the right-side lane where right gloves would be wrong. As a temporary MVP workaround, the project uses:

```text
left-lane right-glove data
↓
180° rotation
↓
synthetic right-glove-on-right-lane wrong case
```

This is useful for early experiments, but it is not treated as final production-quality data. The project still needs real right gloves placed on the right lane for validation.

---

## 🏗 System Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                        SENSING LAYER                        │
│  Top-down USB/global shutter camera                         │
│  Conveyor belt                                               │
│  Rotary encoder, planned                                     │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                    COMPUTER VISION LAYER                    │
│  Current: YOLO-pose                                         │
│  Output: bbox + glove keypoints                             │
│  Planned decision output: KEEP / REJECT / MANUAL            │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                     CONTROL LAYER                           │
│  PC decision queue                                          │
│  UART JSON command to STM32H7                               │
│  Encoder-based pick timing                                  │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                      ACTUATION LAYER                        │
│  SCARA robot arm                                             │
│  Pneumatic/vacuum gripper                                    │
│  Reject bin / manual bin                                     │
└─────────────────────────────────────────────────────────────┘
```

---

## 🧠 Computer Vision Development Timeline

### 1. Normal YOLO detection

Initial work trained YOLO-style object detection models to detect gloves in factory frames.

**Outcome**

- Bounding box detection worked well.
- The model could usually locate gloves in the scene.
- This remains useful as a baseline.

**Limitation**

- Detection alone does not reliably determine whether the glove is left or right.

---

### 2. YOLO left/right object classification

The next approach used YOLO with classes such as:

```text
left_glove
right_glove
unclear_glove
```

**Outcome**

- Validation metrics looked promising on the prepared dataset.
- Some trained models gave strong mAP values on small validation sets.

**Failure mode**

On real mixed conveyor videos, the model did not generalise reliably. Problems included domain shift, motion blur, camera angle differences, partial gloves near frame edges, different placement/orientation of gloves, missed detections, and overconfidence on wrong classes.

**Conclusion**

Normal YOLO classification was useful for learning, but not robust enough for the final MVP decision.

---

### 3. YOLO + CNN crop classifier

A two-stage pipeline was tested:

```text
YOLO detects glove bbox
↓
crop glove
↓
CNN classifies left_glove / right_glove / unclear_glove
```

**Observed result**

The CNN achieved around **86.7% test accuracy** on a small test split. In that small test set, left/right confusion was low, but the `unclear_glove` class was weak.

**Failure mode**

When tested on a new mixed real video, the CNN-based pipeline did not perform well enough. The problem was that the CNN depended heavily on crop quality. If YOLO cropped a glove partially, or if the glove was blurred, folded, or placed differently, the CNN prediction became unreliable.

**Conclusion**

The CNN route is not the preferred final approach. It may still be useful for small auxiliary classification tasks, but not as the main left/right decision system.

---

### 4. Current direction: YOLO-pose

The current approach uses YOLO-pose to detect a glove and predict anatomical keypoints.

```text
YOLO-pose
↓
bbox + wrist/palm/finger keypoints
↓
geometry-based decision
```

This is more explainable because the system can reject a glove based on finger-side geometry rather than relying on a black-box class label.

---

## 🦴 Current YOLO-Pose Direction

### Current object class

The pose model uses only one object class:

```text
0 = glove
```

It does **not** use `left_glove` and `right_glove` as YOLO classes.

### Current 7-keypoint schema

```text
0 = wrist_center
1 = palm_center
2 = thumb_tip
3 = index_tip
4 = middle_tip
5 = ring_tip
6 = pinky_tip
```

### Current result

The first YOLO-pose model was trained with only about **52 annotated images**:

```text
26 right-glove frames
26 left-glove frames
```

This was enough to test the pipeline but not enough to produce reliable keypoints.

On video testing:

```text
bbox detection: good
keypoint prediction: not reliable enough yet
KEEP/REJECT logic: not ready yet
```

The model can detect the glove box confidently, but it still confuses some keypoint identities, such as finger tips and palm/wrist points.

---

## 🔁 Possible Updated Keypoint Schema

Because the thumb is often hidden in palm-down gloves, the current plan is to reduce dependence on thumb detection.

A possible improved MVP keypoint schema is:

```text
0 = wrist_left_edge
1 = wrist_right_edge
2 = palm_center
3 = index_tip
4 = middle_tip
5 = ring_tip
6 = pinky_tip
```

This gives the model stronger palm/wrist orientation information while avoiding over-reliance on a hidden thumb.

Another simplified version is:

```text
0 = wrist_center
1 = palm_center
2 = index_tip
3 = middle_tip
4 = ring_tip
5 = pinky_tip
```

The next dataset round will decide which schema is easier to annotate consistently and produces more stable predictions.

---

## ✅ What Worked, What Failed

### What worked

| Attempt | Result |
|--------|--------|
| Frame extraction from factory videos | Worked |
| YOLO glove bounding box detection | Worked well |
| Dataset splitting and augmentation scripts | Worked |
| Custom Tkinter annotation/prototype UI | Worked for quick tests |
| CVAT/Roboflow-style keypoint annotation exploration | Useful for understanding pose annotation |
| YOLO-pose training pipeline | Successfully trained and ran inference |
| YOLO-pose video test script | Successfully displayed bbox + skeleton |

### What failed or was not good enough

| Attempt | Problem |
|--------|---------|
| Direct left/right YOLO classes | Failed to generalise well to new mixed video |
| CNN crop classifier | Good small test result, but weak on mixed real video |
| `unclear_glove` CNN class | Too few real examples and poor recall |
| 7-keypoint pose model with 52 images | Bbox good, keypoints unstable |
| Thumb-based decision rule | Thumb often hidden in palm-down gloves |
| Full-lane centre-line logic | Not usable because centre line is not visible in one-lane camera view |
| Large frame extraction at 0.5 s intervals | Produced too many repeated/empty frames |

---

## 📊 Current Results

These are experimental results from the development process and should not be interpreted as final production performance.

| Stage | Result | Status |
|------|--------|--------|
| Glove detection | High mAP on prepared validation data | Useful baseline |
| Left/right YOLO classification | High validation score on prepared data | Not robust enough on new video |
| CNN cropped classifier | 86.7% test accuracy on small split | Not reliable enough in mixed video |
| YOLO-pose sanity test | Bbox good, keypoints unstable | Current active direction |

---

## 📁 Dataset Strategy

### Current recommendation

Do not crop individual gloves before YOLO-pose training. Train on frames similar to deployment view:

```text
raw/fixed-lane frame
↓
label every visible glove
↓
bbox + keypoints per glove
```

### Frame extraction settings

Recommended extraction settings:

```text
Extract every seconds: 2.0
Blur threshold: 5.0 to 8.0
Use glove color check: ON
Min glove color ratio: 0.01
Reject near-duplicate frames: ON
Max accepted frames/video: 80 to 120
```

### Minimum target for next pose dataset

```text
100 to 150 clean correct-glove examples
100 to 150 wrong/synthetic wrong-glove examples
50 difficult or partial examples
```

Better target:

```text
300 to 600 labelled frames
```

Final target:

```text
1000+ consistently annotated glove instances
```

---

## ✍️ Annotation Rules

### General rules

- Annotate every visible glove in a frame.
- Use one class only: `glove`.
- Draw a tight bounding box around the entire glove.
- Do not force keypoints when the anatomy is not visible.
- Bad labels are worse than skipped labels.

### Visibility rules

```text
Visible = clearly identifiable
Occluded/uncertain = partly hidden but reasonably estimable
Missing = cannot identify without guessing
```

### Current keypoint meaning

```text
wrist_center: centre of cuff/wrist side
palm_center: centre of palm body
thumb_tip: thumb tip if visible
index_tip: finger closest to thumb side
middle_tip: middle long finger
ring_tip: between middle and pinky
pinky_tip: smallest outer finger
```

### Training rule

For pose training, disable single-image flips:

```text
fliplr = 0.0
flipud = 0.0
```

Single horizontal or vertical flip can corrupt finger-side geometry unless a correct `flip_idx` is fully verified.

---

## 🧪 Progress Tracker

### Dataset and annotation

- [x] Factory videos recorded
- [x] Frame extraction UI/script created
- [x] Initial left/right normal YOLO dataset created
- [x] Cropped CNN dataset created
- [x] Initial YOLO-pose annotation UI tested
- [x] Initial 52-image YOLO-pose sanity dataset created
- [ ] Collect real right-glove-on-right-lane wrong examples
- [ ] Build clean 300–600 image YOLO-pose dataset
- [ ] Decide final keypoint schema
- [ ] Annotate more consistent pose data
- [ ] Add difficult/manual examples separately

### Model experiments

- [x] YOLO glove detection baseline
- [x] Left/right YOLO class experiment
- [x] CNN crop classifier experiment
- [x] Initial YOLO-pose training test
- [x] YOLO-pose video inference test
- [ ] Retrain YOLO-pose with improved keypoint schema
- [ ] Validate keypoint stability on unseen video
- [ ] Implement geometry-based KEEP/REJECT/MANUAL rule
- [ ] Add tracker to avoid duplicate robot commands

### Hardware and integration

- [ ] Camera exposure/shutter improvement
- [ ] Better lighting setup
- [ ] Conveyor speed measurement verification
- [ ] Camera-to-SCARA distance measurement
- [ ] Rotary encoder integration
- [ ] STM32H7 UART command receiver
- [ ] SCARA pick trajectory
- [ ] Pneumatic/vacuum end-effector test
- [ ] End-to-end pick test

---

## 🐛 Known Issues and Mitigations

### 1. Keypoints are currently unstable

**Status:** Active.

**Cause:** Too few pose annotations and possibly too much dependence on hidden thumb/finger points.

**Mitigation:** Annotate more clean examples, reduce dependence on thumb, consider wrist-edge + palm-center keypoints, and mark hidden keypoints as missing instead of guessing.

### 2. Bounding boxes work better than keypoints

**Status:** Expected for early dataset.

**Cause:** Detecting a glove object is easier than identifying exact finger anatomy.

**Mitigation:** Keep YOLO-pose approach, improve pose labels, and evaluate keypoint predictions visually before adding robot logic.

### 3. CNN route failed on mixed video

**Status:** Deprioritised.

**Cause:** Cropped classifier depended too much on crop quality and did not generalise well.

**Mitigation:** Do not use CNN as the main decision system. Use pose geometry instead.

### 4. Synthetic wrong-glove data may not match reality

**Status:** Active limitation.

**Cause:** Left-lane right-glove data was rotated to simulate right-glove-on-right-lane cases.

**Mitigation:** Use synthetic data only for MVP testing and collect 50–100 real wrong-glove samples on the right lane before final demo.

### 5. Motion blur and camera quality

**Status:** Active.

**Cause:** Current camera/video has motion blur at conveyor speed.

**Mitigation:** Use stronger lighting, reduce exposure time, use a global shutter camera if possible, and include mild blur examples in training.

### 6. Large empty/repeated frame extraction

**Status:** Resolved by settings.

**Cause:** Extracting every 0.5 seconds from long videos creates thousands of frames.

**Mitigation:** Extract every 2 seconds, use colour filtering, cap frames per video, and inspect contact sheets before annotation.

---

## 💻 Installation

```bash
# Clone repository
git clone https://github.com/LGsekara1/Glove_Rejection_and_Inspection_Process_-GRIP-.git
cd Glove_Rejection_and_Inspection_Process_-GRIP-

# Create virtual environment
py -3.11 -m venv grip_env

# Activate on Windows
grip_env\Scripts\activate

# Install PyTorch with CUDA
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Install project dependencies
pip install ultralytics opencv-python numpy albumentations pillow tqdm pyyaml scikit-learn pyserial
```

Check GPU:

```bash
python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

---

## ▶️ Running the Current Experiments

### Extract frames for pose annotation

```bash
python grip_yolo_pose_dataset_ui.py
```

Recommended extraction settings:

```text
Extract every seconds: 2.0
Blur threshold: 5.0 to 8.0
Min glove color ratio: 0.01
Reject near-duplicate frames: ON
Max accepted frames/video: 80 to 120
```

### Create YOLO-pose split

```bash
python 03_create_yolo_pose_split.py
```

Expected output:

```text
YOLO_pose_MVP_dataset/
└── yolo_pose_70_20_10
    ├── images
    │   ├── train
    │   ├── val
    │   └── test
    ├── labels
    │   ├── train
    │   ├── val
    │   └── test
    └── data.yaml
```

### Train YOLO-pose

```python
from ultralytics import YOLO

model = YOLO("yolo26s-pose.pt")

model.train(
    data="data.yaml",
    epochs=80,
    imgsz=768,
    batch=8,
    fliplr=0.0,
    flipud=0.0,
    mosaic=0.0,
    mixup=0.0,
    degrees=10.0,
    translate=0.04,
    scale=0.08,
    optimizer="AdamW"
)
```

### Test YOLO-pose on video

```bash
python test_yolo_pose_video_ui.py
```

This script currently checks pose quality only. It does not yet perform final KEEP/REJECT decisions.

---

## 📂 Suggested File Structure

```text
GRIP/
├── README.md
├── requirements.txt
├── data/
│   ├── raw_videos/
│   ├── extracted_frames/
│   ├── pose_annotations/
│   └── yolo_pose_70_20_10/
│       ├── images/
│       ├── labels/
│       └── data.yaml
│
├── scripts/
│   ├── grip_yolo_pose_dataset_ui.py
│   ├── 03_create_yolo_pose_split.py
│   ├── test_yolo_pose_video_ui.py
│   ├── extract_frames_filter_blur.py
│   └── rotate_right_glove_data.py
│
├── notebooks/
│   ├── train_yolo_detect.ipynb
│   ├── train_cnn_classifier.ipynb
│   └── train_yolo_pose.ipynb
│
├── models/
│   ├── legacy_detect/
│   ├── legacy_cnn/
│   └── pose/
│
├── reports/
│   ├── experiment_logs/
│   ├── confusion_matrices/
│   └── failure_cases/
│
└── hardware/
    ├── stm32/
    ├── scara/
    └── serial_protocol/
```

---

## 🤖 SCARA Integration Plan

The robot integration is not active yet. It will be added after the pose decision logic becomes reliable.

### Planned command format

```json
{
  "cmd": "pick",
  "track_id": 42,
  "decision": "REJECT",
  "reason": "wrong_hand_geometry",
  "bbox": [x1, y1, x2, y2],
  "timestamp": 1720100234.512
}
```

### Planned decision output

| Decision | Meaning |
|---------|---------|
| `KEEP` | Glove is correct for lane |
| `REJECT` | Confident wrong glove |
| `MANUAL` | Uncertain/folded/low-confidence pose |
| `IGNORE` | False detection or too low confidence |

### Pick timing formula

```text
pick_delay_seconds = camera_to_scara_distance_mm / conveyor_speed_mm_s
trigger_pulse = detection_encoder_pulse + distance_pulses + latency_offset
```

The timestamp-based version will be used for early testing. Encoder-based timing is required for final precision.

---

## 🚀 Future Work

### Immediate next steps

1. Decide final keypoint schema.
2. Annotate 300–600 pose frames with consistent keypoints.
3. Collect real right gloves on the right lane.
4. Retrain YOLO-pose.
5. Test keypoint stability on unseen videos.
6. Implement geometry-based `KEEP / REJECT / MANUAL` logic.
7. Add tracking to avoid duplicate reject commands.

### Medium-term steps

- Add label defect detection.
- Add label ROI extraction using bbox or keypoints.
- Add rotary encoder timing.
- Add STM32H7 UART JSON command receiver.
- Add SCARA pick queue.
- Test end-to-end rejection on moving belt.

### Long-term improvements

- Jetson Orin Nano deployment.
- TensorRT model export.
- Multi-lane operation.
- Size classification.
- OEE/quality dashboard.
- Automatic failure-case logging.

---

## 📚 References

- Ultralytics YOLO Pose Estimation Documentation
- Ultralytics YOLO Detection Documentation
- CVAT Skeleton/Keypoint Annotation Workflow
- GRIP Project Proposal and Mid-Evaluation Materials
- Factory video experiments and failure-case logs

---

<div align="center">

**Group MOSFET · ENTC, University of Moratuwa · 2026**

*GRIP - Glove Rejection and Inspection Process*

</div>

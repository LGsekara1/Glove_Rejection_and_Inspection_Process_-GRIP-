# 🧤 GRIP — Glove Rejection and Inspection Process

<div align="center">

![GRIP Banner](https://img.shields.io/badge/GRIP-Glove%20Rejection%20%26%20Inspection%20Process-0D9488?style=for-the-badge&labelColor=0F172A)

[![Status](https://img.shields.io/badge/Status-Stage%202%20Complete-16A34A?style=flat-square)](.)
[![Model](https://img.shields.io/badge/Model-YOLOv8n%20ONNX-2563EB?style=flat-square)](.)
[![mAP50](https://img.shields.io/badge/mAP50-0.980-0D9488?style=flat-square)](.)
[![Platform](https://img.shields.io/badge/Platform-PC%20→%20Jetson%20Orin%20Nano-7C3AED?style=flat-square)](.)
[![University](https://img.shields.io/badge/ENTC-University%20of%20Moratuwa-1B3A6B?style=flat-square)](.)

**Automated Computer Vision + SCARA Robotic Quality Control System**  
*Real-time glove orientation detection, defect inspection, and belt-synchronised pick-and-place*

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

## 📋 Table of Contents

- [Project Overview](#-project-overview)
- [System Architecture](#-system-architecture)
- [Full CV Pipeline](#-full-cv-pipeline)
- [Progress Tracker](#-progress-tracker)
- [Current Results](#-current-results)
- [Known Issues & Fixes](#-known-issues--fixes)
- [Where Things Can Go Wrong](#-where-things-can-go-wrong)
- [Dataset Guide](#-dataset-guide)
- [Hardware](#-hardware)
- [Installation](#-installation)
- [Running the System](#-running-the-system)
- [File Structure](#-file-structure)
- [SCARA Integration](#-scara-integration)
- [Future Improvements](#-future-improvements)
- [Bill of Materials](#-bill-of-materials)

---

## 🎯 Project Overview

GRIP is a real-time automated quality control system for a nitrile glove manufacturing conveyor line. A top-down camera detects gloves passing on a left-hand-only belt. The computer vision pipeline classifies each glove and detects label defects. A SCARA robot arm controlled by an STM32H7 microcontroller physically removes rejected gloves from the belt without stopping production.

### Problem Being Solved

| Problem | Impact |
|---------|--------|
| Right-hand gloves enter left-hand belt | Packaging errors, customer returns |
| 40–60 gloves/min exceeds manual inspection | Inconsistent, fatigue-driven misses |
| Smudged / missing wrist label ink | Regulatory non-compliance (ISO 11193) |
| No audit data logged | Cannot track or improve defect rate |

### Decision Logic

```
For each tracked glove:
│
├── Detector confidence LOW?
│     └── → MANUAL INSPECTION
│
├── Landmarks fail or low keypoint confidence?
│     └── → MANUAL INSPECTION
│
├── Determined as RIGHT glove?
│     └── → PICK_RIGHT_GLOVE → wrong-hand bin
│
├── Size mismatch with batch? (future)
│     └── → PICK_WRONG_SIZE → wrong-size bin
│
├── Label ROI crop fails?
│     └── → MANUAL INSPECTION
│
├── Label classifier confidence LOW?
│     └── → MANUAL INSPECTION
│
├── Label classified as DEFECT?
│     └── → PICK_LABEL_DEFECT → defect bin
│
└── All checks pass?
      └── → PASS — left on belt ✅
```

### Confidence Thresholds

| Confidence | Decision | Action | Indicator |
|------------|----------|--------|-----------|
| ≥ 0.85 | Automatic removal | SCARA picks immediately | 🔴 Red LED |
| 0.60 – 0.84 | Manual inspection | Yellow alert on display | 🟡 Yellow LED |
| < 0.60 | Ignored | Likely false detection | ⚪ None |

---

## 🏗 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        SENSING LAYER                        │
│  ┌──────────────────┐  ┌─────────────┐  ┌───────────────┐  │
│  │  Global Shutter  │  │   Rotary    │  │   Conveyor    │  │
│  │  Camera (USB)    │  │   Encoder   │  │     Belt      │  │
│  └────────┬─────────┘  └──────┬──────┘  └───────────────┘  │
└───────────┼───────────────────┼─────────────────────────────┘
            │ USB               │ GPIO Timer
            ▼                   ▼
┌───────────────────────────────────────────────────────────────────────────┐
│                           HOST PC                                         │
│                                                                           │
│  Stage 1: YOLOv8n ONNX → Glove detection (mAP50: 0.994)                 │
│  Stage 2: YOLO11n-Pose  → Landmark verification → Left/Right             │
│  Stage 3: YOLO11n-cls   → Label ROI defect classification                │
│  ByteTrack              → Glove tracking (duplicate prevention)          │
│  Decision Logic         → PASS / PICK / MANUAL                           │
│  SCARA Queue            → Timed pick commands                            │
│                                                                           │
└──────────────────────────────┬────────────────────────────────────────────┘
                               │ UART JSON @ 115200 baud
                               ▼
┌──────────────────────────────────────────────────────────────┐
│                      STM32H7 MCU                             │
│  Encoder reading  │  Pick timing  │  SCARA control           │
│  Pneumatic valve  │  Bin routing  │  Return-to-home          │
└──────────────────────────────┬───────────────────────────────┘
                               │
              ┌────────────────┼───────────────┐
              ▼                ▼               ▼
        ┌──────────┐   ┌────────────┐   ┌──────────────┐
        │ Right    │   │  Defect    │   │   Manual     │
        │ Glove    │   │   Bin      │   │  Inspect Bin │
        │  Bin     │   │            │   │              │
        └──────────┘   └────────────┘   └──────────────┘
```

### Belt Timing Formula

```
pick_delay_seconds = camera_to_scara_distance_mm / conveyor_speed_mm_s
trigger_pulse      = detection_encoder_pulse + (distance_mm × pulses_per_mm) + latency_offset
```

> **Conveyor speed:** ~0.254 m/s (50 ft/min)  
> **Total PC-side latency:** ~50–60 ms (compensated in encoder offset)  
> **Pick position accuracy:** ±10 mm (timestamp) → ±1 mm (with encoder)

---

## 🔬 Full CV Pipeline

```
1080p top-down camera
         │
         ▼
 Factory video recording
         │
         ├──▶ Dataset A: Full-frame glove detection
         │    - Class: glove (1 class)
         │    - Train: YOLOv8n Detect
         │    - Purpose: Find every glove in scene
         │
         ├──▶ Dataset B: Cropped glove landmark annotations
         │    - 21 keypoints (MediaPipe-style schema)
         │    - Optional: 4 label-corner keypoints (kp 21–24)
         │    - Train: YOLO11n-Pose
         │    - Purpose: Left/right via thumb position + label ROI location
         │
         └──▶ Dataset C: Cropped label ROI classification
              - Classes: non_defect / defect
              - Train: YOLO11n-cls or small CNN
              - Purpose: Detect smudged / missing / bad ink

Real-time inference flow:
──────────────────────────
Full 1080p frame
  → YOLOv8n glove detector + ByteTrack
  → for each tracked glove:
      crop glove ROI
      → YOLO11n-Pose landmark model
          → determine left/right (thumb_tip vs palm_center x)
          → estimate size from palm_width + glove_length landmarks
          → locate label ROI (wrist keypoints or 4-corner keypoints)
      → preprocess label ROI (CLAHE → sharpen → denoise → threshold)
      → YOLO11n-cls label defect classifier
      → final decision
      → if PICK or MANUAL: add to SCARA timing queue with track_id
  → at each frame: check queue, send due commands via UART JSON
```

### 21-Point Keypoint Schema

| Index | Name | Used For |
|-------|------|----------|
| 0 | wrist_center | Base reference, label ROI anchor |
| 1 | thumb_cmc | Thumb base direction |
| 2 | thumb_mcp | Thumb joint |
| 3 | thumb_ip | Thumb bend |
| **4** | **thumb_tip** | **Left/right determination ← critical** |
| 5 | index_mcp | Palm width reference |
| 6–8 | index pip/dip/tip | Finger shape |
| 9 | middle_mcp | Palm center |
| 10–12 | middle pip/dip/tip | Size reference (middle_tip to wrist) |
| 13 | ring_mcp | Palm width |
| 14–16 | ring pip/dip/tip | Finger completeness |
| 17 | little_mcp | Palm width edge |
| 18–20 | little pip/dip/tip | Finger completeness |
| 21–24 | label_tl/tr/br/bl | Direct label crop (optional) |

### Left/Right Rule (verify with your camera orientation)

```python
thumb_x = kpts[4][0]
palm_x  = mean(kpts[5][0], kpts[9][0], kpts[13][0], kpts[17][0])

# ⚠️ VERIFY THIS WITH 10 KNOWN LEFT + 10 KNOWN RIGHT GLOVES
# The correct mapping depends on camera orientation + palm face direction
if thumb_x < palm_x:
    handedness = "right_glove"   # or left — CONFIRM BEFORE HARDCODING
else:
    handedness = "left_glove"
```

### Label ROI Preprocessing Flow

```
label_roi (cropped wrist area)
  → grayscale
  → CLAHE (clipLimit=2.0, tileGridSize=8×8)
  → sharpening kernel [[0,-1,0],[-1,5,-1],[0,-1,0]]
  → median blur (kernel=3)
  → Otsu threshold
  → morphology open/close
  → YOLO11n-cls classifier
```

---

## ✅ Progress Tracker

> Update checkboxes as stages are completed. Add date completed in brackets.

### 📦 Dataset Collection

- [x] Left glove detection dataset — 302 images *(completed)*
- [x] Right glove dataset — 255 images *(completed)*
- [x] Merged dataset — 495 train + 119 val *(completed)*
- [ ] Landmark annotation dataset — target 250–300 cropped gloves
- [ ] Label defect dataset — target 150 defect + 150 non-defect ROI crops
- [ ] Size dataset — S/M/L/XL known samples *(future — do not rush)*
- [ ] Empty conveyor background video
- [ ] Motion blur speed test video
- [ ] Multiple simultaneous gloves video

### 🤖 Model Training

- [x] Stage 1 — Glove detection — `mAP50: 0.994` *(completed)*
- [x] Stage 2 — Left/Right classification (2-class YOLOv8n) — `mAP50: 0.980` *(completed)*
- [ ] Stage 2b — Left/Right via YOLO11n-Pose landmarks *(in progress)*
- [ ] Stage 3 — Label defect classifier (YOLO11n-cls)
- [ ] Verify left/right rule with 10 known samples each orientation
- [ ] Validate pose model flip_idx or disable fliplr during training
- [ ] Export all models to ONNX
- [ ] Export to TensorRT for Jetson deployment *(future)*

### 💻 Software Pipeline

- [x] `extract_frames.py` — video to frames *(completed)*
- [x] `merge_datasets.py` — merge + remap class IDs *(completed)*
- [x] `train_glove.py` — Stage 1 training *(completed)*
- [x] `train_stage2.py` — Stage 2 fine-tuning *(completed)*
- [x] `export_model.py` — ONNX export *(completed)*
- [x] `laptop_detect.py` — live ONNX webcam inference *(completed)*
- [x] `test_video.py` — video file inference with metrics *(completed)*
- [ ] `filter_blur_duplicates.py` — blur + duplicate filtering
- [ ] `split_dataset.py` — proper train/val/test split script
- [ ] `augment_yolo_detect.py` — detection augmentation
- [ ] `augment_classification.py` — label ROI augmentation
- [ ] `crop_gloves_from_detector.py` — generate crops for landmark annotation
- [ ] `label_roi_from_landmarks.py` — label ROI extraction using keypoints
- [ ] `realtime_pipeline.py` — full 3-stage pipeline with SCARA queue
- [ ] `scara_serial.py` — UART JSON command sender

### 🔌 Hardware Integration

- [ ] Camera calibration (px/mm ratio measured)
- [ ] Belt speed verified (currently assumed 0.254 m/s)
- [ ] Camera-to-SCARA distance measured at factory
- [ ] Rotary encoder connected to STM32H7
- [ ] STM32H7 UART receiving JSON commands from PC
- [ ] SCARA pick trajectory programmed
- [ ] Pneumatic valve timing tuned
- [ ] Return-to-home path defined
- [ ] Reject bin positions within SCARA reach confirmed
- [ ] Pick delay calibrated with test object on belt

### 🧪 System Testing

- [x] Laptop webcam live inference working *(completed)*
- [x] Video file test on mixed_dataset.mp4 *(completed)*
- [ ] Fix domain shift issue (orientation mismatch between train/test)
- [ ] Fix NMS double-detection issue (same glove detected twice)
- [ ] Left/right rule verified with known gloves
- [ ] Label defect classifier validated on real defect samples
- [ ] Full pipeline latency measured (target < 100ms total)
- [ ] ByteTrack duplicate prevention verified
- [ ] End-to-end belt integration test
- [ ] Pick timing calibration on live belt

---

## 📊 Current Results

### Stage 1 — Glove Detection

| Metric | Value | Target |
|--------|-------|--------|
| mAP50 | **0.994** | ≥ 0.95 ✅ |
| mAP50-95 | **0.933** | ≥ 0.60 ✅ |
| Precision | **0.933** | ≥ 0.85 ✅ |
| Recall | **0.977** | ≥ 0.85 ✅ |
| Training images | 302 | — |
| Training time | ~50 sec (RTX 4050) | — |

### Stage 2 — Left/Right Classification

| Class | mAP50 | Precision | Recall |
|-------|-------|-----------|--------|
| left_glove | **0.994** | 0.988 | 0.974 |
| right_glove | **0.966** | 0.923 | 0.920 |
| **Overall** | **0.980** | 0.955 | 0.947 |

### Video Test on mixed_dataset.mp4

| Metric | Value |
|--------|-------|
| Frames processed | 6,902 |
| Total detections | 12,648 |
| Left detections | 3,402 |
| Right detections | 9,246 |
| Right glove rate | 73.1% |
| Avg detections/frame | 1.83 |

> ⚠️ **Note:** Results unreliable due to domain shift and double-detection issue. See Known Issues below.

---

## 🐛 Known Issues & Fixes

### Issue 1 — Domain Shift (ACTIVE — HIGH PRIORITY)

**What:** Model trained on portrait-oriented images (478×850px, gloves upright) performs poorly on test video where gloves appear in different orientation or angle.

**Symptom:** Same glove detected as right_glove on top half and left_glove on bottom half. High false classification rate on mixed_dataset.mp4.

**Root cause:** Training data orientation does not match test video orientation. The model learned "left glove looks like X in portrait" but sees gloves at different angle in test video.

**Fix:**
```python
# 1. Tighten NMS to fix double-detection immediately
results = model(frame, conf=0.50, iou=0.45, verbose=False)[0]

# 2. Add rotation augmentation before retraining
transform = A.Compose([
    A.Rotate(limit=15, p=0.5),
    A.RandomRotate90(p=0.3),
    A.RandomBrightnessContrast(p=0.6),
], bbox_params=A.BboxParams(format="yolo", label_fields=["class_labels"]))

# 3. Verify training vs test video frame size matches
import cv2
cap = cv2.VideoCapture("mixed_dataset.mp4")
ret, frame = cap.read()
print(frame.shape)  # should match training image dimensions
```

**Status:** Not yet resolved. Switching to landmark-based left/right removes dependence on orientation for classification.

---

### Issue 2 — NMS Double Detection (ACTIVE — HIGH PRIORITY)

**What:** Same glove is detected twice — once for the top half (classified as one hand) and once for the bottom half (classified as the other hand).

**Symptom:** Two overlapping bounding boxes on same glove with different class labels. One of them typically has very low confidence (0.00–0.30).

**Root cause:** IoU threshold for NMS is too loose, allowing two boxes with partial overlap to survive suppression.

**Fix:**
```python
# Lower the IoU threshold — boxes that overlap more than 45% get merged
results = model(frame, conf=0.50, iou=0.45, verbose=False)[0]

# Also filter out extremely low confidence detections
# The 0.00 confidence box is a symptom — raising conf threshold removes it
results = model(frame, conf=0.55, iou=0.45, verbose=False)[0]
```

**Status:** Quick fix available. Apply immediately before next test.

---

### Issue 3 — flip_idx Not Defined for Pose Model (PENDING — MEDIUM)

**What:** YOLO11n-Pose uses horizontal flip augmentation during training. For glove keypoints, flipping an image swaps left and right — which corrupts the left/right label if flip_idx is not correctly defined.

**Symptom:** Pose model trained with horizontal flip will have confused left/right logic.

**Fix:**
```yaml
# In training args — disable horizontal flip until flip_idx is verified
fliplr: 0.0

# OR define correct flip_idx in glove_pose.yaml
# flip_idx maps each keypoint index to its mirror-symmetric index
# For gloves: thumb side and little finger side swap on horizontal flip
# You must verify this visually with your camera orientation
flip_idx: [0, 17, 18, 19, 20, 13, 14, 15, 16, 9, 10, 11, 12, 5, 6, 7, 8, 1, 2, 3, 4]
```

**Status:** Disable `fliplr` for first training run. Verify and re-enable after left/right rule is confirmed.

---

### Issue 4 — Left/Right Rule Not Verified (PENDING — HIGH)

**What:** The thumb-position rule for determining left vs right glove depends entirely on camera orientation and whether palm faces up or down. Hardcoding the wrong rule flips all classifications.

**Symptom:** Model classifies all left gloves as right and all right gloves as left (100% wrong).

**Fix:**
```python
# Run this verification BEFORE training or deploying landmark model
# Use 10 known left + 10 known right gloves with labels confirmed visually

def verify_left_right_rule(kpts, known_label):
    thumb_x = kpts[4][0]
    palm_x  = np.mean([kpts[i][0] for i in [5, 9, 13, 17]])
    predicted = "right_glove" if thumb_x < palm_x else "left_glove"
    return predicted == known_label

# Run on your 20 known samples and check accuracy
# If accuracy < 50%, swap the rule (left→right, right→left)
```

**Status:** Must verify before landmark model deployment.

---

### Issue 5 — Label ROI Magic Numbers Not Calibrated (PENDING — MEDIUM)

**What:** The 21-keypoint label ROI extraction uses tunable constants that depend on physical setup.

```python
LABEL_OFFSET = 0.10   # may miss label if wrong
ROI_W_FACTOR = 1.15   # may crop too wide or too narrow
ROI_H_FACTOR = 0.75   # may cut off label text
```

**Symptom:** Label ROI crops miss the printed label, showing belt or glove body instead of text.

**Fix:**
```python
# Option A — Use 4 label-corner keypoints (kp 21-24)
# Annotate the exact label corners during pose annotation
# Gives direct, accurate ROI without tunable constants

# Option B — Fixed bbox offset (simpler, works for consistent orientation)
# Since camera is fixed top-down and gloves are always same orientation:
def get_label_roi_from_bbox(frame, x1, y1, x2, y2):
    h = y2 - y1
    w = x2 - x1
    # Label is near wrist = bottom of glove in top-down view
    # Adjust these percentages by visual inspection on real images
    roi_y1 = int(y1 + h * 0.75)   # bottom 25% of glove bbox
    roi_y2 = y2
    roi_x1 = int(x1 + w * 0.10)   # slight inset left
    roi_x2 = int(x2 - w * 0.10)   # slight inset right
    return frame[roi_y1:roi_y2, roi_x1:roi_x2]
```

**Status:** Recommend annotating 4 label-corner keypoints during pose annotation for most reliable ROI.

---

### Issue 6 — Raspberry Pi Storage Constraint (RESOLVED — DEFERRED)

**What:** Pi 5 microSD card ran out of storage installing ultralytics + OpenCV + onnxruntime (~3 GB total).

**Resolution:** CV inference runs on host PC during prototype phase. Pi integration deferred. Production deployment will use Jetson Orin Nano Super (NVMe SSD, 64+ GB).

---

### Issue 7 — Size Classification Premature in Decision Logic (PENDING — LOW)

**What:** The decision tree includes `PICK_WRONG_SIZE` before size classification is trained or validated. This can cause false picks.

**Fix:**
```python
# In realtime_pipeline.py — gate size check behind a flag
SIZE_CLASSIFICATION_ENABLED = False  # set True only after validation

if SIZE_CLASSIFICATION_ENABLED and EXPECTED_BATCH_SIZE is not None:
    if size != EXPECTED_BATCH_SIZE:
        decision = "PICK_WRONG_SIZE"
```

---

## ⚠️ Where Things Can Go Wrong

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| Detector misses gloves | conf too high, poor varied data | Lower conf 0.6→0.4, add rotated/partial glove frames |
| Detector double-detects same glove | NMS/IoU too loose | Set `iou=0.45`, enable ByteTrack to deduplicate |
| High false removal rate | conf threshold too low | Raise removal threshold to 0.85, route uncertain to manual |
| Landmarks unstable / keypoints jumping | Too few annotated examples | Add more crops, 250+ minimum, clear annotation rules |
| Thumb side rule reversed | Camera orientation mismatch | Verify with 10 known samples each hand before hardcoding |
| Label crop misses label | ROI formula wrong or label moves | Add 4 label-corner keypoints in annotation, or use fixed bbox offset |
| Label classifier false defects | Class imbalance, preprocessing too harsh | Balance 1:1 defect/non-defect, reduce sharpening strength |
| Label classifier misses defects | Not enough defect examples | 150+ real defect ROIs minimum, lower classification conf |
| SCARA picks wrong glove | Distance/speed delay mismatch | Measure exact camera-to-pick distance, calibrate delay_offset |
| Same glove picked twice | ByteTrack not running, sent_track_ids not checked | Always check track_id before queuing pick command |
| SCARA activates with no glove | Belt speed varies, encoder not connected | Use encoder pulses not timestamps for trigger |
| Jetson FPS too low | Model too large or input resolution too high | Use YOLO11n, export TensorRT, lower imgsz to 640 |
| Pose model confuses left/right after flip augmentation | flip_idx wrong | Disable fliplr: 0.0 during training until flip_idx verified |
| Label ROI is blurry | Exposure too long, weak lighting | Add LED backlight, increase shutter speed to ≤4ms |
| Motion blur ruins detection | Belt speed + slow shutter | Global shutter camera required, not rolling shutter |
| Two gloves overlap heavily | Close spacing on belt | Send to manual bin, do not guess; flag for operator |

---

## 📁 Dataset Guide

### Clip Recording Order at Factory

| Order | Filename | Purpose |
|-------|----------|---------|
| 1 | `empty_conveyor.mp4` | Background, lighting, false detection test |
| 2 | `left_good_normal.mp4` | Main pass class — left gloves, clean labels |
| 3 | `left_good_varied.mp4` | Random placement, rotation, varied positions |
| 4 | `right_wrong_orientation.mp4` | Wrong-hand detection dataset |
| 5 | `left_label_defect.mp4` | Label defect examples — most important |
| 6 | `right_label_defect.mp4` | Right glove with label defects |
| 7 | `multiple_gloves.mp4` | Multi-object tracking + duplicate prevention |
| 8 | `motion_blur_test.mp4` | Worst-case blur at conveyor speed |
| 9 | `different_sizes.mp4` | S/M/L/XL samples *(collect when available)* |

### Dataset Targets

| Dataset | Raw images | After 3× augmentation | Notes |
|---------|------------|----------------------|-------|
| Glove detector | 300–400 frames | ~1000 images | Include multi-glove frames |
| Landmark (pose) | 250–350 cropped gloves | ~750–1000 crops | Use Roboflow auto-label + Label Assist |
| Label classifier | 150 defect + 150 non-defect | ~900 ROI crops | Balance classes 1:1 |
| Size *(future)* | Known S/M/L/XL samples | Collect later | Do not rush |

> ⚠️ **Critical:** Split real images into train/val/test FIRST. Augment ONLY the training set. Never augment val or test sets — they must represent real factory conditions.

### Annotation Rules

**Dataset A — Glove Detection**
- Draw tight box around each visible glove including all fingers and wrist
- Do NOT use left/right as detector classes — landmarks decide handedness
- Include partially visible gloves at frame edges

**Dataset B — Landmark Pose**
- Upload cropped gloves (output of `08_crop_gloves_from_detector.py`)
- Annotate 21 keypoints per glove — thumb_tip (kp 4) is most critical
- Optionally annotate 4 label corners (kp 21–24) for direct ROI crop
- Annotate 50–80 manually → use Roboflow Label Assist for remainder

**Dataset C — Label Defect**
- Input: cropped label ROI only — not full glove frame
- Classes: `non_defect` (0), `defect` (1)
- Defect examples: ink smudge, missing characters, unclear print, bad ink
- Balance classes as close to 1:1 as possible

---

## 🔧 Hardware

| Component | Specification | Role |
|-----------|--------------|------|
| Host PC (HP Victus) | Core i7, RTX 4050 6GB, 16GB RAM | CV inference + training |
| Camera | Global shutter, 1080p, USB | Frame capture above belt |
| Lens | 12–16mm C-mount at 500–600mm height | ~500mm FOV width |
| Lighting | LED backlight panel under frosted acrylic | Clean silhouette, no motion blur |
| Rotary encoder | Incremental, on belt drive roller | Real-time belt position (±1mm) |
| STM32H7 | ARM Cortex-M7 @ 480MHz | Real-time timing + SCARA control |
| SCARA arm | 4-DOF, 400mm reach, custom build | Pick-and-place |
| Pneumatic vacuum gripper | 30mm suction cup, solenoid valve | Soft grip for irregular glove shape |
| 2020 V-Slot aluminium frame | Modular, straddles belt | Camera + SCARA mounting |

### Camera Settings

```
Resolution : 1920 × 1080
FPS        : 30 (minimum) — 60 preferred
Shutter    : ≤ 4ms (prevents motion blur at 250mm/s belt)
Mount height: 500–600mm above belt surface
px/mm ratio: ~3.84 px/mm at 600mm height (calibrate with ruler)
```

---

## 💻 Installation

```bash
# Clone repository
git clone https://github.com/LGsekara1/Glove_Rejection_and_Inspection_Process_-GRIP-.git
cd Glove_Rejection_and_Inspection_Process_-GRIP-/ML_implementation

# Requires Python 3.11 (NOT 3.12+ — PyTorch wheels unavailable for 3.13+)
py -3.11 -m venv grip_env

# Activate (Windows)
grip_env\Scripts\activate

# Install PyTorch with CUDA 12.1 (RTX 4050)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Verify GPU
python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"

# Install remaining packages
pip install ultralytics opencv-python numpy albumentations onnxruntime pyserial imagehash pillow tqdm scikit-learn
```

> ⚠️ **Python version matters.** PyTorch currently supports up to Python 3.12. Python 3.13/3.14 will fail with `No matching distribution found`.

---

## ▶️ Running the System

### Test camera
```bash
python 00_camera_test.py
```

### Record factory video
```bash
python 01_record_factory_video.py
# Enter clip label when prompted: e.g. left_good_normal
```

### Extract frames from video
```bash
python 02_extract_frames.py
```

### Filter blur + duplicates
```bash
python 03_filter_blur_duplicates.py
# Tune BLUR_THRESHOLD (default 80.0) and HASH_DISTANCE (default 4)
```

### Train glove detector
```bash
yolo task=detect mode=train model=yolo11n.pt data=glove.yaml epochs=150 imgsz=640 batch=16 patience=30
```

### Train pose (landmark) model
```bash
yolo task=pose mode=train model=yolo11n-pose.pt data=glove_pose.yaml epochs=200 imgsz=640 batch=16 patience=40 fliplr=0.0
```

### Train label defect classifier
```bash
yolo task=classify mode=train model=yolo11n-cls.pt data=label_roi_aug epochs=100 imgsz=224 batch=16 patience=25
```

### Export to ONNX
```bash
python export_model.py
# OR via CLI:
yolo export model=best.pt format=onnx imgsz=640
```

### Run live webcam inference
```bash
python laptop_detect.py
```

### Test on video file
```bash
python test_video.py
```

### Run full real-time pipeline (when all models ready)
```bash
python 10_realtime_pipeline.py
```

---

## 📂 File Structure

```
ML_implementation/
│
├── Training/
│   ├── train/
│   │   ├── images/          ← training images
│   │   └── labels/          ← YOLO format .txt labels
│   ├── valid/
│   │   ├── images/
│   │   └── labels/
│   └── data.yaml            ← dataset config
│
├── Training_v2/             ← merged left+right dataset
│   ├── train/images/
│   ├── valid/images/
│   └── data.yaml
│
├── runs/
│   └── detect/runs/
│       ├── glove_v1/weights/best.pt    ← Stage 1: detection
│       ├── glove_v2/weights/best.pt    ← Stage 1b: more data
│       └── glove_v3_leftright/
│           ├── weights/best.pt         ← Stage 2: left/right ⭐
│           └── weights/best.onnx       ← ONNX export
│
├── 00_camera_test.py
├── 01_record_factory_video.py
├── 02_extract_frames.py        (extract_frames.py)
├── 03_filter_blur_duplicates.py
├── 04_split_dataset.py
├── 05_augment_yolo_detect.py
├── 06_augment_classification.py
├── 08_crop_gloves_from_detector.py
├── 09_label_roi_from_landmarks.py
├── 10_realtime_pipeline.py
├── 11_scara_serial.py
│
├── train_glove.py             ← Stage 1 training script
├── train_stage2.py            ← Stage 2 fine-tuning script
├── merge_datasets.py          ← merge left + right datasets
├── export_model.py            ← ONNX export
├── laptop_detect.py           ← live ONNX webcam inference
├── realtime_detect.py         ← alternative live inference
├── test_video.py              ← video file test with metrics
├── test_model.py              ← validation metrics
│
├── .gitignore
└── requirements.txt
```

> ⭐ `glove_v3_leftright/weights/best.pt` is the current best model. Always use this for inference.

---

## 🤖 SCARA Integration

### JSON Command Format (PC → STM32H7)

```json
{
  "cmd"        : "pick",
  "track_id"   : 42,
  "decision"   : "PICK_RIGHT_GLOVE",
  "bin"        : "wrong_hand_bin",
  "x"          : null,
  "y"          : null,
  "timestamp"  : 1720100234.512
}
```

### Bin Routing

| Decision | Bin |
|----------|-----|
| `PICK_RIGHT_GLOVE` | `wrong_hand_bin` |
| `PICK_LABEL_DEFECT` | `defect_bin` |
| `PICK_WRONG_SIZE` | `wrong_size_bin` *(future)* |
| `MANUAL_*` | `manual_bin` |

### Pick Sequence (STM32H7)

```
1. Receive JSON command via UART
2. Store trigger_pulse = detection_pulse + distance_pulses + latency_offset
3. Poll encoder until trigger_pulse reached
4. SCARA moves to pick position
5. End effector descends → pneumatic vacuum activates
6. Vacuum confirmed → ascend with glove
7. SCARA rotates to target bin
8. Vacuum releases → glove deposited
9. SCARA returns to home/standby
10. Send ACK JSON back to PC
```

### Safety Rules

- Always check `track_id` — send only **one** pick command per tracked glove
- If no detection: no robot movement
- Unclear/low-confidence objects: route to `manual_bin`, not `defect_bin`
- Multiple gloves: each gets a separate `track_id` and queue entry

---

## 🚀 Future Improvements

| Priority | Improvement | Requirement |
|----------|-------------|-------------|
| 🔴 HIGH | Defect label detection (Stage 3) | 150+ defect ROI images |
| 🔴 HIGH | Rotary encoder integration (±1mm pick) | Hardware connection to STM32H7 |
| 🟡 MED | Jetson Orin Nano Super deployment | TensorRT export, $249 dev kit |
| 🟡 MED | Size classification (S/M/L/XL) | Real known-size glove samples |
| 🟡 MED | Multi-lane operation | Additional camera + SCARA per lane |
| 🟢 LOW | OEE quality dashboard | Flask/React frontend |
| 🟢 LOW | Predictive maintenance tracking | Data logging + analytics |
| 🟢 LOW | ERP integration | Factory system API access |

---

## 💰 Bill of Materials

| Category | Item | Qty | Unit (LKR) | Total (LKR) |
|----------|------|-----|-----------|-------------|
| **SCARA Arm** | BLDC Outrunner Motor | 2 | 9,410 | 18,820 |
| | FOC-BLDC Servo Controller | 1 | 15,128 | 15,128 |
| | AS5047P Magnetic Encoder | 2 | 873 | 1,746 |
| | AS5600 Encoder Module | 2 | 400 | 800 |
| | 3K Carbon Fibre Tube 20mm | 2 | 8,250 | 16,500 |
| | Gears, Pullys, Belts | — | — | 3,600 |
| | Power Resistors | 1 | 1,069 | 1,069 |
| | 6808 Thin Wall Bearings | 3 | 140 | 420 |
| | Power Supply Unit | 1 | 2,780 | 2,780 |
| | 2020 V-Slot Aluminium Extrusion | 1 | 2,000 | 2,000 |
| | Miscellaneous (nuts, bolts, wires) | — | — | 5,000 |
| | Screen | 1 | 1,950 | 1,950 |
| | PCB Manufacturing + Components | — | — | 50,000 |
| | **SCARA Subtotal** | | | **119,813** |
| **Pneumatic** | Regulator and Filter | 1 | 3,575 | 3,575 |
| | BM52002s 5/2 Solenoid Valve | 1 | 9,985 | 9,985 |
| | 3/2 Solenoid Valve | 1 | 3,256 | 3,256 |
| | Suction Cup 3cm | 1 | 4,985 | 4,985 |
| | Pneumatic Cylinder | 1 | 2,414 | 2,414 |
| | Speed Controllers | 3 | 230 | 230 |
| | Valves, Connectors, Wires | — | — | 7,000 |
| | Power Supply Unit | 1 | 2,780 | 2,780 |
| | **Pneumatic Subtotal** | | | **34,225** |
| **Computer Vision** | Industrial Camera | 1 | 20,000 | 20,000 |
| | Processing Unit (PC amortised) | 1 | 40,000 | 40,000 |
| | **CV Subtotal** | | | **60,000** |
| | | | **TOTAL** | **LKR 214,038** |

**Suggested selling price:** LKR 650,000 – 800,000 (3× cost + installation + margin)

---

## 📚 References

- [Ultralytics YOLO11 Documentation](https://docs.ultralytics.com/models/yolo11/)
- [Ultralytics Pose Estimation](https://docs.ultralytics.com/tasks/pose/)
- [Ultralytics Image Classification](https://docs.ultralytics.com/tasks/classify/)
- [MediaPipe Hand Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker)
- [MediaPipe Hands Landmarks](https://mediapipe.readthedocs.io/en/latest/solutions/hands.html)
- LeYOLO — Caron et al. 2024
- Embedded YOLO — Feng et al. 2021

---

<div align="center">

**Group MOSFET · ENTC, University of Moratuwa · 2026**

*GRIP — Glove Rejection and Inspection Process*

</div>

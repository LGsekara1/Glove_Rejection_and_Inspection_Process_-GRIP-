# 🧤 GRIP - Glove Rejection and Inspection Process

<div align="center">

![GRIP Banner](https://img.shields.io/badge/GRIP-Glove%20Rejection%20%26%20Inspection%20Process-0D9488?style=for-the-badge&labelColor=0F172A)

[![Status](https://img.shields.io/badge/Status-Active%20MVP%20Research-F59E0B?style=flat-square)](.)
[![Latest Vision Workflow](https://img.shields.io/badge/Latest%20Vision-YOLO11n%203--Class%20Detect-2563EB?style=flat-square)](.)
[![Robot](https://img.shields.io/badge/Robot-Parallel%20SCARA-7C3AED?style=flat-square)](.)
[![Control](https://img.shields.io/badge/Control-STM32H7%20%2B%20Encoder-1B3A6B?style=flat-square)](.)
[![Platform](https://img.shields.io/badge/Platform-PC%20Training%20%E2%86%92%20Jetson%20Orin%20Nano-0D9488?style=flat-square)](.)
[![University](https://img.shields.io/badge/ENTC-University%20of%20Moratuwa-334155?style=flat-square)](.)

**Automated Computer Vision + SCARA Robotic Quality Control System**  
*Real-time glove orientation inspection, uncertainty rejection, and belt-synchronised robotic removal*

---

### Group MOSFET · ENTC, University of Moratuwa · 2026

| Index | Name |
|---|---|
| 230212H | L.U.A. Gunasekara |
| 230171E | C.D. Elapatha |
| 230470U | T.S.R. Peiris |
| 230318M | J.H.D. Kariyawasam |
| 230507R | M.F.A. Rahman |

</div>

---

## 📌 Project Summary

**GRIP**, short for **Glove Rejection and Inspection Process**, is an engineering prototype for automating quality control in a nitrile glove manufacturing conveyor line. The system uses a camera-based computer vision model to identify gloves that should pass, be rejected, or be sent to manual inspection. A downstream **parallel SCARA robot** with a pneumatic/vacuum end-effector is planned to remove rejected gloves from the moving belt.

The current MVP focuses on **wrong-hand glove detection** first:

```text
Expected glove on lane  → left glove
Wrong-hand glove        → right glove
Unsafe/unclear image    → manual/recheck
```

The final target system is:

```text
Factory conveyor
↓
Top-down camera
↓
Computer vision model
↓
Decision logic: PASS / REJECT / MANUAL
↓
Tracking + belt timing
↓
STM32H7 controller
↓
Parallel SCARA robot + pneumatic/vacuum gripper
↓
Reject/manual inspection bin
```

---

## 🎯 Problem Being Solved

| Problem | Production impact |
|---|---|
| Left/right glove mix-up | Packaging errors and customer complaints |
| Crumpled or unclear gloves | Risk of unsafe automatic classification |
| Label defects | Smudged, missing, or unreadable printed information |
| Manual inspection | Fatigue, inconsistency, and missed defects |
| High belt speed | Hard to inspect every glove accurately |
| No automatic logging | Difficult to track defect patterns and improve the process |

The project does **not** try to force a prediction when the glove is visually unsafe. The current industrial logic is:

```text
Clear LEFT  → PASS
Clear RIGHT → REJECT / PICK
UNCLEAR     → MANUAL / RECHECK
```

For the MVP, this is safer than trying to achieve unrealistic 100% classification from blurred or crumpled frames.

---

## 🏗 Complete System Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                        SENSING LAYER                        │
│  Top-down camera                                             │
│  Conveyor belt                                               │
│  Rotary encoder, planned for belt position feedback          │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                    COMPUTER VISION LAYER                    │
│  Current MVP: YOLO11n 3-class detect                         │
│  Classes: left_glove, right_glove, unclear_glove             │
│  Output: bbox + class + confidence                           │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                    DECISION + TRACKING LAYER                 │
│  PASS / PICK decision                                        │
│  Centroid tracking to avoid duplicate commands               │
│  Configurable robot trigger line                             │
│  Future: temporal voting over multiple frames                │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                       CONTROL LAYER                         │
│  PC-side live UI and command generation                      │
│  USB-CDC serial link to STM32H7                              │
│  Line-based ASCII PICK command protocol                      │
│  STM32 acknowledgement support                               │
│  Encoder-based pick timing, planned                          │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                      ACTUATION LAYER                        │
│  Parallel SCARA robot arm                                    │
│  Pneumatic/vacuum gripper                                    │
│  Reject bin / manual bin                                     │
└─────────────────────────────────────────────────────────────┘
```

---

## 🤖 Mechanical and Robotic System

The robot mechanism selected for the project is a **parallel SCARA-style mechanism**. The main reason for selecting this mechanism is that the moving mass can be kept lower compared to a conventional serial SCARA design. This is useful for fast, repetitive pick-and-place operations on lightweight products such as gloves.

### Why parallel SCARA?

| Reason | Benefit |
|---|---|
| Motors remain closer to the base | Lower moving mass |
| Better load distribution | More stable high-speed motion |
| Suitable planar workspace | Matches conveyor pick-and-place task |
| Simpler than delta robot for this application | Easier mechanical design and control |
| Good for lightweight repetitive tasks | Suitable for glove rejection |

### Planned end-effector

The end-effector is planned as a **pneumatic/vacuum gripper** because gloves are soft, flexible, and difficult to grip mechanically with a rigid claw.

```text
Approach glove
↓
Lower vacuum cup / soft gripper
↓
Activate vacuum
↓
Lift glove
↓
Move to reject/manual bin
↓
Release vacuum
↓
Return to standby
```

---

## 🔌 Electrical and Control System

The real-time control layer is planned around an **STM32H7 microcontroller**. The reason for separating the control system from the vision system is that computer vision inference is non-deterministic compared to motor and encoder control. The PC/Jetson handles heavy vision processing, while the STM32 handles timing-critical robot control.

### Planned control responsibilities

| Component | Responsibility |
|---|---|
| PC / Jetson | Camera capture, model inference, decision logic, command generation |
| STM32H7 | Encoder counting, timing, motor/actuator control, safety interlocks |
| Rotary encoder | Belt position measurement |
| SCARA motor controllers | Robot joint actuation |
| Solenoid valves | Pneumatic/vacuum gripper control |
| Status display/LEDs | Operator feedback |

### Current PC-to-STM32 command format

The live UI now generates a line-based ASCII command over USB-CDC:

```text
PICK,ID=17,CLASS=RIGHT,X=428,Y=315,CONF=0.942,FRAME=1832
```

For an unclear glove:

```text
PICK,ID=21,CLASS=UNCLEAR,X=506,Y=287,CONF=0.713,FRAME=1901
```

The STM32 can acknowledge a processed command using:

```text
ACK,ID=17
```

Current field meanings:

| Field | Meaning |
|---|---|
| `ID` | Tracked glove ID used to avoid duplicate commands |
| `CLASS` | `RIGHT` or `UNCLEAR` |
| `X`, `Y` | Current glove centre in image pixels |
| `CONF` | YOLO confidence |
| `FRAME` | Video frame index |

> The current `X` and `Y` values are still image-pixel coordinates. Pixel-to-conveyor and conveyor-to-SCARA calibration must be completed before these values are used as robot positions.

### Planned pick timing

```text
pick_delay_seconds = camera_to_scara_distance_mm / conveyor_speed_mm_s
trigger_pulse = detection_encoder_pulse + distance_pulses + latency_offset
```

The timestamp-based approach can be used for early testing. Encoder-based timing is required for accurate final operation.

---

## 👁️ Computer Vision Development Timeline

The vision part went through multiple iterations. The latest result is not the first attempt, but the most practical MVP based on the current dataset and time constraints.

### 1. Normal YOLO glove detection

Initial experiments trained YOLO-style models to detect gloves in factory frames.

**What worked**

- Bounding box detection worked well.
- The model could locate gloves in the scene.
- This remains useful as a base detection stage.

**Limitation**

- Detection alone does not decide left/right orientation.

---

### 2. YOLO left/right object classification

The next approach trained YOLO with classes such as:

```text
left_glove
right_glove
unclear_glove
```

**What worked**

- Validation results looked promising on prepared data.
- It proved that the dataset pipeline and annotation workflow were functional.

**Failure mode**

- Generalisation to new mixed videos was not reliable enough.
- Motion blur, partial gloves, different positions, and camera-domain shift caused missed detections or wrong predictions.

---

### 3. CNN crop classifier

A two-stage approach was also tested:

```text
YOLO detects glove bbox
↓
Crop glove ROI
↓
CNN classifies left_glove / right_glove / unclear_glove
```

**Observed result**

- The CNN crop classifier reached about **86.7% test accuracy** on a small prepared split.

**Why it was not chosen as final**

- It depended heavily on crop quality.
- If the glove was partial, blurred, or cropped badly, the classifier became unreliable.
- It did not perform strongly enough on a separate mixed video.

---

### 4. YOLO-pose/keypoint experiments

YOLO-pose was considered because handedness is a geometry problem. However, the real glove images showed a practical limitation.

**Why pose was difficult**

- Gloves are not real hands.
- They are empty, deformable, folded, and crumpled.
- The thumb is often hidden.
- Sometimes only four fingers are visible.
- Missing or guessed keypoints reduce training quality.

**Conclusion**

Pose is not abandoned, but it should be used later only after a cleaner, consistently annotated keypoint dataset is available.

---

### 5. Latest MVP: YOLO11n 3-class detection

The latest working vision workflow is a **YOLO11n detect model** trained on three object classes:

```text
0 = left_glove
1 = right_glove
2 = unclear_glove
```

This was selected for the urgent MVP because it avoids dependence on incomplete keypoints and gives a direct testable `best.pt` model for mixed video evaluation.

---

## 🆕 Latest Vision and Integration Work Completed

The current GRIP vision stage has progressed from model training into a working live inspection interface with tracking, decision logic, alerts, and STM32 communication support.

### Completed model and testing work

| Step | Completed work |
|---|---|
| 1 | Annotated factory frames using the custom GRIP annotation UI. |
| 2 | Labelled visually unsafe, folded, blurred, or crumpled samples as `unclear_glove`. |
| 3 | Converted the earlier YOLO-pose-style dataset into YOLO detection labels by retaining bounding-box and class information. |
| 4 | Created a 70/20/10 train/validation/test split. |
| 5 | Trained a YOLO11n three-class detection model and saved the best checkpoint as `best.pt`. |
| 6 | Trained and collected additional YOLO11s and YOLO11m checkpoints for comparison. |
| 7 | Built a mixed-video tester with adjustable confidence, image size, NMS IoU, screenshots, and CSV logging. |
| 8 | Built a model-comparison script to compare accuracy metrics and CPU inference speed. |
| 9 | Confirmed that `best.pt` is the fastest current model on the test laptop CPU, at approximately 28 FPS in the comparison run. |
| 10 | Built a complete Tkinter-based live UI for video/camera inference. |

### Current live UI features

The current application is:

```text
grip_live_ui_scara_alert_fixed.py
```

It includes:

- model selection and loading
- camera or video-file input
- live YOLO bounding boxes
- class, confidence, track ID, and centre-pixel display
- adjustable confidence, NMS IoU, input size, and minimum area
- centroid tracking to avoid repeated commands for the same glove
- configurable robot trigger line
- direction options: `either`, `down`, `up`, `left`, and `right`
- live counters for left, right, unclear, and generated commands
- scrollable settings panel
- USB-CDC COM-port discovery and connection
- optional STM32 acknowledgement handling
- safe simulation mode
- armed serial-output mode
- warning beeps and a large red `SCARA PICKS` alert
- manual alert-test button

### Current decision logic

```text
left_glove    → PASS
right_glove   → PICK
unclear_glove → PICK / recheck
```

A PICK command is not generated every frame. The UI tracks each glove and generates a command only when the tracked centre crosses the configured robot trigger line.

### Current safety behaviour

The UI starts in:

```text
SIMULATION
```

In simulation mode, it logs:

```text
SIMULATED TX: PICK,...
```

without transmitting anything to the STM32.

Actual USB-CDC transmission is enabled only after:

1. the STM32 COM port is selected and connected,
2. `ARM OUTPUT: send PICK commands` is checked,
3. the user confirms the safety warning,
4. a right or unclear glove crosses the trigger line.

### Visual and audio alert

When a right glove or unclear glove creates a PICK event, the UI:

- plays a Windows warning sound,
- shows a large red alert in the detection view,
- displays `WRONG GLOVE - SCARA PICKS` or `UNCLEAR GLOVE - SCARA PICKS`,
- displays the corresponding track ID.

### Dataset summary

| Class | Count |
|---|---:|
| `left_glove` | 117 |
| `right_glove` | 114 |
| `unclear_glove` / crumpled | 377 |
| **Total** | **608** |

### Dataset split

| Split | Images |
|---|---:|
| Train | 425 |
| Validation | 121 |
| Test | 62 |
| **Total** | **608** |

The training set was later balanced/expanded for minority classes:

| Stage | Images |
|---|---:|
| Original train split | 425 |
| Balanced training set used by notebook | 825 |

### Current primary model

| Item | Value |
|---|---|
| Model | `best.pt` |
| Architecture | YOLO11n Detect |
| Classes | `left_glove`, `right_glove`, `unclear_glove` |
| Input size | 640 by default |
| Current role | Live detection, tracking, and command generation |
| Current CPU speed | About 28 FPS in the comparison run |
| Deployment status | Working prototype on laptop |

### Model-comparison caution

Several models were compared using the same test source, but part of the converted test-label set produced out-of-range label warnings during Ultralytics validation. Therefore, the current ranking is used mainly as a practical speed and prototype comparison, not as a final production-grade accuracy claim.

`best.pt` is currently preferred because:

- it loads reliably,
- it performs well visually on the mixed factory video,
- it is significantly faster than the YOLO11s and YOLO11m alternatives on CPU,
- it supports the required three classes,
- it integrates successfully with the live UI and tracking pipeline.

## 📊 Latest Vision Results

The latest result files are stored in:

```text
Vision/GRIP_YOLO_detect_results_package/
```

### Validation confusion matrices

<div align="center">

<img src="Vision/GRIP_YOLO_detect_results_package/confusion_matrix.png" alt="Validation Confusion Matrix" width="47%">
<img src="Vision/GRIP_YOLO_detect_results_package/confusion_matrix_normalized.png" alt="Normalized Validation Confusion Matrix" width="47%">

</div>

### Manual test confusion matrix

<div align="center">

<img src="Vision/GRIP_YOLO_detect_results_package/manual_test_confusion_matrix.png" alt="Manual Test Confusion Matrix" width="60%">

</div>

### Dataset label distribution

<div align="center">

<img src="Vision/GRIP_YOLO_detect_results_package/labels.jpg" alt="Dataset Label Distribution" width="60%">

</div>

### Training metric curves

<div align="center">

<img src="Vision/GRIP_YOLO_detect_results_package/metrics_mAP50(B).png" alt="mAP50 Curve" width="47%">
<img src="Vision/GRIP_YOLO_detect_results_package/metrics_mAP50-95(B).png" alt="mAP50-95 Curve" width="47%">

<br>

<img src="Vision/GRIP_YOLO_detect_results_package/metrics_precision(B).png" alt="Precision Curve" width="47%">
<img src="Vision/GRIP_YOLO_detect_results_package/metrics_recall(B).png" alt="Recall Curve" width="47%">

</div>

### Result package files

| File | Purpose |
|---|---|
| `best.pt` | Best validation checkpoint from training |
| `last.pt` | Final epoch checkpoint |
| `best.onnx` | Exported ONNX model for deployment experiments |
| `results.csv` | Epoch-by-epoch training metrics and losses |
| `confusion_matrix.png` | Ultralytics validation confusion matrix |
| `confusion_matrix_normalized.png` | Normalized validation confusion matrix |
| `labels.jpg` | Label distribution and bbox distribution plot |
| `manual_test_predictions.csv` | Object-level predictions on test split |
| `manual_test_confusion_matrix.csv` | CSV version of manual test confusion matrix |
| `manual_test_confusion_matrix.png` | Manual test confusion matrix plot |
| `manual_test_classification_report.csv` | Precision, recall, and F1-score per class |
| `data.yaml` | Dataset class definitions and split paths |
| `args.yaml` | Training arguments/configuration |

> Note: The exact numeric accuracy, precision, recall, F1-score, and mAP values should be read from `results.csv` and `manual_test_classification_report.csv` in the result package. This README intentionally avoids overclaiming production performance because the current dataset is small and video-derived.

---


---

## ✍️ Annotation Rules Used


```text
Label what is visually decidable, not only what the source folder says.
```

| Situation | Label |
|---|---|
| Clearly visible left glove | `left_glove` |
| Clearly visible right glove | `right_glove` |
| Crumpled, folded, blurred, partial, or visually unsafe glove | `unclear_glove` |
| Known left/right from folder but image itself is not clear | `unclear_glove` |

This prevents the model from learning to pass unsafe blurred or crumpled gloves as left/right.

For future pose work:

```text
Visible keypoint      → mark it
Hidden but certain    → mark as occluded only if truly confident
Unknown keypoint      → do not guess
```

Bad keypoints are worse than missing keypoints.

---


## 📚 References and Project Materials

- Ultralytics YOLO Detection Documentation
- Ultralytics YOLO Pose Documentation
- GRIP Project Proposal and Mid-Evaluation Materials
- Factory video recordings and annotation logs
- Google Colab training notebooks
- `Vision/GRIP_YOLO_detect_results_package/` result package

---

<div align="center">

**Group MOSFET · ENTC, University of Moratuwa · 2026**

*GRIP - Glove Rejection and Inspection Process*

</div>

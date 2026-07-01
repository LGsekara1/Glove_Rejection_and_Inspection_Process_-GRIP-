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
│  PASS / REJECT / MANUAL decision                             │
│  Future: tracker to avoid duplicate commands                 │
│  Future: temporal voting over multiple frames                │
└───────────────────────────────┬─────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                       CONTROL LAYER                         │
│  PC or Jetson command queue                                  │
│  JSON command over UART / serial                             │
│  STM32H7 real-time controller                                │
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

### Planned serial command format

```json
{
  "cmd": "pick",
  "track_id": 42,
  "decision": "REJECT",
  "reason": "wrong_hand_or_unclear",
  "bbox": [120, 180, 640, 520],
  "confidence": 0.91,
  "timestamp": 1720100234.512
}
```

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

## 🆕 Latest Vision Work Done Today

Today’s work focused on converting the annotated factory-frame dataset into a complete, testable YOLO detection workflow.

| Step | Completed work |
|---|---|
| 1 | Annotated factory frames using the custom GRIP annotation UI. |
| 2 | Reviewed crumpled/blurred frames and labelled visually unsafe samples as `unclear_glove`. |
| 3 | Decided not to rely on pose for the urgent MVP due to hidden fingers/thumbs. |
| 4 | Used annotated labels but converted them into YOLO detect format by keeping only bbox + class. |
| 5 | Created a 70/20/10 train/validation/test split. |
| 6 | Trained a YOLO11n detect model in Google Colab. |
| 7 | Saved the best validation checkpoint as `best.pt`. |
| 8 | Generated confusion matrices, label plots, mAP curves, precision/recall curves, CSV reports, and ONNX export. |
| 9 | Uploaded the result package into `Vision/GRIP_YOLO_detect_results_package/`. |
| 10 | Prepared a separate video detector tester script for testing `best.pt` on a mixed conveyor video. |

### Annotated dataset before split

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

During Colab training, the training set was balanced/expanded for the minority `left_glove` and `right_glove` classes:

| Stage | Images |
|---|---:|
| Original train split | 425 |
| Balanced training set used by notebook | 825 |

### Training configuration

| Item | Value |
|---|---|
| Model | `yolo11n.pt` |
| Task | YOLO Detect |
| Classes | `left_glove`, `right_glove`, `unclear_glove` |
| Image size | 640 |
| Max epochs | 100 |
| Early stopping patience | 20 |
| Horizontal flip | Disabled |
| Reason for disabling flip | Horizontal flip changes glove handedness and can corrupt labels |

### Most important safety metric

For this project, the most dangerous error is:

```text
true right_glove predicted as left_glove
```

This means a wrong-hand glove could pass as a correct glove. This error matters more than overall accuracy.

---

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

## 📁 Repository Structure

```text
Glove_Rejection_and_Inspection_Process_-GRIP-/
├── README.md
│
├── Vision/
│   ├── GRIP_YOLO_detect_results_package/
│   │   ├── README.md
│   │   ├── best.pt
│   │   ├── best.onnx
│   │   ├── last.pt
│   │   ├── data.yaml
│   │   ├── args.yaml
│   │   ├── results.csv
│   │   ├── labels.jpg
│   │   ├── confusion_matrix.png
│   │   ├── confusion_matrix_normalized.png
│   │   ├── manual_test_predictions.csv
│   │   ├── manual_test_confusion_matrix.csv
│   │   ├── manual_test_confusion_matrix.png
│   │   ├── manual_test_classification_report.csv
│   │   ├── metrics_mAP50(B).png
│   │   ├── metrics_mAP50-95(B).png
│   │   ├── metrics_precision(B).png
│   │   └── metrics_recall(B).png
│   │
│   ├── scripts/
│   │   ├── grip_video_detect_tester.py
│   │   ├── grip_model_tester_ui.py
│   │   ├── grip_yolo_pose_dataset_ui.py
│   │   └── 03_create_yolo_pose_split.py
│   │
│   └── notebooks/
│       ├── GRIP_YOLO_Already_Split_Detect_Colab.ipynb
│       └── GRIP_YOLO_Train_Evaluate_Colab.ipynb
│
├── SCARA/
│   ├── mechanical_design/
│   ├── gearbox/
│   ├── end_effector/
│   └── simulations/
│
├── Control/
│   ├── stm32h7/
│   ├── motor_control/
│   ├── encoder/
│   └── serial_protocol/
│
├── Pneumatics/
│   ├── vacuum_gripper/
│   ├── valves/
│   └── testing/
│
└── Reports/
    ├── proposal/
    ├── mid_evaluation/
    └── final_results/
```

Folder names can be adjusted, but the main structure is:

```text
Vision → perception model and results
SCARA → robot mechanism
Control → STM32H7 and timing logic
Pneumatics → gripper system
Reports → documentation and presentations
```

---

## ✍️ Annotation Rules Used

The most important rule used during today’s dataset preparation was:

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

## ▶️ Running the Latest Vision Model

Place these files in one testing folder:

```text
test_folder/
├── grip_video_detect_tester.py
├── best.pt
└── Mixed_Dataset_video.mp4
```

Install dependencies:

```bash
py -3.11 -m venv grip_test_env
grip_test_env\Scripts\activate
pip install ultralytics opencv-python numpy
```

Run the detector tester:

```bash
python grip_video_detect_tester.py --model best.pt --source Mixed_Dataset_video.mp4 --conf 0.05 --imgsz 960
```

If detections are missing, reduce confidence and increase image size:

```bash
python grip_video_detect_tester.py --model best.pt --source Mixed_Dataset_video.mp4 --conf 0.01 --imgsz 1280
```

The correct model should print:

```text
model.task = detect
model.names = {0: 'left_glove', 1: 'right_glove', 2: 'unclear_glove'}
```

If the window says:

```text
GRIP YOLO-Pose Test
This test checks pose quality only
```

then the wrong tester script is being used for the latest model.

---

## 💻 Installation for Development

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
pip install ultralytics opencv-python numpy albumentations pillow tqdm pyyaml scikit-learn pyserial streamlit
```

Check GPU:

```bash
python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

---

## ✅ Progress Checklist

### Vision system

- [x] Factory videos recorded
- [x] Frame extraction workflow created
- [x] Custom annotation UI tested
- [x] Initial YOLO detection experiments completed
- [x] CNN crop classifier experiment completed
- [x] YOLO-pose/keypoint experiment tested
- [x] 608-image left/right/unclear dataset annotated
- [x] 70/20/10 dataset split created
- [x] YOLO-pose-style labels converted into detect format for urgent MVP
- [x] YOLO11n 3-class detect model trained in Colab
- [x] `best.pt`, `last.pt`, `best.onnx`, confusion matrices, and CSV reports generated
- [x] Latest result package uploaded under `Vision/GRIP_YOLO_detect_results_package/`
- [x] Video tester script prepared for mixed dataset testing
- [ ] Test latest `best.pt` with the correct detector tester on mixed video
- [ ] Save failure cases and false predictions
- [ ] Collect more balanced left/right/unclear examples
- [ ] Add temporal voting over multiple frames
- [ ] Add tracker to avoid duplicate decisions
- [ ] Revisit pose only after a cleaner keypoint dataset is available

### SCARA mechanical system

- [x] Robot mechanism selected as parallel SCARA
- [x] Basic mechanical concept prepared
- [x] Gearbox/end-effector concept prepared
- [ ] Complete CAD refinement
- [ ] Complete mechanical fabrication
- [ ] Test repeatable pick-and-place motion
- [ ] Tune workspace and belt pick position

### Electrical/control system

- [x] STM32H7 selected as real-time control unit
- [x] Encoder-based timing concept prepared
- [x] JSON/UART command concept prepared
- [ ] Implement UART command receiver
- [ ] Implement encoder pulse counting
- [ ] Implement motor control and homing
- [ ] Integrate robot command queue
- [ ] Test PC/Jetson to STM32 communication

### Pneumatic system

- [x] Pneumatic/vacuum gripping approach selected
- [x] Solenoid/vacuum control concept prepared
- [ ] Test suction cup on glove material
- [ ] Tune vacuum pressure and contact time
- [ ] Test pick reliability on flat and crumpled gloves

### Final integration

- [ ] Camera + model + decision pipeline
- [ ] Decision queue + belt timing
- [ ] STM32 command execution
- [ ] SCARA pick-and-place test
- [ ] Manual/reject bin routing
- [ ] End-to-end moving-belt demonstration

---

## 🐛 Known Limitations and Risks

| Risk | Why it matters | Mitigation |
|---|---|---|
| Right glove predicted as left | Most dangerous false pass | Track this separately, tune threshold, add more right-glove samples |
| Dataset imbalance | `unclear_glove` has many more examples | Balance train set and collect more left/right examples |
| Motion blur | Thumb/fingers become unclear | Stronger lighting, shorter exposure, better camera, blur augmentation |
| Partial gloves near frame edge | Model may misclassify incomplete shapes | Use tracking and decide only near a stable decision line |
| Pose/keypoint instability | Hidden glove anatomy breaks keypoint logic | Revisit pose after better keypoint data |
| Domain shift on mixed video | Training videos and test videos may differ | Use separate validation clips and failure-case retraining |
| SCARA timing error | Pick may happen too early or late | Use encoder-based timing and calibration offset |
| Vacuum grip failure | Glove may not lift correctly | Test suction cup size, pressure, and contact time |

---

## 🧪 Viva / Explanation Summary

If asked what was finally done in the computer vision part:

> We built a YOLO-based vision prototype for glove classification on conveyor frames. The final testable model is a YOLO11n detection model trained with three classes: `left_glove`, `right_glove`, and `unclear_glove`. We originally considered YOLO-pose because handedness is a geometry problem, but real gloves are empty, folded, crumpled, and often have hidden thumbs or fingers, so keypoints were not reliable enough for the current dataset. Therefore, for the MVP, we trained a simpler 3-class detection model using 608 annotated factory frames. The dataset was split 70/20/10, trained in Google Colab, and evaluated using confusion matrices, precision, recall, F1-score, mAP curves, and test prediction CSVs. The most important safety metric is not only accuracy, but whether a true right glove is predicted as a left glove, because that would allow a wrong glove to pass.

If asked about the whole system:

> The full project is an automated conveyor-belt inspection and rejection system. The camera detects and classifies gloves, the decision layer decides pass/reject/manual, and the planned parallel SCARA robot removes wrong or uncertain gloves using a pneumatic/vacuum gripper. The STM32H7 handles real-time robot and encoder timing, while the PC or Jetson handles computer vision inference.

---

## 🚀 Next Steps

### Immediate

1. Test the latest `best.pt` using `grip_video_detect_tester.py` on the mixed video dataset.
2. Confirm that the script prints `model.task = detect`.
3. Save screenshots of missed detections and wrong predictions.
4. Count dangerous errors: true `right_glove` predicted as `left_glove`.
5. Add more real right and left glove examples from the same test setup.

### Short term

- Add tracker and temporal voting across 5-10 frames.
- Improve camera lighting and exposure.
- Retrain with a more balanced dataset.
- Build a failure-case dataset from mixed-video testing.

### Medium term

- Integrate detection output with a command queue.
- Implement UART JSON communication to STM32H7.
- Test conveyor timing using measured belt speed or encoder pulses.
- Test pneumatic pick on real gloves.

### Long term

- Deploy model on Jetson Orin Nano.
- Export and benchmark TensorRT.
- Add label defect inspection.
- Add size classification.
- Complete end-to-end SCARA rejection demo.

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

# 🧤 GRIP Vision Results - YOLO11n 3-Class Detect Prototype

<div align="center">

![GRIP Vision](https://img.shields.io/badge/GRIP-Vision%20Model%20Results-0D9488?style=for-the-badge&labelColor=0F172A)

[![Workflow](https://img.shields.io/badge/Workflow-YOLO11n%203--Class%20Detect-2563EB?style=flat-square)](.)
[![Task](https://img.shields.io/badge/Task-Left%20%7C%20Right%20%7C%20Unclear-F59E0B?style=flat-square)](.)
[![Dataset](https://img.shields.io/badge/Dataset-Factory%20Conveyor%20Frames-0D9488?style=flat-square)](.)
[![Model](https://img.shields.io/badge/Model-best.pt%20%2B%20best.onnx-7C3AED?style=flat-square)](.)
[![Status](https://img.shields.io/badge/Status-MVP%20Prototype%20Result-E11D48?style=flat-square)](.)

**Computer Vision results package for the GRIP glove rejection prototype**  
*YOLO11n detection model trained to classify gloves as LEFT, RIGHT, or UNCLEAR*

</div>

---

## 📌 Folder Purpose

This folder contains the **Vision-stage YOLO detect experiment results** for the GRIP project.

Current GitHub location:

```text
Glove_Rejection_and_Inspection_Process_-GRIP-/
└── Vision/
    └── GRIP_YOLO_detect_results_package/
```

This is the latest MVP computer vision workflow where a YOLO11n detection model was trained to identify:

```text
0 = left_glove
1 = right_glove
2 = unclear_glove
```

The purpose of this experiment was to create a quick, testable model from real factory conveyor frames and evaluate whether left/right/unclear glove classification is learnable from the current camera dataset.

---

## 🧠 Why This YOLO Detect Model Was Used

The project previously explored YOLO-pose/keypoints. However, the real glove images are difficult for pose because gloves are not real hands:

- gloves are empty and deformable,
- fingers may be folded together,
- the thumb is often hidden,
- many examples are blurred due to conveyor motion,
- crumpled gloves do not have stable anatomical keypoints.

Because of this, the latest practical MVP uses **YOLO object detection with three classes** instead of relying on keypoints for the urgent test.

The logic is:

```text
Clear LEFT glove     → allow/pass
Clear RIGHT glove    → reject/pick
UNCLEAR glove        → manual/recheck
```

This avoids forcing a risky left/right decision when the visual evidence is poor.

---

## ✅ What Was Completed in This Workflow

| Step | Completed work |
|---:|---|
| 1 | Factory video frames were selected for the first urgent vision model test. |
| 2 | Gloves were annotated with bounding boxes and class labels. |
| 3 | Three classes were used: `left_glove`, `right_glove`, and `unclear_glove`. |
| 4 | Existing YOLO-pose style labels were converted to YOLO detect format by keeping only bbox + class. |
| 5 | A 70/20/10 train/validation/test split was created. |
| 6 | YOLO11n was trained in Google Colab using Ultralytics. |
| 7 | The best validation checkpoint was saved as `best.pt`. |
| 8 | The trained model was exported to ONNX as `best.onnx`. |
| 9 | Confusion matrices, metric curves, prediction CSV files, and classification reports were generated. |
| 10 | A detector video tester script was prepared for mixed-video testing. |

---

## 📊 Dataset Summary

### Annotated class counts before split

| Class | Meaning | Count |
|---|---|---:|
| `left_glove` | Visually clear left glove | 117 |
| `right_glove` | Visually clear right glove | 114 |
| `unclear_glove` | Crumpled, folded, blurred, partial, or unsafe-to-classify glove | 377 |
| **Total** |  | **608** |

### Dataset split

| Split | Images |
|---|---:|
| Train | 425 |
| Validation | 121 |
| Test | 62 |
| **Total** | **608** |

During training in Colab, the training set was balanced/expanded to reduce the strong class imbalance caused by the larger `unclear_glove` class.

| Stage | Images |
|---|---:|
| Original train split | 425 |
| Balanced training set used by notebook | 825 |

> Validation and test sets were kept separate and were not artificially expanded, so they remain useful for evaluation.

---

## ⚙️ Training Configuration

| Item | Value |
|---|---|
| Base model | `yolo11n.pt` |
| Task | YOLO object detection |
| Input image size | 640 |
| Classes | `left_glove`, `right_glove`, `unclear_glove` |
| Max epochs | 100 |
| Early stopping patience | 20 |
| Best model file | `best.pt` |
| Final model file | `last.pt` |
| ONNX export | `best.onnx` |
| Horizontal flip | Disabled |
| Reason for disabling flip | Flipping can change apparent glove handedness and corrupt labels |

---

## 🚨 Most Important Safety Metric

For this industrial use case, the most dangerous error is:

```text
true right_glove predicted as left_glove
```

Why?

A right glove predicted as left could incorrectly pass along the left-glove lane and create a packaging error. Therefore, this workflow evaluates not only general accuracy, but also the **dangerous right-as-left error**.

General model accuracy is useful, but for deployment the priority is:

```text
Minimize RIGHT → LEFT false passes
```

Uncertain or unclear cases should be sent to manual/recheck instead of being forced into a left/right decision.

---

## 📈 Vision Results

The result images below are generated from the training and evaluation workflow and are stored in this same folder.

### 1. Validation Confusion Matrices

<div align="center">

<img src="./confusion_matrix.png" alt="YOLO Validation Confusion Matrix" width="48%">
<img src="./confusion_matrix_normalized.png" alt="YOLO Normalized Validation Confusion Matrix" width="48%">

</div>

**Purpose:** These show how the YOLO validation predictions are distributed between `left_glove`, `right_glove`, and `unclear_glove`.

---

### 2. Manual Test Confusion Matrix

<div align="center">

<img src="./manual_test_confusion_matrix.png" alt="Manual Test Confusion Matrix" width="65%">

</div>

**Purpose:** This object-level test matrix is used to inspect real prediction behavior on the held-out test set, including dangerous class confusion.

---

### 3. Dataset Label Distribution

<div align="center">

<img src="./labels.jpg" alt="Dataset Label Distribution" width="65%">

</div>

**Purpose:** This shows the dataset distribution and helps identify class imbalance. In this experiment, `unclear_glove` is the dominant class.

---

### 4. Training Metric Curves

<div align="center">

<img src="./metrics_mAP50(B).png" alt="mAP50 Curve" width="48%">
<img src="./metrics_mAP50-95(B).png" alt="mAP50-95 Curve" width="48%">

<br>

<img src="./metrics_precision(B).png" alt="Precision Curve" width="48%">
<img src="./metrics_recall(B).png" alt="Recall Curve" width="48%">

</div>

**Purpose:** These curves show training/validation behavior across epochs and help judge whether the model improved, saturated, or overfit.

---

## 📁 Files in This Results Package

| File | Description |
|---|---|
| `README.md` | This results documentation file |
| `args.yaml` | Ultralytics training arguments |
| `data.yaml` | Dataset class names and split configuration |
| `best.pt` | Best PyTorch YOLO checkpoint selected by validation performance |
| `last.pt` | Final epoch PyTorch checkpoint |
| `best.onnx` | ONNX export of the best model for deployment/testing |
| `results.csv` | Epoch-by-epoch training metrics and losses |
| `labels.jpg` | Dataset label distribution visualisation |
| `confusion_matrix.png` | Ultralytics validation confusion matrix |
| `confusion_matrix_normalized.png` | Normalized validation confusion matrix |
| `manual_test_predictions.csv` | Object-level predictions on test images |
| `manual_test_confusion_matrix.csv` | CSV version of manual test confusion matrix |
| `manual_test_confusion_matrix.png` | Manual test confusion matrix image |
| `manual_test_classification_report.csv` | Precision, recall, F1-score, and support per class |
| `metrics_mAP50(B).png` | mAP50 curve |
| `metrics_mAP50-95(B).png` | mAP50-95 curve |
| `metrics_precision(B).png` | Precision curve |
| `metrics_recall(B).png` | Recall curve |

---

## ▶️ Testing the Model on a Mixed Video

The latest model is a **detect model**, not a pose model.

Do not test this `best.pt` using the old pose tester. If the window title says:

```text
GRIP YOLO-Pose Test
```

then the wrong tester script is being used.

Use the detector tester script instead:

```text
grip_video_detect_tester.py
```

### Recommended command

Place the tester script, `best.pt`, and the mixed test video in one folder:

```text
test_folder/
├── grip_video_detect_tester.py
├── best.pt
└── Mixed_Dataset_video.mp4
```

Run:

```bash
python grip_video_detect_tester.py --model best.pt --source Mixed_Dataset_video.mp4 --conf 0.05 --imgsz 960
```

If the model misses gloves, test with a lower confidence threshold and higher image size:

```bash
python grip_video_detect_tester.py --model best.pt --source Mixed_Dataset_video.mp4 --conf 0.01 --imgsz 1280
```

### Correct terminal output should show

```text
model.task = detect
model.names = {0: 'left_glove', 1: 'right_glove', 2: 'unclear_glove'}
```

If it prints `pose`, then the wrong `best.pt` file was loaded.

---

## 🧪 Interpretation Checklist

Use this checklist when explaining or evaluating the model:

- [x] The dataset came from factory conveyor frames.
- [x] The model was trained using YOLO11n transfer learning.
- [x] The model predicts three classes: left, right, and unclear.
- [x] `unclear_glove` is treated as a safe/manual outcome, not a failure.
- [x] Horizontal flipping was disabled because it can corrupt handedness.
- [x] `best.pt` was saved automatically based on validation performance.
- [x] `best.onnx` was exported for future deployment testing.
- [x] Confusion matrices and F1/precision/recall reports were generated.
- [x] The most important risk is true right gloves being predicted as left gloves.
- [ ] The current model still needs testing on independent mixed videos.
- [ ] More balanced real data is needed before production deployment.
- [ ] Better lighting and reduced motion blur are needed for improved reliability.
- [ ] Tracking/temporal voting should be added before SCARA integration.

---

## ✅ What Worked

| Area | Result |
|---|---|
| Annotation workflow | Custom annotation process worked for quick dataset creation |
| Dataset split | 70/20/10 split was created successfully |
| Label conversion | Pose-style labels were converted into detect labels for urgent training |
| YOLO11n training | Training completed successfully in Google Colab |
| Best checkpoint | `best.pt` generated |
| ONNX export | `best.onnx` generated |
| Evaluation files | Confusion matrices, curves, and CSV reports generated |
| GitHub documentation | Results package organized under `Vision/GRIP_YOLO_detect_results_package/` |

---

## ⚠️ Current Limitations

| Limitation | Why it matters | Next improvement |
|---|---|---|
| Small dataset | 608 images is prototype-scale | Collect more balanced real factory data |
| Class imbalance | `unclear_glove` dominates dataset | Add more clear left/right samples |
| Motion blur | Fingers/thumb are less visible | Use stronger lighting and shorter exposure |
| Pose not reliable yet | Glove keypoints are unstable when gloves are folded/crumpled | Build a cleaner keypoint dataset later |
| Mixed-video generalization not proven | Current split may not fully represent future factory conditions | Test on separate mixed videos |
| No tracker yet | Same glove may be detected repeatedly | Add ByteTrack or similar tracking |
| No robot command queue yet | Vision is not yet connected to SCARA pick logic | Add decision line + timed command queue |

---

## 🔁 Recommended Next Engineering Steps

### Immediate

1. Test `best.pt` using the detector tester, not the pose tester.
2. Record screenshots of false predictions.
3. Check the number of true right gloves predicted as left.
4. Add more clear `left_glove` and `right_glove` examples.
5. Retrain with a cleaner and more balanced dataset.

### Next model iteration

1. Add temporal voting across 5-10 frames per tracked glove.
2. Decide only when the glove crosses a stable decision line.
3. Use stricter confidence thresholds for automatic pass/reject.
4. Send low-confidence cases to `unclear/manual`.
5. Compare single-frame detection vs temporal voting.

### Future

1. Improve lighting and camera exposure.
2. Collect real right-glove wrong-lane samples.
3. Revisit pose/keypoints only after collecting clean keypoint annotations.
4. Add label defect detection as a separate ROI classifier.
5. Export and benchmark on Jetson Orin Nano using ONNX/TensorRT.
6. Integrate with SCARA robot command queue.

---

<div align="center">

**GRIP - Glove Rejection and Inspection Process**  
**Vision / YOLO Detect Results Package**  
**Team MOSFET · ENTC · University of Moratuwa · 2026**

</div>

# GRIP Left/Right/Unclear YOLO Detect Prototype Results

Classes:
- 0 = left_glove
- 1 = right_glove
- 2 = unclear_glove

Dataset split used:
- Train images: 825
- Val images: 121
- Test images: 62

Training:
- Model: yolo11n.pt
- Task: YOLO detect
- Image size: 640
- Epochs max: 100
- Early stopping patience: 20
- Horizontal flip disabled because it changes glove handedness.

Important result files:
- best.pt: best validation model
- last.pt: final epoch model
- results.csv: epoch metrics and losses
- manual_test_predictions.csv: object-level predictions
- manual_test_confusion_matrix.png: object-level test confusion matrix
- manual_test_classification_report.csv: precision, recall, F1

Industrial safety note:
The most important error is true right_glove predicted as left_glove, because that means a wrong-hand glove may pass.

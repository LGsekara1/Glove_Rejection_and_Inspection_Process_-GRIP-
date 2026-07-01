from ultralytics import YOLO
import cv2
import time
from collections import deque
from pathlib import Path

from label_roi_from_landmarks import crop_label_roi_from_21_keypoints
from label_roi_from_landmarks import preprocess_label_roi, determine_left_right
from scara_serial import ScaraSender, decision_to_bin

GLOVE_MODEL = "models/glove_detect.pt"
POSE_MODEL = "models/glove_pose.pt"
LABEL_MODEL = "models/label_cls.pt"

CAM_INDEX = 0
CONVEYOR_SPEED = 0.254
CAMERA_TO_SCARA_DISTANCE = 0.80
DELAY_OFFSET = 0.0
PICK_DELAY = CAMERA_TO_SCARA_DISTANCE / CONVEYOR_SPEED + DELAY_OFFSET

DETECT_CONF = 0.45
POSE_CONF = 0.35
LABEL_CONF = 0.70
EXPECTED_BATCH_SIZE = None
USE_SERIAL = False

sent_track_ids = set()
queue = deque()

glove_model = YOLO(GLOVE_MODEL)
pose_model = YOLO(POSE_MODEL)
label_model = YOLO(LABEL_MODEL)
scara = ScaraSender(enabled=USE_SERIAL)

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

def extract_keypoints_from_pose_result(result):
    if result.keypoints is None or len(result.keypoints) == 0:
        return None
    return result.keypoints.data[0].cpu().numpy()

def classify_size_from_landmarks(kpts):
    return "unknown"

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        now = time.time()
        display = frame.copy()

        results = glove_model.track(frame, persist=True, tracker="bytetrack.yaml",
                                    imgsz=640, conf=DETECT_CONF, verbose=False)[0]

        if results.boxes is not None:
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                track_id = int(box.id[0]) if box.id is not None else -1

                glove_crop = frame[y1:y2, x1:x2]
                if glove_crop.size == 0:
                    continue

                decision = "PASS_GOOD_LEFT_GLOVE"
                pose_result = pose_model(glove_crop, imgsz=640, conf=POSE_CONF, verbose=False)[0]
                kpts = extract_keypoints_from_pose_result(pose_result)

                if kpts is None:
                    decision = "MANUAL_LANDMARK_FAIL"
                else:
                    hand = determine_left_right(kpts)
                    size = classify_size_from_landmarks(kpts)

                    # TODO: map setup_A_side/setup_B_side to actual left/right after visual testing.
                    if hand == "right":
                        decision = "PICK_RIGHT_GLOVE"
                    elif EXPECTED_BATCH_SIZE is not None and size != EXPECTED_BATCH_SIZE:
                        decision = "PICK_WRONG_SIZE"
                    else:
                        label_roi, roi_box = crop_label_roi_from_21_keypoints(glove_crop, kpts)
                        if label_roi is None:
                            decision = "MANUAL_LABEL_ROI_FAIL"
                        else:
                            cls_result = label_model(label_roi, imgsz=224, verbose=False)[0]
                            probs = cls_result.probs
                            cls_id = int(probs.top1)
                            cls_conf = float(probs.top1conf)
                            cls_name = label_model.names[cls_id]

                            if cls_conf < LABEL_CONF:
                                decision = "MANUAL_LABEL_LOW_CONF"
                            elif cls_name == "defect":
                                decision = "PICK_LABEL_DEFECT"
                            else:
                                decision = "PASS_GOOD_LEFT_GLOVE"

                should_pick = decision.startswith("PICK") or decision.startswith("MANUAL")
                if should_pick and track_id not in sent_track_ids:
                    queue.append({
                        "target_time": now + PICK_DELAY,
                        "track_id": track_id,
                        "decision": decision,
                    })
                    sent_track_ids.add(track_id)

                color = (0, 255, 0) if decision.startswith("PASS") else (0, 0, 255)
                cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
                cv2.putText(display, f"ID {track_id} {decision}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        while queue and now >= queue[0]["target_time"]:
            item = queue.popleft()
            bin_name = decision_to_bin(item["decision"])
            scara.send_pick(item["decision"], item["track_id"], bin_name)

        preview = cv2.resize(display, (960, 540))
        cv2.imshow("GRIP real-time", preview)
        if cv2.waitKey(1) & 0xFF == 27:
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    scara.close()

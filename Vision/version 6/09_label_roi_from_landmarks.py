import cv2
import numpy as np

WRIST = 0
INDEX_MCP = 5
MIDDLE_MCP = 9
RING_MCP = 13
LITTLE_MCP = 17
THUMB_TIP = 4
MIDDLE_TIP = 12

LABEL_TL = 21
LABEL_TR = 22
LABEL_BR = 23
LABEL_BL = 24

def point_ok(kpts, idx, conf_th=0.35):
    return idx < len(kpts) and kpts[idx][2] >= conf_th

def crop_label_roi_from_25_keypoints(glove_crop, kpts):
    needed = [LABEL_TL, LABEL_TR, LABEL_BR, LABEL_BL]
    if not all(point_ok(kpts, i) for i in needed):
        return None, None

    pts = np.array([[kpts[i][0], kpts[i][1]] for i in needed], dtype=np.float32)
    x, y, w, h = cv2.boundingRect(pts.astype(np.int32))
    pad = int(0.08 * max(w, h))
    H, W = glove_crop.shape[:2]
    x1, y1 = max(0, x - pad), max(0, y - pad)
    x2, y2 = min(W, x + w + pad), min(H, y + h + pad)
    return glove_crop[y1:y2, x1:x2], (x1, y1, x2, y2)

def crop_label_roi_from_21_keypoints(glove_crop, kpts):
    needed = [WRIST, INDEX_MCP, MIDDLE_MCP, RING_MCP, LITTLE_MCP]
    if not all(point_ok(kpts, i) for i in needed):
        return None, None

    wrist = np.array(kpts[WRIST][:2], dtype=np.float32)
    mcp_pts = np.array([kpts[i][:2] for i in [INDEX_MCP, MIDDLE_MCP, RING_MCP, LITTLE_MCP]], dtype=np.float32)
    palm_center = mcp_pts.mean(axis=0)

    direction = wrist - palm_center
    norm = np.linalg.norm(direction)
    if norm < 1e-6:
        return None, None
    direction = direction / norm

    palm_width = np.linalg.norm(np.array(kpts[INDEX_MCP][:2]) - np.array(kpts[LITTLE_MCP][:2]))

    LABEL_OFFSET = 0.10
    ROI_W_FACTOR = 1.15
    ROI_H_FACTOR = 0.75

    label_center = wrist + direction * (LABEL_OFFSET * palm_width)
    roi_w = int(ROI_W_FACTOR * palm_width)
    roi_h = int(ROI_H_FACTOR * palm_width)

    x1 = int(label_center[0] - roi_w / 2)
    y1 = int(label_center[1] - roi_h / 2)
    x2 = int(label_center[0] + roi_w / 2)
    y2 = int(label_center[1] + roi_h / 2)

    H, W = glove_crop.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(W, x2), min(H, y2)

    if x2 <= x1 or y2 <= y1:
        return None, None

    return glove_crop[y1:y2, x1:x2], (x1, y1, x2, y2)

def determine_left_right(kpts):
    needed = [THUMB_TIP, INDEX_MCP, MIDDLE_MCP, RING_MCP, LITTLE_MCP]
    if not all(point_ok(kpts, i) for i in needed):
        return "unknown"

    thumb_x = kpts[THUMB_TIP][0]
    palm_x = np.mean([kpts[i][0] for i in [INDEX_MCP, MIDDLE_MCP, RING_MCP, LITTLE_MCP]])

    # Must be verified for your camera orientation and glove side.
    if thumb_x < palm_x:
        return "setup_A_side"
    else:
        return "setup_B_side"

def preprocess_label_roi(label_roi):
    gray = cv2.cvtColor(label_roi, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    sharp = cv2.filter2D(enhanced, -1, kernel)
    denoised = cv2.medianBlur(sharp, 3)
    _, binary = cv2.threshold(denoised, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary

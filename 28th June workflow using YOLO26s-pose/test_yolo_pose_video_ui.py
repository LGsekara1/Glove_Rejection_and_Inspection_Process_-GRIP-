from pathlib import Path
import time
import cv2
import torch
import tkinter as tk
from tkinter import filedialog, messagebox
from ultralytics import YOLO


# ============================================================
# DEFAULT PATHS
# ============================================================

DEFAULT_MODEL_PATH = r"D:\GRIP_Yolo_pose\YOLO_pose_MVP_dataset\best.pt"
DEFAULT_VIDEO_PATH = r"D:\GRIP_Yolo_pose\test_video.mp4"
DEFAULT_OUTPUT_PATH = r"D:\GRIP_Yolo_pose\pose_detected_output.mp4"


KEYPOINT_NAMES = [
    "wrist_center",   # 0
    "palm_center",    # 1
    "thumb_tip",      # 2
    "index_tip",      # 3
    "middle_tip",     # 4
    "ring_tip",       # 5
    "pinky_tip",      # 6
]

SKELETON_EDGES = [
    (0, 1),
    (1, 2),
    (1, 3),
    (1, 4),
    (1, 5),
    (1, 6),
]

KEYPOINT_COLORS = [
    (255, 255, 255),  # wrist
    (0, 255, 255),    # palm
    (255, 0, 255),    # thumb
    (0, 255, 0),      # index
    (255, 255, 0),    # middle
    (255, 128, 0),    # ring
    (0, 128, 255),    # pinky
]


# ============================================================
# CONFIG UI
# ============================================================

class AppConfig:
    def __init__(self):
        self.model_path = DEFAULT_MODEL_PATH
        self.video_path = DEFAULT_VIDEO_PATH
        self.confidence = 0.15
        self.imgsz = 768
        self.keypoint_confidence = 0.25
        self.save_output = False
        self.output_path = DEFAULT_OUTPUT_PATH


def choose_settings():
    config = AppConfig()

    root = tk.Tk()
    root.title("GRIP YOLO-Pose Video Test")
    root.geometry("820x410")
    root.resizable(False, False)

    model_var = tk.StringVar(value=config.model_path)
    video_var = tk.StringVar(value=config.video_path)
    conf_var = tk.StringVar(value=str(config.confidence))
    imgsz_var = tk.StringVar(value=str(config.imgsz))
    kpt_conf_var = tk.StringVar(value=str(config.keypoint_confidence))
    save_var = tk.BooleanVar(value=config.save_output)
    output_var = tk.StringVar(value=config.output_path)

    def browse_model():
        path = filedialog.askopenfilename(
            title="Select YOLO-pose best.pt",
            filetypes=[("PyTorch model", "*.pt"), ("All files", "*.*")]
        )
        if path:
            model_var.set(path)

    def browse_video():
        path = filedialog.askopenfilename(
            title="Select video",
            filetypes=[
                ("Video files", "*.mp4 *.avi *.mov *.mkv *.wmv"),
                ("All files", "*.*")
            ]
        )
        if path:
            video_var.set(path)

    def browse_output():
        path = filedialog.asksaveasfilename(
            title="Save output video as",
            defaultextension=".mp4",
            filetypes=[("MP4 video", "*.mp4"), ("All files", "*.*")]
        )
        if path:
            output_var.set(path)

    def start():
        model_path = model_var.get().strip()
        video_path = video_var.get().strip()
        output_path = output_var.get().strip()

        if not Path(model_path).exists():
            messagebox.showerror("Error", f"Model file not found:\n{model_path}")
            return

        if not Path(video_path).exists():
            messagebox.showerror("Error", f"Video file not found:\n{video_path}")
            return

        try:
            conf = float(conf_var.get())
            if conf <= 0 or conf >= 1:
                raise ValueError
        except ValueError:
            messagebox.showerror("Error", "Detection confidence must be between 0 and 1.")
            return

        try:
            kpt_conf = float(kpt_conf_var.get())
            if kpt_conf < 0 or kpt_conf >= 1:
                raise ValueError
        except ValueError:
            messagebox.showerror("Error", "Keypoint confidence must be between 0 and 1.")
            return

        try:
            imgsz = int(imgsz_var.get())
        except ValueError:
            messagebox.showerror("Error", "Image size must be an integer.")
            return

        config.model_path = model_path
        config.video_path = video_path
        config.confidence = conf
        config.keypoint_confidence = kpt_conf
        config.imgsz = imgsz
        config.save_output = save_var.get()
        config.output_path = output_path

        root.destroy()

    tk.Label(
        root,
        text="GRIP YOLO-Pose Video Test",
        font=("Arial", 16, "bold")
    ).pack(pady=12)

    frame = tk.Frame(root)
    frame.pack(padx=15, pady=5, fill="x")

    tk.Label(frame, text="YOLO-pose .pt:", width=20, anchor="w").grid(row=0, column=0, pady=8)
    tk.Entry(frame, textvariable=model_var, width=78).grid(row=0, column=1, pady=8)
    tk.Button(frame, text="Browse", command=browse_model).grid(row=0, column=2, padx=5)

    tk.Label(frame, text="Video:", width=20, anchor="w").grid(row=1, column=0, pady=8)
    tk.Entry(frame, textvariable=video_var, width=78).grid(row=1, column=1, pady=8)
    tk.Button(frame, text="Browse", command=browse_video).grid(row=1, column=2, padx=5)

    tk.Label(frame, text="Detection confidence:", width=20, anchor="w").grid(row=2, column=0, pady=8)
    tk.Entry(frame, textvariable=conf_var, width=12).grid(row=2, column=1, sticky="w", pady=8)

    tk.Label(frame, text="Image size:", width=20, anchor="w").grid(row=3, column=0, pady=8)
    tk.Entry(frame, textvariable=imgsz_var, width=12).grid(row=3, column=1, sticky="w", pady=8)

    tk.Label(frame, text="Keypoint confidence:", width=20, anchor="w").grid(row=4, column=0, pady=8)
    tk.Entry(frame, textvariable=kpt_conf_var, width=12).grid(row=4, column=1, sticky="w", pady=8)

    tk.Checkbutton(
        frame,
        text="Save output video",
        variable=save_var
    ).grid(row=5, column=1, sticky="w", pady=8)

    tk.Label(frame, text="Output path:", width=20, anchor="w").grid(row=6, column=0, pady=8)
    tk.Entry(frame, textvariable=output_var, width=78).grid(row=6, column=1, pady=8)
    tk.Button(frame, text="Browse", command=browse_output).grid(row=6, column=2, padx=5)

    tk.Button(
        root,
        text="Start Pose Test",
        command=start,
        font=("Arial", 12, "bold"),
        bg="#0B5ED7",
        fg="white",
        width=20,
        height=2
    ).pack(pady=14)

    root.mainloop()
    return config


# ============================================================
# DRAWING HELPERS
# ============================================================

def draw_text_bg(img, text, x, y, color, scale=0.6, thickness=2):
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), base = cv2.getTextSize(text, font, scale, thickness)

    x = max(0, int(x))
    y = max(th + 8, int(y))

    cv2.rectangle(
        img,
        (x, y - th - 8),
        (x + tw + 8, y + base + 4),
        (0, 0, 0),
        -1
    )

    cv2.putText(
        img,
        text,
        (x + 4, y - 4),
        font,
        scale,
        color,
        thickness,
        cv2.LINE_AA
    )


def pose_quality(kpts_xy, kpts_conf, threshold):
    """
    For now this is not KEEP/REJECT.
    It only checks whether important keypoints are confident enough.
    """

    important = {
        "wrist": 0,
        "palm": 1,
        "index": 3,
        "middle": 4,
        "ring": 5,
        "pinky": 6,
    }

    good = []
    bad = []

    for name, idx in important.items():
        conf = float(kpts_conf[idx]) if kpts_conf is not None else 1.0
        x, y = kpts_xy[idx]

        if conf >= threshold and x > 0 and y > 0:
            good.append(name)
        else:
            bad.append(name)

    if len(bad) == 0:
        return "POSE_OK", (0, 255, 0)

    if len(good) >= 4:
        return "POSE_PARTIAL", (0, 255, 255)

    return "MANUAL_WEAK_POSE", (0, 165, 255)


def draw_pose_result(frame, result, kpt_threshold):
    boxes = result.boxes
    keypoints = result.keypoints

    if boxes is None or keypoints is None:
        return frame

    box_xyxy = boxes.xyxy.cpu().numpy()
    box_conf = boxes.conf.cpu().numpy()

    kpts_xy = keypoints.xy.cpu().numpy()

    kpts_conf = None
    if keypoints.conf is not None:
        kpts_conf = keypoints.conf.cpu().numpy()

    for i in range(len(box_xyxy)):
        x1, y1, x2, y2 = box_xyxy[i].astype(int)
        det_conf = float(box_conf[i])

        this_xy = kpts_xy[i]

        if kpts_conf is not None:
            this_conf = kpts_conf[i]
        else:
            this_conf = [1.0] * len(KEYPOINT_NAMES)

        status, color = pose_quality(this_xy, this_conf, kpt_threshold)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        draw_text_bg(
            frame,
            f"glove {det_conf:.2f} | {status}",
            x1,
            y1 - 10,
            color,
            scale=0.62,
            thickness=2
        )

        # Draw skeleton edges
        for a, b in SKELETON_EDGES:
            xa, ya = this_xy[a]
            xb, yb = this_xy[b]

            ca = float(this_conf[a])
            cb = float(this_conf[b])

            if ca >= kpt_threshold and cb >= kpt_threshold:
                cv2.line(
                    frame,
                    (int(xa), int(ya)),
                    (int(xb), int(yb)),
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA
                )

        # Draw keypoints
        for kp_idx, (x, y) in enumerate(this_xy):
            conf = float(this_conf[kp_idx])

            if conf < kpt_threshold:
                continue

            color_kp = KEYPOINT_COLORS[kp_idx]
            cv2.circle(frame, (int(x), int(y)), 5, color_kp, -1)
            cv2.circle(frame, (int(x), int(y)), 6, (0, 0, 0), 1)

            cv2.putText(
                frame,
                str(kp_idx),
                (int(x) + 7, int(y) - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color_kp,
                2,
                cv2.LINE_AA
            )

    return frame


# ============================================================
# MAIN VIDEO LOOP
# ============================================================

def run_pose_video(config):
    model_path = Path(config.model_path)
    video_path = Path(config.video_path)

    print("Loading YOLO-pose model:", model_path)
    model = YOLO(str(model_path))

    print("Model task:", getattr(model, "task", "unknown"))
    print("Model names:", model.names)

    device = 0 if torch.cuda.is_available() else "cpu"
    print("Device:", "cuda:0" if device == 0 else "cpu")

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    video_fps = cap.get(cv2.CAP_PROP_FPS)
    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print("Video:", video_path)
    print("Resolution:", frame_w, "x", frame_h)
    print("FPS:", video_fps)
    print("Total frames:", total_frames)

    writer = None

    if config.save_output:
        out_path = Path(config.output_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(
            str(out_path),
            fourcc,
            video_fps if video_fps > 0 else 25,
            (frame_w, frame_h)
        )

        print("Saving output to:", out_path)

    window_name = "GRIP YOLO-Pose Test | Q quit | SPACE pause | S screenshot"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)

    paused = False
    frame_count = 0
    last_frame = None
    prev_time = time.time()
    realtime_fps = 0.0

    while True:
        if not paused:
            ret, frame = cap.read()

            if not ret:
                print("End of video.")
                break

            frame_count += 1

            results = model.predict(
                source=frame,
                imgsz=config.imgsz,
                conf=config.confidence,
                iou=0.45,
                max_det=50,
                device=device,
                verbose=False
            )

            for result in results:
                frame = draw_pose_result(frame, result, config.keypoint_confidence)

            now = time.time()
            dt = now - prev_time
            prev_time = now

            if dt > 0:
                realtime_fps = 1.0 / dt

            cv2.putText(
                frame,
                f"Frame {frame_count}/{total_frames} | FPS {realtime_fps:.1f} | det {config.confidence} | kpt {config.keypoint_confidence}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

            cv2.putText(
                frame,
                "This test checks pose quality only. KEEP/REJECT logic comes later.",
                (20, 78),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

            cv2.putText(
                frame,
                "0 wrist | 1 palm | 2 thumb | 3 index | 4 middle | 5 ring | 6 pinky",
                (20, 112),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

            if writer is not None:
                writer.write(frame)

            last_frame = frame.copy()
            cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q") or key == 27:
            print("Stopped by user.")
            break

        elif key == ord(" "):
            paused = not paused
            print("Paused" if paused else "Playing")

        elif key == ord("s"):
            if last_frame is not None:
                screenshot_path = video_path.parent / f"pose_screenshot_{frame_count:06d}.jpg"
                cv2.imwrite(str(screenshot_path), last_frame)
                print("Screenshot saved:", screenshot_path)

    cap.release()

    if writer is not None:
        writer.release()

    cv2.destroyAllWindows()

    print("Finished.")


if __name__ == "__main__":
    settings = choose_settings()
    run_pose_video(settings)
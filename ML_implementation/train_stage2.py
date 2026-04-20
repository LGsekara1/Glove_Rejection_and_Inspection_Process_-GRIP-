from ultralytics import YOLO
import torch

if __name__ == '__main__':
    print(f"GPU: {torch.cuda.get_device_name(0)}")

    # Fine-tune from your best glove detection model
    model = YOLO(r"D:/Abdul Rahman/Engineering_UOM/Personal Projects/GRIP/VsCode implementation/Find_gloves.yolov8/runs\detect/runs/glove_v2/weights/best.pt")

    results = model.train(
        data=r"D:/Abdul Rahman/Engineering_UOM/Personal Projects/GRIP/VsCode implementation/Find_gloves.yolov8/Training_v2/data.yaml",
        epochs=50,
        imgsz=640,
        batch=8,
        device=0,
        workers=2,
        project="runs",
        name="glove_v3_leftright",
        exist_ok=True,
        patience=15,
        save=True,
        plots=True,
        val=True,
        lr0=0.001,
        lrf=0.01,
    )

    print(f"\n✅ Stage 2 complete.")
    print(f"📁 Results: {results.save_dir}")
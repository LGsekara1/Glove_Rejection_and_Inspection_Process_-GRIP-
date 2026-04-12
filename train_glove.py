from ultralytics import YOLO
import torch

if __name__ == '__main__':
    print(f"PyTorch : {torch.__version__}")
    print(f"CUDA    : {torch.cuda.is_available()}")
    print(f"GPU     : {torch.cuda.get_device_name(0)}")

    model = YOLO("yolov8n.pt")

    results = model.train(
        data=r"D:\Abdul Rahman\Engineering_UOM\Personal Projects\GRIP\VsCode implementation\Find_gloves.yolov8\Training\data.yaml",
        epochs=50,
        imgsz=640,
        batch=8,
        device=0,
        workers=2,
        project="runs",
        name="glove_v1",
        exist_ok=True,
        patience=20,
        save=True,
        plots=True,
        val=True,
    )

    print("\n✅ Training complete.")
    print(f"📁 Results saved to: {results.save_dir}")
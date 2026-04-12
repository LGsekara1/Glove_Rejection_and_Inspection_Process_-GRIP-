from ultralytics import YOLO

if __name__ == '__main__':
    model = YOLO(r"runs\detect\runs\glove_v1\weights\best.pt")

    # Test on your validation images
    results = model.predict(
        source=r"D:\Abdul Rahman\Engineering_UOM\Personal Projects\GRIP\VsCode implementation\Find_gloves.yolov8\Training\valid\images",
        conf=0.5,       # confidence threshold
        save=True,      # saves output images with boxes drawn
        project="test_output",
        name="glove_test",
        exist_ok=True,
    )

    print("✅ Test complete — check test_output/glove_test/ folder")
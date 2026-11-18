# SAUVC 2027 YOLOv8 Detection Model

## Files in this package:
- `best.pt`: Trained YOLOv8 model weights (6.2 MB)
- `test_model.py`: Ready-to-run Python test script

## Classes detected:
0: blue_tub
1: flare
2: gate
3: golf
4: pole
5: red_tub
6: tubs

---

## How to Test:

### 1. Install ultralytics
```bash
pip install ultralytics
```

### 2. Run prediction on an image or video:
```bash
python3 test_model.py path/to/underwater_image.jpg
```
or run directly with YOLO CLI:
```bash
yolo predict model=best.pt source=path/to/image.jpg show=True
```

### 3. Run real-time on live camera / webcam:
```bash
python3 test_model.py 0
```
or:
```bash
yolo predict model=best.pt source=0 show=True
```

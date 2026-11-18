"""
Quick test script for SAUVC 2027 YOLOv8 model
Usage:
    python3 test_model.py [path_to_image_or_video_or_0_for_webcam]
"""

import sys
from ultralytics import YOLO

# Load model weights
model_path = "best.pt"
model = YOLO(model_path)

print("[+] Model loaded successfully!")
print(f"[+] Classes: {model.names}")

# Test source: default to webcam (0), or image/video passed as argument
source = sys.argv[1] if len(sys.argv) > 1 else 0

print(f"[+] Running inference on source: {source}")
results = model.predict(source=source, conf=0.5, show=True, save=True)

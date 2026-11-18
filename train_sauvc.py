#!/usr/bin/env python3
"""
SAUVC 2027 YOLOv8 Training Script
Optimized for multi-threaded CPU execution.
"""

import sys
from ultralytics import YOLO

def main():
    print("[+] Loading YOLOv8 nano model (yolov8n.pt)...")
    model = YOLO("yolov8n.pt")

    print("[+] Starting training on SAUVC dataset...")
    results = model.train(
        data="/home/manav/Desktop/SAUVC/dataset/data.yaml",
        epochs=30,
        imgsz=640,
        batch=16,
        device="cpu",
        workers=4,
        project="/home/manav/Desktop/SAUVC/runs/train",
        name="sauvc2027",
        exist_ok=True,
        save=True,
        verbose=True
    )

    print("\n[✓] Training completed successfully!")
    print(f"[+] Best weights saved at: {results.save_dir}/weights/best.pt")

if __name__ == "__main__":
    main()

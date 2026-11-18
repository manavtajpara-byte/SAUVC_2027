import os
import sys
from roboflow import Roboflow

# Workspace & Project details extracted from your link:
# https://app.roboflow.com/tajpara-manav/sauvc-o23xq-w1fwr
WORKSPACE = "tajpara-manav"
PROJECT = "sauvc-o23xq-w1fwr"

# Get API key from argument, environment variable, or prompt
api_key = os.environ.get("ROBOFLOW_API_KEY")
if len(sys.argv) > 1:
    api_key = sys.argv[1]

if not api_key:
    print("\n[!] Roboflow API key needed.")
    print("Usage: python3 download_sauvc_dataset.py <YOUR_API_KEY>")
    print("Or run: export ROBOFLOW_API_KEY='your_key' && python3 download_sauvc_dataset.py\n")
    sys.exit(1)

# Target directory
download_dir = os.path.expanduser("~/Desktop/SAUVC/datasets")
os.makedirs(download_dir, exist_ok=True)
os.chdir(download_dir)

print(f"Connecting to Roboflow workspace '{WORKSPACE}', project '{PROJECT}'...")
rf = Roboflow(api_key=api_key)
project = rf.workspace(WORKSPACE).project(PROJECT)

# Get the latest generated version (default to version 1)
version_num = 1
if len(sys.argv) > 2:
    version_num = int(sys.argv[2])

print(f"Downloading version {version_num} in YOLOv8 format...")
version = project.version(version_num)
dataset = version.download("yolov8")

print(f"\n[✓] Successfully downloaded dataset to: {dataset.location}")

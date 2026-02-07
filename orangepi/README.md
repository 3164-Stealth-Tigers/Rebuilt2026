# Orange Pi 5 Vision System

ML-based vision system for FRC robot detection and FUEL game piece tracking. Runs on the Orange Pi 5 using the Rockchip NPU for hardware-accelerated inference.

## Overview

This system detects:
- **Robots**: Opposing robots on the field for avoidance and defense
- **FUEL**: Yellow ball game pieces for autonomous collection

Detection results are published to NetworkTables for the roboRIO to consume.

## Hardware Requirements

- Orange Pi 5 (with RK3588 NPU)
- USB camera (tested with Logitech C920)
- Ethernet connection to robot network

## Quick Start

### 1. Installation

Copy this folder to the Orange Pi and run:

```bash
sudo ./install.sh
```

### 2. Add Model Files

Copy your RKNN model files to `/opt/orangepi-vision/models/`:
- `robot_detector.rknn`
- `fuel_detector.rknn`

### 3. Configure

Edit `/opt/orangepi-vision/config.py`:
- Set `NT_SERVER_IP` to your roboRIO's IP (default: `10.31.64.2`)
- Adjust camera settings if needed

### 4. Start Service

```bash
sudo systemctl start orangepi-vision
```

## Model Conversion

### Converting YOLOv8 to RKNN

1. **Train your YOLOv8 model** using Ultralytics:
   ```python
   from ultralytics import YOLO
   model = YOLO('yolov8n.pt')
   model.train(data='your_dataset.yaml', epochs=100)
   ```

2. **Export to ONNX**:
   ```python
   model.export(format='onnx', opset=12, simplify=True)
   ```

3. **Convert ONNX to RKNN** (on x86 machine with rknn-toolkit2):
   ```python
   from rknn.api import RKNN

   rknn = RKNN()

   # Configure for RK3588
   rknn.config(
       target_platform='rk3588',
       mean_values=[[0, 0, 0]],
       std_values=[[255, 255, 255]],
   )

   # Load ONNX model
   rknn.load_onnx(model='best.onnx')

   # Build RKNN model
   rknn.build(do_quantization=True, dataset='calibration_images.txt')

   # Export
   rknn.export_rknn('robot_detector.rknn')
   ```

4. **Copy to Orange Pi**:
   ```bash
   scp robot_detector.rknn orangepi:/opt/orangepi-vision/models/
   ```

## NetworkTables Interface

### Robot Detection (`Vision/Robots/`)

| Key | Type | Description |
|-----|------|-------------|
| `robotCount` | int | Number of robots detected |
| `closestRobotX` | double | X position (meters, +forward) |
| `closestRobotY` | double | Y position (meters, +left) |
| `closestRobotDistance` | double | Distance to closest robot |
| `closestRobotVelX` | double | X velocity (m/s) |
| `closestRobotVelY` | double | Y velocity (m/s) |
| `closestConfidence` | double | Detection confidence (0-1) |
| `timestamp` | double | Frame timestamp |

### FUEL Detection (`Vision/Fuel/`)

| Key | Type | Description |
|-----|------|-------------|
| `fuelCount` | int | Number of FUEL detected |
| `hasFuel` | boolean | True if any FUEL visible |
| `bestFuelX` | double | X position (meters, +forward) |
| `bestFuelY` | double | Y position (meters, +left) |
| `bestFuelDistance` | double | Distance to best FUEL |
| `bestFuelAngle` | double | Angle to FUEL (degrees, +CCW) |
| `bestFuelConfidence` | double | Detection confidence (0-1) |
| `bestFuelInIntakeRange` | boolean | True if within intake range |
| `intakeReady` | boolean | True if ready for intake |
| `timestamp` | double | Frame timestamp |

## Coordinate System

Robot-relative coordinates:
- **+X**: Front of robot
- **+Y**: Left of robot
- **Angles**: 0 = directly ahead, positive = counterclockwise

## Service Management

```bash
# Start service
sudo systemctl start orangepi-vision

# Stop service
sudo systemctl stop orangepi-vision

# View status
sudo systemctl status orangepi-vision

# View logs (live)
sudo journalctl -u orangepi-vision -f

# Disable auto-start
sudo systemctl disable orangepi-vision
```

## Manual Testing

```bash
cd /opt/orangepi-vision
source venv/bin/activate

# Run in simulation mode (no camera/models)
python vision_service.py --simulate

# Run with actual hardware
python vision_service.py
```

## Troubleshooting

### Camera not detected
```bash
# List video devices
v4l2-ctl --list-devices

# Test camera
ffplay /dev/video0
```

### NetworkTables not connecting
- Verify roboRIO IP address in config.py
- Check network connectivity: `ping 10.31.64.2`
- Ensure roboRIO is running NetworkTables server

### RKNN errors
- Ensure RKNN Toolkit2 Lite is installed correctly
- Verify model file exists and is valid RKNN format
- Check NPU is accessible: `ls /dev/dri/`

### Low FPS
- Reduce camera resolution in config.py
- Check CPU/NPU utilization: `htop`
- Ensure NPU is being used (not CPU fallback)

## Camera Calibration

For accurate distance estimation, calibrate your camera:

1. Measure known object sizes at various distances
2. Update `config.py` with:
   - `CAMERA_FOV_HORIZONTAL`: Horizontal field of view
   - `CAMERA_FOV_VERTICAL`: Vertical field of view
   - `CAMERA_MOUNT_HEIGHT`: Camera height above ground

## Directory Structure

```
orangepi/
+-- README.md                    # This file
+-- requirements.txt             # Python dependencies
+-- install.sh                   # Installation script
+-- vision_service.py            # Main entry point
+-- config.py                    # Configuration
+-- detectors/
|   +-- base_detector.py         # Abstract detector class
|   +-- robot_detector.py        # Robot detection
|   +-- fuel_detector.py         # FUEL detection
+-- network/
|   +-- nt_publisher.py          # NetworkTables publisher
+-- utils/
|   +-- camera.py                # Camera utilities
|   +-- geometry.py              # Coordinate transforms
+-- models/                      # RKNN model files
+-- systemd/
    +-- orangepi-vision.service  # Systemd service
```

## License

Part of FRC Team 3164 codebase.

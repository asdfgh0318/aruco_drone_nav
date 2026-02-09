# Testing Guide

Complete guide for testing the ArUco Vision GPS system.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [RPi Testing](#rpi-testing)
3. [SITL Testing](#sitl-testing)
4. [Hardware Testing](#hardware-testing)
5. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Print Markers

Print ArUco markers from the `markers/` folder:

| File | What it is | Print settings |
|------|------------|----------------|
| ArUco markers (DICT_4X4_50) | Ceiling navigation markers | **100% scale**, no fit-to-page |
| `chessboard_9x6.pdf` | Calibration pattern | **100% scale**, no fit-to-page |

**Verify printed size:**
- ArUco markers should be **18cm x 18cm** (A4 printable)
- Chessboard squares should be **~25mm x 25mm**

### Generate Markers

```bash
# Generate 18cm markers for A4 paper
python3 tools/generate_markers.py --ids 0,1,2,3 --size 180 -o markers/
```

### Hardware Checklist

- [ ] Raspberry Pi Zero 2W
- [ ] USB camera (MJPG capable, 720p)
- [ ] Printed ArUco markers (at least 1)
- [ ] Good lighting (avoid direct sunlight/shadows on markers)

---

## RPi Testing

### Step 1: Sync Code to RPi

```bash
# From your development machine
./sync_to_rpi.sh
```

### Step 2: SSH and Run

```bash
ssh aruconav@aruconav.local
cd /home/aruconav/aruco_drone_nav

# Test mode - prints position to console
python3 -m src --mode test

# Stream mode - HTTP server for remote monitoring
python3 -m src --mode stream --port 8001
```

### Step 3: Verify Detection

```bash
# From your local machine (stream mode must be running)
curl http://aruconav.local:8001/position
```

**Expected output:**
```json
{
  "x": 0.549, "y": -0.289, "z": 1.712,
  "yaw": 159.2, "marker_ids": ["0"], "confidence": 1.0,
  "detection_rate": 1.0, "uptime": 10.4,
  "timing": {"gray": "5", "clahe": "21", "detect": "107", "total": "134"}
}
```

### Step 4: Debug Viewer

```bash
# From your local machine
python3 tools/debug_viewer.py --host aruconav.local --port 8001
```

### What to Verify

- [ ] Camera initializes (MJPG 1280x720)
- [ ] Markers detected (99-100% detection rate)
- [ ] Position values are reasonable
- [ ] Timing: ~140ms/frame, ~6 FPS
- [ ] HTTP endpoints respond (/position, /debug-frame)

### Current Performance (RPi Zero 2W)

| Metric | Value |
|--------|-------|
| Resolution | 1280x720 (MJPG) |
| Detection Rate | 99-100% |
| Processing Time | ~140ms/frame |
| FPS | ~6 |
| Timing | gray:3ms CLAHE:20ms bgr:2ms detect:110ms |

---

## SITL Testing

Validate the full vision-to-FC MAVLink pipeline without hardware.

### Start SITL

```bash
cd /tmp && ~/ardupilot/build/sitl/bin/arducopter \
    --model + --speedup 1 -I0 \
    --home 52.2297,21.0122,100,0 \
    --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm,config/sitl_params.parm
```

### Run Validation

```bash
# Basic: EKF convergence
python3 tools/test_sitl.py -v --skip-params

# Arm + guided flight
python3 tools/test_sitl.py -v --skip-params --arm

# Circle pattern
python3 tools/test_sitl.py -v --skip-params -p circle -d 20
```

See [SITL Testing Guide](sitl.html) for full details.

---

## Hardware Testing

### Pre-Flight Checklist

1. [ ] Wire RPi to FC (UART serial, TX->RX, RX->TX, 921600 baud)
2. [ ] Mount camera facing up on drone frame
3. [ ] Set FC parameters (see [FC_CONFIG.md](FC_CONFIG.md))
4. [ ] Mount marker(s) on ceiling above test area
5. [ ] Measure marker position(s) and update `config/marker_map.yaml`

### Test Sequence

#### 1. Bench Test
RPi + FC powered on bench, no propellers.

```bash
# On RPi
python3 -m src --mode run
```

**Verify:** VISION_POSITION_ESTIMATE received in MAVLink Inspector.

#### 2. EKF Convergence
Wait for FC status message: "EKF3 IMU0 origin set"

**Verify:** Position error converges to <0.5m.

#### 3. Ground Test
Drone on ground under ceiling marker.

**Verify:** Stable position reading, no jumps.

#### 4. Tethered Hover
Safety tether attached. Take off in Loiter mode.

**Verify:** Position hold is stable, no "toilet bowl" oscillation.

#### 5. Free Hover
Remove tether. Hover under single marker.

**Verify:** Stable hover at ~6 Hz update rate.

#### 6. Movement Test
Move between markers (if multiple deployed).

**Verify:** Smooth position transitions.

### What to Watch For

- EKF position error should converge to <0.5m
- No "toilet bowl" oscillation in Loiter (indicates yaw misalignment)
- Position should not jump when marker is lost/regained
- ~6 Hz update rate should be sufficient for stable hover

### Serial Port Test

```bash
# Check serial port exists
ls -la /dev/serial0 /dev/ttyS0 /dev/ttyAMA0

# MAVLink connection test
python3 tools/test_mavlink.py --port /dev/serial0 --baud 921600
```

---

## Camera Calibration

Camera calibration improves pose estimation accuracy. Do this once per camera.

### Remote Calibration (Recommended)

```bash
# Start camera server on RPi
ssh aruconav@aruconav.local
python3 tools/camera_server.py --port 8000

# Run calibration from local machine
python3 tools/calibrate_remote.py --host aruconav.local --port 8000
```

### Local Calibration

```bash
python3 tools/calibrate_camera.py --camera 0 --output config/camera_params.yaml
```

See [Calibration Guide](calibration.html) for detailed instructions.

---

## Troubleshooting

### Camera Issues

**"No camera found"**
```bash
ls /dev/video*
v4l2-ctl --list-devices
sudo usermod -a -G video $USER
```

**"Camera busy"**
```bash
fuser /dev/video0
sudo fuser -k /dev/video0
```

### Detection Issues

**Markers not detected**
- Check lighting (avoid shadows, reflections)
- Verify marker size parameter matches actual size (18cm)
- Try closer/farther distance (optimal: 1.5-2.5m)
- Check marker is not damaged/wrinkled
- Verify dictionary is DICT_4X4_50

**Low FPS**
- Expected: ~6 FPS on RPi Zero 2W at 720p
- Check CPU usage: `htop`
- Close other applications

### Serial/MAVLink Issues

**"Permission denied"**
```bash
sudo usermod -a -G dialout $USER
```

**"Connection timeout"**
- Check wiring (TX->RX, RX->TX)
- Verify baud rate: 921600
- Check FC is powered and running

---

*Last updated: 2026-02-09*

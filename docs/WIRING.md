# Wiring Guide: RPi Zero 2W + SpeedyBee F405 V3

## Overview

The RPi communicates with the flight controller over UART serial using MAVLink2 at 921600 baud. Only 3 wires are needed.

## Connections

```
RPi Zero 2W                    SpeedyBee F405 V3
+------------+                 +------------------+
| GPIO 14 TX |------>----------| RXn              |
| (pin 8)    |                 |                  |
|            |                 |                  |
| GPIO 15 RX |------<----------| TXn              |
| (pin 10)   |                 |                  |
|            |                 |                  |
| GND        |--------o--------| GND              |
| (pin 6)    |                 |                  |
+------------+                 +------------------+
```

**Important:** TX crosses to RX (RPi TX → FC RX, RPi RX → FC TX).

Both RPi and SpeedyBee F405 V3 use 3.3V logic levels — no level shifter needed.

## Which UART to Use

The SpeedyBee F405 V3 has multiple UARTs. Choose any free one:

| UART | ArduCopter Serial | Typical Use | Available? |
|------|-------------------|-------------|------------|
| UART1 | SERIAL1 | ESC telemetry | Usually taken |
| UART2 | SERIAL2 | GPS | Check your setup |
| UART3 | SERIAL3 | Free | Yes |
| UART4 | SERIAL4 | Free | Yes |
| UART5 | SERIAL5 | Free | Yes |
| UART6 | SERIAL6 | DJI VTX | Check your setup |

Find the TXn/RXn solder pads on your FC board for the chosen UART.

## FC Parameters (Mission Planner)

After wiring, set these parameters for the chosen UART. Example for UART4 (SERIAL4):

```
SERIAL4_PROTOCOL = 2      # MAVLink2
SERIAL4_BAUD = 921        # 921600 baud
```

Replace `4` with your UART number. Reboot the FC after changing.

### Vision Parameters

Set these in Mission Planner (Config → Full Parameter List):

```
AHRS_EKF_TYPE = 3         # Use EKF3
EK3_ENABLE = 1
EK2_ENABLE = 0
EK3_SRC1_POSXY = 6        # ExternalNav for XY position
EK3_SRC1_POSZ = 1         # Barometer for altitude
EK3_SRC1_VELXY = 0
EK3_SRC1_VELZ = 0
EK3_SRC1_YAW = 6          # ExternalNav for yaw
VISO_TYPE = 1              # MAVLink vision input
VISO_POS_M_NSE = 0.2      # Position noise (meters)
VISO_YAW_M_NSE = 0.3      # Yaw noise (radians)
VISO_DELAY_MS = 140        # Processing latency (matches RPi ~140ms)
GPS_TYPE = 0               # Disable GPS
COMPASS_ENABLE = 0         # Disable compass
COMPASS_USE = 0
COMPASS_USE2 = 0
COMPASS_USE3 = 0
```

### Camera Offset (measure on your drone)

```
VISO_POS_X = 0.0           # Camera forward from center of gravity (meters)
VISO_POS_Y = 0.0           # Camera right from center of gravity (meters)
VISO_POS_Z = 0.0           # Camera down from center of gravity (meters)
```

## RPi UART Setup

### Enable UART on RPi

```bash
sudo raspi-config
# Interface Options → Serial Port
# Login shell over serial? → No
# Enable serial hardware? → Yes
```

Or manually edit `/boot/config.txt`:
```
enable_uart=1
dtoverlay=disable-bt
```

Reboot after changes.

### Verify UART

```bash
# Check serial port exists
ls -la /dev/serial0
# Should show: /dev/serial0 -> ttyS0 (or ttyAMA0)

# Check permissions
groups
# Should include 'dialout'. If not:
sudo usermod -a -G dialout $USER
# Then logout and login again
```

## Testing the Connection

### 1. Start Vision GPS on RPi

```bash
cd /home/aruconav/aruco_drone_nav
python3 -m src --mode run
```

Expected output:
```
Connecting to /dev/serial0...
Waiting for heartbeat...
Connected to system 1
EKF origin set to (52.2297, 21.0122, 100m)
Running Vision GPS (run mode)
```

### 2. Verify in Mission Planner

Open MAVLink Inspector and look for `VISION_POSITION_ESTIMATE` messages. You should see:
- x, y, z position values changing
- Messages arriving at ~6 Hz
- yaw value matching marker orientation

### 3. Check EKF Status

In Mission Planner Messages tab, wait for:
```
EKF3 IMU0 origin set
EKF3 IMU0 is using external nav data
```

## Troubleshooting

### "No heartbeat received"
- Check wiring: TX to RX, RX to TX (crossed)
- Verify FC is powered and running ArduCopter
- Check SERIALn_PROTOCOL = 2 on the correct UART
- Check baud rate matches (921600)

### "Permission denied" on /dev/serial0
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

### No /dev/serial0
- Run `sudo raspi-config` to enable UART
- Check `/boot/config.txt` has `enable_uart=1`
- Reboot

### VISION_POSITION_ESTIMATE not appearing in Mission Planner
- Verify RPi vision GPS is running (`python3 -m src --mode run`)
- Check FC is receiving on the correct UART
- Try a different UART if unsure

---

*Last updated: 2026-02-09*

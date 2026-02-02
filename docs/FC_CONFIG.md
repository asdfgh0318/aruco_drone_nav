# Flight Controller Configuration for Vision GPS

ArduCopter parameters for using ArUco Vision GPS as the position source.

## Required Parameters

### EKF3 Setup
```
AHRS_EKF_TYPE = 3     # Use EKF3
EK3_ENABLE    = 1     # Enable EKF3
EK2_ENABLE    = 0     # Disable EKF2
```

### EKF3 Source (ExternalNav = 6)
```
EK3_SRC1_POSXY = 6    # Horizontal position from vision
EK3_SRC1_POSZ  = 1    # Altitude from barometer (safer than vision Z)
EK3_SRC1_VELXY = 0    # None (EKF derives from position updates)
EK3_SRC1_VELZ  = 0    # None
EK3_SRC1_YAW   = 6    # Yaw from vision
```

Use `EK3_SRC1_POSZ = 6` instead of `1` only if your ArUco altitude estimate is reliable and detection rate is high.

### Visual Odometry
```
VISO_TYPE      = 1     # MAVLink (VISION_POSITION_ESTIMATE)
VISO_POS_M_NSE = 0.2   # Position noise floor (meters)
VISO_YAW_M_NSE = 0.3   # Yaw noise floor (radians, ~17 degrees)
VISO_DELAY_MS  = 80    # Pipeline latency: capture + detection + send (ms)
VISO_POS_X     = 0.0   # Camera offset forward from center of gravity
VISO_POS_Y     = 0.0   # Camera offset right from center of gravity
VISO_POS_Z     = 0.0   # Camera offset down from center of gravity
```

### Disable GPS
```
GPS_TYPE = 0           # No GPS (use GPS1_TYPE on newer firmware)
```

### Disable Compass (indoor, vision provides yaw)
```
COMPASS_ENABLE = 0
COMPASS_USE    = 0
COMPASS_USE2   = 0
COMPASS_USE3   = 0
```

## Before Arming

1. **EKF origin must be set** -- the Vision GPS software sends `SET_GPS_GLOBAL_ORIGIN` automatically on startup
2. **Vision data must be streaming** at 10+ Hz before arming
3. **Wait for EKF to converge** -- watch for "EKF3 IMU0 origin set" message

## Coordinate Frame

VISION_POSITION_ESTIMATE uses NED (North-East-Down):
- x = North (meters)
- y = East (meters)
- z = Down (meters, positive toward ground)
- yaw = heading from North, clockwise (radians)

The Vision GPS software converts from its internal ENU frame automatically.

## Tuning VISO_DELAY_MS

This compensates for the time between camera capture and the FC receiving the position estimate. Too low causes oscillation, too high causes sluggish response.

Measure your pipeline latency:
1. Camera capture: ~30ms (USB camera at 30 FPS)
2. ArUco detection + pose: ~10-20ms
3. Position estimation: ~1ms
4. MAVLink send: ~5ms
5. **Total: ~50-80ms**

Start with `80` and adjust if you see position hold oscillation.

## Compatible Flight Modes

With ExternalNav configured, these modes work:
- **Loiter** -- GPS-like position hold
- **PosHold** -- Position hold with stick control
- **Guided** -- Autonomous waypoint navigation
- **Auto** -- Mission execution
- **Land** -- Automated landing
- **RTL** -- Return to launch

Fallback modes (no position needed):
- **Stabilize** -- Manual with self-leveling
- **Alt Hold** -- Manual with altitude hold

## Troubleshooting

### "PreArm: Need Position Estimate"
- Check VISO_TYPE = 1
- Check EK3_SRC1_POSXY = 6
- Verify vision data is streaming (check MAVLink Inspector for VISION_POSITION_ESTIMATE)
- Ensure EKF origin was set
- Reboot FC after parameter changes

### "Toilet Bowl" oscillation in Loiter
- Yaw misalignment between vision and FC
- Increase VISO_DELAY_MS
- Verify ENU→NED conversion is correct

### Position drifts when no markers visible
- EKF coasts on inertial data, drift is expected
- Improve marker coverage (more markers, better lighting)
- Consider using barometer for altitude (EK3_SRC1_POSZ = 1)

## References

- [ArduPilot Non-GPS Position Estimation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)
- [EKF Source Selection](https://ardupilot.org/copter/docs/common-ekf-sources.html)
- [Setting EKF Origin](https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html)

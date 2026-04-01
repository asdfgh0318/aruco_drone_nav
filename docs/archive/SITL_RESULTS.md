# SITL Validation Results

Date: 2026-02-03
ArduCopter SITL built from source (`~/ardupilot`)
Connection: `tcp:127.0.0.1:5760` (raw SITL binary)

## Test Run 1: Basic 6-Step Validation

**Config:** hover pattern, 15s duration, set params, no arm

```
==================================================
       SITL VALIDATION
==================================================
  Connection:  tcp:127.0.0.1:5760
  Pattern:     hover
  Duration:    15.0s
  Skip params: False
  Arm flight:  False

  Connect to SITL                      [PASS]
  Set FC parameters                    [PASS]
  Reboot & reconnect                   [PASS]
  Set EKF origin                       [PASS]
  Stream vision data (15s)             [PASS]
  Check EKF convergence                [PASS]

  Result: 6/6 PASSED
```

**Details:**
- All 17 FC parameters set and confirmed (AHRS_EKF_TYPE=3, EK3_SRC1_POSXY=6, VISO_TYPE=1, GPS_TYPE=0, COMPASS_ENABLE=0, etc.)
- Reconnected after reboot in ~8s
- 300 vision messages sent at 20.0 Hz
- EKF convergence: FC pos x=-0.003, y=0.003 vs sent x=0.000, y=0.000 -- **XY error: 0.004m**

**FC messages:**
```
Barometer 1 calibration complete
Barometer 2 calibration complete
ArduPilot Ready
EKF3 IMU0 origin set
EKF3 IMU1 origin set
Field Elevation Set: 100m
EKF3 IMU0 initialised
EKF3 IMU1 initialised
AHRS: EKF3 active
EKF3 IMU0 tilt alignment complete
EKF3 IMU1 tilt alignment complete
EKF3 IMU0 yaw aligned
EKF3 IMU1 yaw aligned
EKF3 IMU0 is using external nav data
EKF3 IMU1 is using external nav data
EKF3 IMU0 initial pos NED = 0.0,0.0,0.0 (m)
```

## Test Run 2: Arm + Guided Flight

**Config:** hover pattern, 15s duration, skip params (pre-configured), arm

```
==================================================
       SITL VALIDATION
==================================================
  Connection:  tcp:127.0.0.1:5760
  Pattern:     hover
  Duration:    15.0s
  Skip params: True
  Arm flight:  True

  Connect to SITL                      [PASS]
  Set EKF origin                       [PASS]
  Stream vision data (15s)             [PASS]
  Check EKF convergence                [PASS]
  Arm + guided flight                  [PASS]

  Result: 5/5 PASSED
```

**Details:**
- EKF convergence: FC pos x=-0.002, y=0.003 vs sent x=0.000, y=0.000 -- **XY error: 0.003m**
- GUIDED mode set successfully
- Armed, took off to 2.0m, hovered at 1.86m
- LAND mode commanded, touched down at 0.493 m/s
- Disarmed cleanly

**FC messages:**
```
EKF3 IMU0 is using external nav data
EKF3 IMU1 is using external nav data
Arming motors
Takeoff to 2.0m commanded
SIM Hit ground at 0.493355 m/s
Disarming motors
```

## Test Run 3: Circle Pattern

**Config:** circle pattern (1.5m radius), 20s duration, skip params, no arm

```
==================================================
       SITL VALIDATION
==================================================
  Connection:  tcp:127.0.0.1:5760
  Pattern:     circle
  Duration:    20.0s
  Skip params: True
  Arm flight:  False

  Connect to SITL                      [PASS]
  Set EKF origin                       [PASS]
  Stream vision data (20s)             [PASS]
  Check EKF convergence                [PASS]

  Result: 4/4 PASSED
```

**Details:**
- 400 vision messages sent at 20.0 Hz
- Circle: 1.5m radius, 10s period, with yaw rotation
- EKF convergence: FC pos x=-1.434, y=-0.177 vs sent x=-1.500, y=0.000 -- **XY error: 0.189m**
- Required fresh SITL instance (stale EKF state from prior arm test causes divergence)

## Summary

| Test | Steps | Result | Position Error |
|------|-------|--------|---------------|
| Basic validation | 6/6 | PASS | 0.004m |
| Arm + guided flight | 5/5 | PASS | 0.003m |
| Circle pattern | 4/4 | PASS | 0.189m |

All three core scenarios pass. The full vision-to-FC pipeline works: params accepted, EKF origin set, vision data streamed, EKF converges, vehicle arms, flies, and lands.

## Lessons Learned

1. **TCP vs UDP**: Raw SITL binary uses TCP (port 5760), not UDP. After FC soft-reboot, TCP does not reconnect cleanly. Use `--defaults` to pre-load params instead of setting them at runtime.
2. **Stale EKF state**: After an arm+flight test, the EKF retains state. A fresh SITL instance is needed for clean convergence tests.
3. **Recommended workflow**: Start SITL with pre-configured params, use `--skip-params`:
   ```bash
   cd /tmp && ~/ardupilot/build/sitl/bin/arducopter --model + --speedup 1 -I0 \
       --home 52.2297,21.0122,100,0 \
       --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm,/path/to/config/sitl_params.parm
   ```
   Then:
   ```bash
   python3 tools/test_sitl.py -v --skip-params
   python3 tools/test_sitl.py -v --skip-params --arm
   python3 tools/test_sitl.py -v --skip-params -p circle -d 20
   ```

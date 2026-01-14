# Project TODO List

## Priority: High

### Hardware Integration
- [ ] **Add wiring diagrams** - Create detailed wiring diagrams once final parts are selected
  - RPi Zero to FC serial connection
  - Camera connection
  - Power distribution
  - LiDAR integration points

- [ ] **Adjust code to final parts** - Modify configuration and code for actual hardware
  - Update serial port configuration for specific FC
  - Calibrate camera intrinsics for actual USB camera model
  - Test and tune PID gains for actual drone dynamics
  - Verify marker detection range with actual setup

### Marker Spacing Calculator
- [ ] **Create marker spacing calculator script** - Tool to calculate optimal marker placement
  - Input: camera FOV (horizontal/vertical), flight altitude, marker size
  - Output: recommended marker spacing for continuous coverage
  - Consider overlap requirements (seeing 2+ markers at once)
  - Generate ceiling layout diagram
  - Formula: `spacing = 2 * altitude * tan(FOV/2) - marker_size`

## Priority: Medium

### RPi Deployment
- [ ] **Create SD card image for RPi** - Preconfigured Raspberry Pi OS image
  - Install all dependencies
  - Configure auto-start on boot
  - Set up serial port permissions
  - Include calibration and test tools
  - Document flashing process

### LiDAR Integration
- [ ] **Add LiDAR support to the drone** - Integrate downward-facing LiDAR for altitude
  - Research compatible LiDAR sensors (TF-Luna, VL53L1X, etc.)
  - Add LiDAR driver module
  - Fuse LiDAR altitude with ArUco Z estimate
  - Improve landing detection with ground distance
  - Consider obstacle avoidance capabilities

### Testing & Validation
- [ ] **SITL testing** - Test with ArduCopter simulator before real flight
  - Create SITL configuration file
  - Test mission execution in simulation
  - Validate MAVLink command sequences

- [ ] **Real flight testing** - Systematic testing with actual hardware
  - Tethered single-marker hover test
  - Free hover test
  - Multi-marker transition test
  - Full mission execution test
  - Edge case testing (marker loss, lighting variations)

### Performance Optimization
- [ ] **Optimize for RPi Zero** - Performance improvements for constrained hardware
  - Profile CPU usage
  - Consider reduced resolution modes
  - Optimize detection parameters
  - Test headless OpenCV performance

## Priority: Low

### Features
- [ ] **Dynamic marker discovery** - Auto-detect and register new markers
- [ ] **Multi-drone support** - Coordinate multiple drones in same space
- [ ] **Real-time visualization** - Web interface for monitoring flight
- [ ] **Automatic calibration** - On-startup camera calibration check
- [ ] **Flight replay tool** - Visualize recorded flights

### Documentation
- [ ] **User manual** - Step-by-step setup guide for new users
- [ ] **Video tutorials** - Recording setup and usage demonstrations
- [ ] **API documentation** - Docstrings and Sphinx docs

### VR Integration
- [ ] **Define VR JSON schema** - Coordinate with VR team on exact format
- [ ] **Bidirectional sync** - Real-time position streaming to VR
- [ ] **Mission validation** - Check waypoints are reachable before execution

## Completed

### Phase 1 - Single Marker Navigation ✓
- [x] Camera calibration module
- [x] ArUco detection with pose estimation
- [x] Position estimator (marker-relative)
- [x] MAVLink interface
- [x] Basic hover control loop
- [x] Ground test mode

### Phase 2 - Multi-Marker Mission ✓
- [x] Marker map configuration
- [x] World position estimation
- [x] Mission executor (JSON parsing)
- [x] Waypoint navigation
- [x] Position predictor (dead reckoning)
- [x] Flight recorder

### Tools & Documentation ✓
- [x] Camera calibration tool
- [x] ArUco detection test tool
- [x] MAVLink connection test tool
- [x] Marker generator (PDF output)
- [x] README documentation
- [x] Technical documentation

---

## Notes

### Marker Spacing Quick Reference
For 60° FOV camera at 1.5m flight altitude:
- Visible area: ~1.7m diameter
- Recommended spacing: ~1.2m (30% overlap)
- Markers needed for 3m x 3m area: ~9 markers (3x3 grid)

### Hardware Candidates
- **LiDAR**: TF-Luna (8m range, $25), TFmini Plus, Garmin LIDAR-Lite
- **Camera**: Logitech C270 (common, well-supported), RPi Camera Module
- **FC**: Pixhawk 4, Kakute H7, Matek H743

---

*Last updated: 2024-01-14*

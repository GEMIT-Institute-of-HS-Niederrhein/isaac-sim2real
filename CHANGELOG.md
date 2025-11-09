# Changelog

All notable changes to the Isaac Sim2Real project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Dual-Robot Obstacle Avoidance System** - Main feature showcasing intelligent multi-robot navigation
  - 2 Jetbots with independent sensors and avoidance controllers
  - PhysX raycast-based forward sensing (60° FOV, 8m range, ~10 rays)
  - 5-state machine: FORWARD, STOP, REVERSE, TURN, EMERGENCY_STOP
  - Intelligent behaviors:
    - Gradual slowdown as obstacles approach (proactive safety zone)
    - Smart turn direction selection (analyzes left/right clearance)
    - Smooth acceleration/deceleration transitions
    - Hysteresis-based state transitions (prevents oscillation)
  - Mutual robot-to-robot avoidance
  - Hardware integration for 4 motors (IDs 1-4)
  - 13 tunable parameters for customizing behavior
  - Real-time console feedback with emojis (⚠️ ⚙️ 🔄 ↻ ✓)

- **Multi-Robot Hardware Support**
  - DynamixelController now supports 4+ motors
  - `set_wheel_velocities()` accepts `robot_id` parameter
  - `get_velocities()` accepts `robot_id` parameter
  - Per-robot motor feedback in console

- **Enhanced Documentation**
  - Completely rewritten README with dual-robot focus
  - Updated examples README with clear use-case guide
  - Created docs/archive/ for historical documentation
  - Added comprehensive project structure overview

### Changed
- **README.md** - Complete rewrite focusing on current features
  - Highlighted intelligent obstacle avoidance as main feature
  - Added multi-robot support documentation
  - Updated quick start guide
  - Cleaner project structure overview
  - Better organized troubleshooting section

- **Examples Documentation**
  - `wearhaus_room_jetbot_avoidance.py` now featured as main demo
  - Added which-example-to-use guide
  - Included parameter tuning documentation
  - Updated console output examples

### Fixed
- Motor ID configuration for dual-robot setup (IDs 1-4 all working)
- Raycast sensor parameterization to work with multiple robots
- Avoidance controller duplication for independent robot control
- Emergency stop now affects both robots simultaneously

### Deprecated
- Old LiDAR documentation (moved to docs/archive/)
  - LIDAR_AVOIDANCE_IMPLEMENTATION.md
  - LIDAR_BUGFIXES.md
  - LIDAR_PARAMETERS.md
  - LIDAR_UPDATE_FIX.md
  - SENSOR_SWITCH.md
- Old status documents (moved to docs/archive/)
  - CLEANUP_SUMMARY.md
  - ISSUE_RESOLVED.md
  - TEST_CHECKLIST.md

### Removed
- Redundant documentation from root directory (moved to archive)

## [Previous Work] - Before November 9, 2025

### Added
- Initial Isaac Sim 5.1 integration
- Dynamixel XL430-W250-T servo control
- Bidirectional sync (Sim ↔ Hardware)
- Keyboard-driven manual control
- Autonomous waypoint navigation
- Hardware diagnostic tools (ID scanner, motor tester)
- ESP32 PWM alternative control system
- Sample environment examples
- Test suite infrastructure

### Features
- 4-wheel differential drive support
- Real-time control with <100ms latency
- Emergency stop functionality
- Velocity limiting and safety features
- Modular testing capabilities
- Live monitoring and diagnostics

---

## Version History Notes

This project uses **date-based development** rather than semantic versioning until a stable 1.0 release.

### Major Milestones
- **Nov 9, 2025**: Dual-robot obstacle avoidance system completed
- **Oct 2025**: Single-robot autonomous navigation
- **Sep 2025**: Initial hardware bridge implementation
- **Aug 2025**: Project inception

---

For detailed information about any feature, see the main [README.md](README.md) or relevant documentation in `docs/`.

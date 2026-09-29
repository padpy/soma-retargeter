# Changelog

## [Unreleased]

### Added

- Bundled `booster_k1` package: Booster K1 (22 DoF, no waist, 4-DoF arms, no hands) with its URDF and meshes, scaler, retargeter and post-processing configs, and the named reference pose used for calibration. See `documentation/booster-k1.md`.
- `app/tools/robot_calibration_headless.py`: the Robot Configurator's scale, reference-pose and offset calibration without a window. It reproduces the bundled `booster_t1` scaler exactly from that robot's stored reference pose and scale.
- `app/tools/validate_retarget.py`: kinematic quality metrics for retargeted CSVs (joint-limit contact, foot penetration and sliding, airborne frames, root height, joint-speed spikes), for comparing a new target against an established one.

## [0.2.0] - 2026-09-15

### Added

- Manifest-driven robot package discovery, including custom package paths through `extra_robot_paths`.
- Bundled packages for Unitree G1 and H2, Booster T1, AgiBot A3/T3, and AgiBot X2 Ultra, with their license files.
- Robot-package selection in the viewer.
- Robot Configurator for registering URDF or MJCF robots, mapping links, mirroring mappings, and editing joint offsets.
- Example retargeted motions for every bundled robot.
- Experimental contact detection and foot-plant correction.
- Experimental IK Weight Optimizer for generating candidate weights from tracking and smoothness metrics.

### Changed

- Migrated robot assets and configurations to manifest-based packages; custom v0.1 configurations require updating.
- Improved bundled mapping, scaling, and post-processing configurations for better pose matching.
- Upgraded Newton to 1.3.0 and set the minimum required Warp version to 1.14.0.

### Fixed

- Corrected smooth joint-limit filtering for active DOF masks and configurable lower and upper offsets.
- Improved validation and error reporting for malformed BVH files and incomplete custom robot configurations.

## [0.1.0] - 2026-03-16

Initial public release.

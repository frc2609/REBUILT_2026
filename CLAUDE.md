# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository layout

```
REBUILT_2026/
  src/main/java/frc/robot/
    Robot.java / RobotContainer.java   — entry points; RobotContainer wires subsystems + controls
    RobotFactory.java                  — instantiates all IO implementations based on Mode (REAL/SIM)
    Constants.java                     — all robot-wide constants; vision constants ~lines 363-416
    generated/TunerConstants.java      — CTRE Phoenix Tuner output; DO NOT hand-edit
    subsystems/drive/                  — swerve drivetrain, PhoenixOdometryThread (high-freq sampling)
    subsystems/vision/                 — VisionSubsystem, VisionIOLimelight, VisionIOPhotonVisionSim
    subsystems/io/motor/               — generic motor IO interfaces (Percent/Velocity/Position)
    subsystems/io/encoder/             — AbsEncoderIO interface + CANCoder/DutyCycle impls
    commands/                          — DriveCommands, FullShoot, AimTurretField, intake commands
  scripts/
    replay_2d_vision.py               — offline replay: pipeline 1 via botpose, pipeline 0 via bearing tri
    train_2d_correction.py            — trains bearing_correction_net.pt from P1 log data
    bearing_correction_net.pt         — TorchScript model loaded by replay_2d_vision.py
  logs/                               — raw .wpilog files + _2d.wpilog replays
```

## Java build & deploy

```bash
cd /mnt/c/frc/REBUILT_2026   # or the Windows path C:\frc\REBUILT_2026

./gradlew build          # compile + WPILib checks
./gradlew deploy         # deploy to RoboRIO (robot on network)
./gradlew simulateJava   # run simulation (uses Constants.simMode = SIM)
```

There are no unit tests. Build errors are the primary check.

## Python scripts

Run from `REBUILT_2026/scripts/`. Dependencies: `robotpy-wpiutil`, `robotpy-wpimath`, `torch`.

```bash
# Train NN correction model from competition logs:
python3 train_2d_correction.py        # reads logs/14th/vision/*.wpilog → bearing_correction_net.pt

# Replay a log with 2D + 3D vision fusion:
python3 replay_2d_vision.py                          # uses default log path
python3 replay_2d_vision.py input.wpilog [out.wpilog]

# Output goes to *_2d.wpilog; open in AdvantageScope to compare:
#   RealOutputs/Odometry/Robot2D       — fused estimate (pipeline 0 + 1)
#   RealOutputs/Odometry/RobotOriginal — original on-robot estimate
#   RealOutputs/Vision/{Left,Front,Right}/Pipeline — pipeline indicator
```

## Architecture

### IO layer pattern (AdvantageKit)

Every hardware device has an `XxxIO` interface with a nested `XxxIOInputs` struct. `RobotFactory` selects the concrete implementation (real hardware vs sim) at startup based on `Constants.currentMode`. Subsystems call `io.updateInputs(inputs)` each loop then read from `inputs`. All inputs are auto-logged via AdvantageKit's `@AutoLog` annotation.

When adding a new hardware abstraction, follow this pattern: interface → `Inputs` struct → real impl → sim impl → wire in `RobotFactory`.

### Swerve drivetrain

- `PhoenixOdometryThread` runs at 250 Hz (CAN FD) or 100 Hz to batch-sample wheel positions and gyro; batches are queued and consumed by `DriveSubsystem.periodic()`.
- `DriveSubsystem` owns a `SwerveDrivePoseEstimator` and exposes `addVisionMeasurement()` which `VisionSubsystem` calls via a `VisionConsumer` callback injected at construction.
- Swerve module geometry and CAN IDs live in `TunerConstants` (generated); wheel radius and drive base radius are derived there.

### Vision / pose estimation

**On-robot:** `VisionSubsystem` loops over `VisionIO[]` instances. Each camera provides `PoseObservation` records. Observations are filtered (ambiguity < 0.3, |Z| < 0.75 m, field bounds) then forwarded to `DriveSubsystem.addVisionMeasurement()` with std devs scaled as `σ = baseline × dist² / tagCount`. Constants in `Constants.Vision`.

Two observation types:
- `MEGATAG_1` — full 3-D PnP (`botpose_wpiblue`), provides x/y/heading.
- `MEGATAG_2` — gyro-constrained (`botpose_orb_wpiblue`), angular σ = ∞.

**VisionIOLimelight** reads `botpose_wpiblue` / `botpose_orb_wpiblue` from NetworkTables and pushes `robot_orientation_set` for MegaTag 2. Latency correction: `obs_ts = server_ts - tl_ms * 1e-3`.

**Offline 2D replay (Python):** `replay_2d_vision.py` handles pipeline 0 (2D neural-net detection mode, no `distToCamera`). It:
1. Bootstraps position from first valid MegaTag observation (never from pipeline 0 data).
2. For pipeline 0 frames: height-triangulates distance using the unit direction vector rotated through camera mount transform, then scales so Z = tag height. Bearing = `robot_heading + cam_yaw - txnc_rad`.
3. Rate-limits bearing updates to 1 per camera per 250 ms, averaging estimates in the window.
4. Applies a trained MLP correction (`bearing_correction_net.pt`) to remove systematic bias.
5. Rejects bearing observations > 1.5 m from the current estimated position (outlier gate).
6. Fuses everything via `SwerveDrive4PoseEstimator`.

### Camera constants

Camera heights and pitches used by the 2D triangulation are **calibrated values** (not the Constants.java values, which were measured to the bellypan):

| Camera | NT key | cz (height) | pitch | yaw |
|---|---|---|---|---|
| Left | `limelight-left` | 10.5 in | −13° | +90° |
| Front | `limelight-front` | 16.25 in | 0° | 0° |
| Right | `limelight-right` | 8.25 in | −16° | −90° |

`Constants.java` still has the original (bellypan) values (Left 8 in / Right 6 in) — only update those if you intend to change the on-robot 3D pipeline behavior.

### NN correction model

- Architecture: `Linear(13→64) + BN + ReLU + Linear(64→32) + ReLU + Linear(32→16) + ReLU + Linear(16→2)`
- Features (13): `[cam0, cam1, cam2 (one-hot), avg_d_h, avg_d_h², 1/n_tags, avg_txnc_abs, avg_tync, avg_tag_z, sin(heading), cos(heading), est_x, est_y]`
- Output: `(dx, dy)` correction added to raw triangulated position
- Training source: pipeline-1 frames from competition logs at `logs/14th/vision/` (220315, 220450, 014426); 220659 is excluded due to a WPILib SIGFPE during log reading
- Trained to 95% improvement over raw triangulation (mean error 0.45 m → 0.023 m on training data)

### Heading convention

`robot_heading = raw_gyro + heading_offset`. The offset is established at first MegaTag lock:
`heading_offset = megatag_yaw − raw_gyro_at_that_time`.
All subsequent bearing calculations use `gyro[ts] + heading_offset` as field-relative heading.

### WPILog / AdvantageScope notes

- All log keys are prefixed `NT:/AdvantageKit/`.
- Gyro is at `NT:/AdvantageKit/Drive/Gyro/OdometryYawPositions` — raw (power-on relative), not field heading.
- The Python WPILog writer (`_WPILogWriter` in `replay_2d_vision.py`) is pure Python to avoid SIGFPE from C++ `DataLogWriter`. Schema records must be DATA records, not embedded in Start metadata.
- `DataLogReader` from robotpy-wpiutil raises SIGFPE on some large logs. Mitigated with `fedisableexcept(0x3F)` and a subprocess retry pattern.

## Key files to know before editing

| Task | Files |
|---|---|
| Vision std devs / rejection thresholds | `Constants.java` lines 363–416, `VisionSubsystem.java` |
| Camera transforms (3D pipeline) | `Constants.Vision.Left/Right/Front` in `Constants.java` |
| Camera params (2D triangulation) | `CAMERAS` dict in `replay_2d_vision.py` and `train_2d_correction.py` (must stay in sync) |
| Motor/encoder wiring | `RobotFactory.java` |
| Swerve config | `generated/TunerConstants.java` (do not hand-edit) |
| 2D vision fusion logic | `replay_2d_vision.py` (bearing block ~lines 491–620) |

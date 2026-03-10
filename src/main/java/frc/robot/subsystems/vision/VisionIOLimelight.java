// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Limelight vision IO that computes robot poses from raw per-tag fiducial data
 * using the Pigeon 2 gyro heading for rotation. Avoids relying on the Limelight's
 * internal PnP solver (botpose) by doing its own coordinate transforms.
 */
public class VisionIOLimelight implements VisionIO {
  private final String name;
  private final Transform3d robotToCamera;
  private final Supplier<Rotation2d> rotationSupplier;

  private final DoubleSubscriber latencySubscriber;       // tl: pipeline latency (ms)
  private final DoubleSubscriber captureLatencySubscriber; // cl: capture latency (ms)
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber rawFiducialsSubscriber;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name             The configured hostname of the Limelight.
   * @param robotToCamera    Transform3d from the robot centre to the camera (WPILib NWU frame).
   * @param rotationSupplier Supplier for the current Pigeon 2 gyro heading (yaw only).
   */
  public VisionIOLimelight(
      String name, Transform3d robotToCamera, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.name = name;
    this.robotToCamera = robotToCamera;
    this.rotationSupplier = rotationSupplier;

    latencySubscriber        = table.getDoubleTopic("tl").subscribe(0.0);
    captureLatencySubscriber = table.getDoubleTopic("cl").subscribe(0.0);
    txSubscriber             = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber             = table.getDoubleTopic("ty").subscribe(0.0);
    rawFiducialsSubscriber   =
        table.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Connection check: last NT update within 250 ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Simple angle-to-target (used by turret servoing, not pose estimation)
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()),
            Rotation2d.fromDegrees(tySubscriber.get()));

    // Total latency = pipeline latency (tl) + capture latency (cl)
    // botpose_wpiblue[6] = tl + cl; we replicate that here for rawfiducials.
    double totalLatencyMs = latencySubscriber.get() + captureLatencySubscriber.get();

    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // rawfiducials layout (7 doubles per tag, no header):
    //   [0] id   [1] txnc  [2] tync  [3] ta
    //   [4] distToCamera  [5] distToRobot  [6] ambiguity
    for (var rawSample : rawFiducialsSubscriber.readQueue()) {
      double[] rawFids = rawSample.value;
      if (rawFids.length == 0 || rawFids.length % 7 != 0) continue;

      // Retroactive capture timestamp matching what botpose code used
      double timestamp = rawSample.timestamp * 1.0e-6 - totalLatencyMs * 1.0e-3;

      int numTags = rawFids.length / 7;
      for (int t = 0; t < numTags; t++) {
        int base = t * 7;
        int    tagId       = (int) rawFids[base];
        double txncDeg     = rawFids[base + 1]; // horizontal angle, degrees
        double tyncDeg     = rawFids[base + 2]; // vertical angle, degrees
        // rawFids[base + 3] = ta (tag area, unused)
        double distToCamera = rawFids[base + 4]; // metres
        // rawFids[base + 5] = distToRobot (unused; we calculate ourselves)
        double ambiguity   = rawFids[base + 6];

        // ── Step 1: tag in Limelight camera frame (Z=depth, X=right, Y=up) ──────────
        double txncRad = Units.degreesToRadians(txncDeg);
        double tyncRad = Units.degreesToRadians(tyncDeg);
        double ll_Z = distToCamera * Math.cos(tyncRad) * Math.cos(txncRad); // depth
        double ll_X = distToCamera * Math.cos(tyncRad) * Math.sin(txncRad); // rightward
        double ll_Y = distToCamera * Math.sin(tyncRad);                     // upward

        // ── Step 2: convert to WPILib NWU camera frame (X=forward, Y=left, Z=up) ────
        Translation3d tagInCamFrame = new Translation3d(ll_Z, -ll_X, ll_Y);

        // ── Step 3: tag in robot frame via camera mount transform ────────────────────
        // Pose3d.transformBy() applies the given transform in the pose's local frame,
        // correctly rotating tagInCamFrame by the camera's orientation in robot space.
        Translation3d tagInRobot = new Pose3d(
                robotToCamera.getTranslation(), robotToCamera.getRotation())
            .transformBy(new Transform3d(tagInCamFrame, Rotation3d.kZero))
            .getTranslation();

        // ── Step 3b: ground-plane correction ────────────────────────────────────────
        // The robot is always on the ground, so tagInRobot.getZ() must equal the tag's
        // known field height.  Angular measurements (txnc/tync) are far more accurate
        // than the PnP distance, so we rescale the direction vector to satisfy the
        // Z constraint, effectively replacing the PnP distance with a height-triangulated one.
        var tagFieldPoseOpt = aprilTagLayout.getTagPose(tagId);
        if (tagFieldPoseOpt.isEmpty()) continue;
        Translation3d tagFieldPos = tagFieldPoseOpt.get().getTranslation();
        tagIds.add(tagId);

        double expectedZ  = tagFieldPos.getZ();
        double computedZ  = tagInRobot.getZ();
        double heightResidual = Math.abs(computedZ - expectedZ);

        if (Math.abs(computedZ) > 0.1 && heightResidual / Math.abs(computedZ) < 0.5) {
          double scale = expectedZ / computedZ;
          tagInRobot = new Translation3d(
              tagInRobot.getX() * scale,
              tagInRobot.getY() * scale,
              expectedZ); // exact known value; robotZ will be 0 after Step 4
        }

        // Height quality factor: larger residual → less trustworthy → inflate std dev
        // +100% per 5 cm of uncorrected height error (before correction was applied)
        double heightQualityFactor = 1.0 + (heightResidual / 0.05);

        // ── Step 4: robot field position using gyro heading ─────────────────────────
        Rotation2d heading = rotationSupplier.get();

        // Rotate the robot-relative tag vector into the field frame, then subtract from
        // the tag's known field position to get the robot's field position.
        Translation2d tagFromRobotInField = tagInRobot.toTranslation2d().rotateBy(heading);
        Translation2d robotXY = tagFieldPos.toTranslation2d().minus(tagFromRobotInField);
        double robotZ = tagFieldPos.getZ() - tagInRobot.getZ(); // ≈ 0 after correction

        Pose3d robotPose = new Pose3d(
            new Translation3d(robotXY.getX(), robotXY.getY(), robotZ),
            new Rotation3d(0.0, 0.0, heading.getRadians()));

        // ── Step 5: record observation ───────────────────────────────────────────────
        // MEGATAG_2 type → VisionSubsystem applies infinite angular std dev (gyro owns rotation)
        // distToCamera * heightQualityFactor inflates std dev when height residual is large.
        poseObservations.add(new PoseObservation(
            timestamp,
            robotPose,
            ambiguity,
            1,                               // single-tag observation
            distToCamera * heightQualityFactor,
            PoseObservationType.MEGATAG_2));
      }
    }

    // Commit results to inputs
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }
}

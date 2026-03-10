// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // ── Multi-camera cross-validation ───────────────────────────────────────────
  // When two cameras both accept an observation within 30 ms of each other,
  // |poseA - poseB|² is compared with the expected combined variance 2*(σA² + σB²).
  // The ratio drives a per-camera adaptive scale factor (low-pass, α = 0.01).
  private static final int CROSS_VAL_WINDOW = 50;
  private final double[][] cvActualResidualSq;   // rolling window of |pA-pB|² per camera
  private final double[][] cvExpectedResidualSq; // rolling window of 2*(σA²+σB²) per camera
  private final int[] cvWriteIdx;                // circular-buffer write pointer
  private final int[] cvCount;                   // valid samples in window (capped at WINDOW)
  private final double[] adaptiveScale;          // per-camera linearStdDev multiplier

  public VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // Initialize cross-validation state
    cvActualResidualSq   = new double[io.length][CROSS_VAL_WINDOW];
    cvExpectedResidualSq = new double[io.length][CROSS_VAL_WINDOW];
    cvWriteIdx           = new int[io.length];
    cvCount              = new int[io.length];
    adaptiveScale        = new double[io.length];
    Arrays.fill(adaptiveScale, 1.0);
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Accepted observations collected this cycle for cross-validation (pass 2)
    record AcceptedObs(int cam, double ts, Pose2d pose, double baseLinearStdDev) {}
    List<AcceptedObs> thisFrameObs = new ArrayList<>();

    // ── Pass 1: process each camera ─────────────────────────────────────────
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double baseLinearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev    = angularStdDevBaseline * stdDevFactor;

        if (observation.type() == PoseObservationType.MEGATAG_2) {
          baseLinearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev    *= angularStdDevMegatag2Factor; // POSITIVE_INFINITY: gyro owns rotation
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          baseLinearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev    *= cameraStdDevFactors[cameraIndex];
        }

        // Apply adaptive scale from cross-validation (previous cycle's estimate)
        double linearStdDev = baseLinearStdDev * adaptiveScale[cameraIndex];

        // Send vision observation to pose estimator
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        // Record for cross-validation (store BASE std dev before adaptive scale)
        thisFrameObs.add(new AcceptedObs(
            cameraIndex,
            observation.timestamp(),
            observation.pose().toPose2d(),
            baseLinearStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));

    // ── Pass 2: multi-camera cross-validation ────────────────────────────────
    // Compare every pair of accepted observations from DIFFERENT cameras that land
    // within 30 ms of each other.
    //
    // For two independent 2D estimates with isotropic noise σA and σB:
    //   E[|pA - pB|²] = 2·(σA² + σB²)
    //
    // We accumulate (actualResidual², expectedResidual²) in per-camera circular buffers.
    // The ratio sqrt(ΣActual / ΣExpected) drives a multiplicative low-pass update to
    // adaptiveScale[i].  ratio > 1 → cameras less accurate than assumed → inflate std dev.
    for (int a = 0; a < thisFrameObs.size(); a++) {
      for (int b = a + 1; b < thisFrameObs.size(); b++) {
        AcceptedObs A = thisFrameObs.get(a);
        AcceptedObs B = thisFrameObs.get(b);
        if (A.cam() == B.cam()) continue;
        if (Math.abs(A.ts() - B.ts()) > 0.030) continue; // 30 ms temporal window

        double actualSq = Math.pow(
            A.pose().getTranslation().getDistance(B.pose().getTranslation()), 2.0);

        // Effective std devs (including the adaptive scale already applied)
        double effA = A.baseLinearStdDev() * adaptiveScale[A.cam()];
        double effB = B.baseLinearStdDev() * adaptiveScale[B.cam()];
        double expectedSq = 2.0 * (effA * effA + effB * effB);

        // Update circular buffer for both cameras
        for (int cam : new int[]{A.cam(), B.cam()}) {
          cvActualResidualSq[cam][cvWriteIdx[cam]]   = actualSq;
          cvExpectedResidualSq[cam][cvWriteIdx[cam]] = expectedSq;
          cvWriteIdx[cam] = (cvWriteIdx[cam] + 1) % CROSS_VAL_WINDOW;
          cvCount[cam]    = Math.min(cvCount[cam] + 1, CROSS_VAL_WINDOW);
        }
      }
    }

    // Update adaptive scale factors and log them
    for (int i = 0; i < io.length; i++) {
      int n = cvCount[i];
      if (n >= 10) {
        double sumActual = 0.0, sumExpected = 0.0;
        for (int j = 0; j < n; j++) {
          sumActual   += cvActualResidualSq[i][j];
          sumExpected += cvExpectedResidualSq[i][j];
        }
        // ratio > 1: actual residuals larger than expected → std dev was too small
        double ratio = Math.sqrt(sumActual / Math.max(sumExpected, 1e-9));
        // Multiplicative low-pass: α = 0.01 (≈100-sample time constant)
        adaptiveScale[i] = MathUtil.clamp(
            adaptiveScale[i] * (0.99 + 0.01 * ratio), 0.1, 5.0);
      }
      Logger.recordOutput("Vision/Camera" + i + "/CrossVal/AdaptiveScale", adaptiveScale[i]);
      Logger.recordOutput("Vision/Camera" + i + "/CrossVal/SampleCount",   (double) cvCount[i]);
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}

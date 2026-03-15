// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  /** Set true to negate yaw (reverses rotation direction). */
  private static final boolean INVERT_YAW = false;
  /** Add this to raw yaw in degrees (e.g. 180 if Pigeon is mounted 180° rotated). */
  private static final double YAW_OFFSET_DEGREES = 180.0;

  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(DriveSubsystem.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  private static double correctYaw(double degrees) {
    double d = INVERT_YAW ? -degrees : degrees;
    d += YAW_OFFSET_DEGREES;
    return d;
  }

  /** Zeros the Pigeon so that current heading becomes 0 after correction. */
  @Override
  public void zeroYaw() {
    double pigeonZero =
        INVERT_YAW ? YAW_OFFSET_DEGREES : -YAW_OFFSET_DEGREES;
    pigeon.getConfigurator().setYaw(pigeonZero);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(correctYaw(yaw.getValueAsDouble()));
    inputs.yawVelocityRadPerSec =
        INVERT_YAW ? -Units.degreesToRadians(yawVelocity.getValueAsDouble()) : Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(correctYaw(value)))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}

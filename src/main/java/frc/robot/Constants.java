// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final String CANBUS = "CANivore";
  //public static final CANBus CANBUS = CANBus.roboRIO();

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum NeutralMode {
    BRAKE,
    COAST
  }

  public record CtreTalonFxPositionConfig(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kG,
      double cruiseVelocityRps,
      double accelerationRpsSq,
      double jerk,
      boolean forwardSoftLimitEnabled,
      double forwardSoftLimitRotations,
      boolean reverseSoftLimitEnabled,
      double reverseSoftLimitRotations,
      double supplyCurrentLimit,
      boolean supplyCurrentLimitEnabled,
      double statorCurrentLimit,
      boolean statorCurrentLimitEnabled,
      boolean inverted,
      NeutralMode neutralMode) {}

  public record CtreTalonFxVelocityConfig(
      double kP,
      double kI,
      double kD,
      double kV,
      double kS,
      double supplyCurrentLimit,
      boolean supplyCurrentLimitEnabled,
      double statorCurrentLimit,
      boolean statorCurrentLimitEnabled,
      NeutralMode neutralMode) {}

  public record CtreTalonFxPercentConfig(NeutralMode neutralMode) {}

  public record SparkMaxMotorConfig(
      boolean inverted, double kP, double kI, double kD, double kFF) {}

  public enum PositionMotorType {
    CTRE_TALON_FX,
    REV_SPARK_MAX,
    SIM
  }

  public enum VelocityMotorType {
    CTRE_TALON_FX,
    REV_SPARK_MAX,
    SIM
  }

  public enum PercentMotorType {
    CTRE_TALON_FX,
    REV_SPARK_MAX,
    SIM
  }

  public static final PositionMotorType ARM_POSITION_MOTOR_TYPE = PositionMotorType.CTRE_TALON_FX;
  public static final VelocityMotorType SHOOTER_VELOCITY_MOTOR_TYPE =
      VelocityMotorType.CTRE_TALON_FX;
  public static final PercentMotorType SIMPLE_PERCENT_MOTOR_TYPE = PercentMotorType.REV_SPARK_MAX;

  public static final class Controls {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // Example bindings / demo setpoints
    public static final double SIMPLE_MOTOR_PERCENT = 0.3;
    public static final double ARM_STOW_DEGREES = 0.0;
    public static final double SHOOTER_HOLD_RPS = 75.0;

    private Controls() {}
  }

  public static final class Shooter {
    public static final int MOTOR_ID = 40;
    public static final int FOLLOWER_ID = 41;

    public static final class TalonConfig {
      public static final double KP = 0.15;
      public static final double KI = 0.0;
      public static final double KD = 0.0;
      public static final double KV = 0.12;
      public static final double KS = 0.05;

      public static final double SUPPLY_CURRENT_LIMIT = 60.0;
      public static final boolean SUPPLY_CURRENT_LIMIT_ENABLED = true;
      public static final double STATOR_CURRENT_LIMIT = 80.0;
      public static final boolean STATOR_CURRENT_LIMIT_ENABLED = true;
      public static final boolean INVERTED = false;
    }
  }

  public static final class Vision {
    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    public static double[] cameraStdDevFactors = {1.0, 1.0};

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available

    public static final class Left {
      public static final String name = "limelight-left";
      public static Transform3d fromRobot =
          new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    }

    public static final class Right {
      public static final String name = "limelight-right";
      public static Transform3d fromRobot =
          new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));
    }
  }

  public static final class SimpleMotor {
    public static final int MOTOR_ID = 1;
    public static final boolean INVERTED = false;
  }

  /** Build the TalonFX config object for the shooter (velocity). */
  public static CtreTalonFxVelocityConfig shooterTalonFxVelocityConfig() {
    return new CtreTalonFxVelocityConfig(
        Shooter.TalonConfig.KP,
        Shooter.TalonConfig.KI,
        Shooter.TalonConfig.KD,
        Shooter.TalonConfig.KV,
        Shooter.TalonConfig.KS,
        Shooter.TalonConfig.SUPPLY_CURRENT_LIMIT,
        Shooter.TalonConfig.SUPPLY_CURRENT_LIMIT_ENABLED,
        Shooter.TalonConfig.STATOR_CURRENT_LIMIT,
        Shooter.TalonConfig.STATOR_CURRENT_LIMIT_ENABLED,
        NeutralMode.COAST);
  }

  /** Build the TalonFX config object for a simple percent-output motor. */
  public static CtreTalonFxPercentConfig simpleTalonFxPercentConfig() {
    return new CtreTalonFxPercentConfig(NeutralMode.COAST);
  }

  /** Build the SparkMax config object for a simple percent-output motor. */
  public static SparkMaxMotorConfig simpleSparkMaxConfig() {
    return new SparkMaxMotorConfig(SimpleMotor.INVERTED, 0.0, 0.0, 0.0, 0.0);
  }
}

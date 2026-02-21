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
import java.util.HashMap;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // public static final String CANBUS = "CANivore";
    public static final CANBus CANBUS = CANBus.roboRIO();

    public static final Mode simMode = Mode.SIM;
    public static final double SIM_DELTA = 0.02;

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

    public enum SimMotor
    {
        KRAKEN_X60,
        KRAKEN_X44
    }

    public static final VelocityMotorType SHOOTER_VELOCITY_MOTOR_TYPE =
        VelocityMotorType.CTRE_TALON_FX;
    public static final VelocityMotorType INTAKE_ROLLER_VELOCITY_MOTOR_TYPE =
        VelocityMotorType.CTRE_TALON_FX;
    public static final PositionMotorType INTAKE_DEPLOY_POSITION_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;
    public static final PositionMotorType AGITATOR_VELOCITY_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;
    public static final PositionMotorType CLIMBER_POSITION_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;

    public static final class Controls {
        public static final int DRIVER_CONTROLLER_PORT = 0;

        // Example bindings / demo setpoints
        public static final double SIMPLE_MOTOR_PERCENT = 0.3;
        public static final double ARM_STOW_DEGREES = 0.0;
        public static final double SHOOTER_HOLD_RPS = 75.0;

        private Controls() {}
    }

    public static final class Climber {
        public static final int EncoderChannel = 0;
        public static final double INERTIA = 1.0;
        public static final double GEAR_RATIO = 1.0;
        public static final double ENCODER_RATIO = 1.0;
        public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;

        // NOTE: Cuts off at 10 key-value pairs
        public static final Map<String, Object> Config = new HashMap<>(
            Map.of(
                "motorId", 49,
                "kP", 0.15,
                "kI", 0.0,
                "kD", 0.0,
                "kV", 0.12,
                "kS", 0.05,
                "forwardLimitEnabled", false,
                "forwardLimitRotations", 100.0,
                "reverseLimitEnabled", false,
                "reverseLimitRotations", 100.0
                // public static final double SUPPLY_CURRENT_LIMIT = 60.0;
                // public static final boolean SUPPLY_CURRENT_LIMIT_ENABLED = true;
                // public static final double STATOR_CURRENT_LIMIT = 80.0;
                // public static final boolean STATOR_CURRENT_LIMIT_ENABLED = true;
        ));
        static {
            Config.put("inverted", false);
        }
    }

    public static final class Shooter {
        public static final double INERTIA = 0.01;
        public static final double GEAR_RATIO = 1.0;
        public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;

        public static final Map<String, Object> Config =
            new HashMap<>(
                Map.of(
                    "motorId", 50,
                    "kP", 0.15,
                    "kI", 0.0,
                    "kD", 0.0,
                    "kV", 0.12,
                    "inverted", false));
    }

    public static final class Agitator {
        public static final double INERTIA = 0.01;
        public static final double GEAR_RATIO = 1.0;
        public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;

        public static final Map<String, Object> MotorConfig =
            new HashMap<>(
                Map.of(
                    "motorId", 43,
                    "kP", 0.15,
                    "kI", 0.0,
                    "kD", 0.0,
                    "kV", 0.12,
                    "inverted", false));
    }

    public static final class Intake {
        public static final int EncoderChannel = 0;

        public static final double Roller_INERTIA = 0.01;
        public static final double Roller_GEAR_RATIO = 1.0;
        public static final SimMotor Roller_SIM_MOTOR = SimMotor.KRAKEN_X60;

        public static final double Deploy_INERTIA = 1.0;
        public static final double Deploy_GEAR_RATIO = 1.0;
        public static final double Deploy_ENCODER_RATIO = 1.0;
        public static final SimMotor Deploy_SIM_MOTOR = SimMotor.KRAKEN_X60;

        public static final Map<String, Object> RollerConfig =
            new HashMap<>(
                Map.of(
                    "motorId", 47,
                    "kP", 0.15,
                    "kI", 0.0,
                    "kD", 0.0,
                    "kV", 0.12,
                    "inverted", false));
        // NOTE: Cuts off at 10 key-value pairs
        public static final Map<String, Object> DeployConfig =
            new HashMap<>(
                Map.of(
                    "motorId",46,
                    "kP",0.15,
                    "kI",0.0,
                    "kD",0.0,
                    "kV",0.12,
                    "kS",0.05,
                    "forwardLimitEnabled",false,
                    "forwardLimitRotations",100.0,
                    "reverseLimitEnabled",false,
                    "reverseLimitRotations",100.0
                    // public static final double SUPPLY_CURRENT_LIMIT = 60.0;
                    // public static final boolean SUPPLY_CURRENT_LIMIT_ENABLED = true;
                    // public static final double STATOR_CURRENT_LIMIT = 80.0;
                    // public static final boolean STATOR_CURRENT_LIMIT_ENABLED = true;
            ));
        static {
            DeployConfig.put("inverted", false);
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
}

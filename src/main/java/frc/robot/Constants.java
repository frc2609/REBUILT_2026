// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Conversions;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final CANBus CANBUS = new CANBus("CANivore");
    public static final CANBus RioCANBUS = CANBus.roboRIO();

    public static final Mode simMode = Mode.SIM;
    public static final double SIM_DELTA = 0.01;

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

    public static final Map<Integer, String> motorNames = 
        Map.of(
            20, "Spindexer",
            21, "Feeder",
            30, "Intake/Deploy",
            31, "Intake/Roller",
            40, "Climber",
            50, "Flywheel",
            51, "Flywheel(f)",
            52, "Turret/Hood",
            53, "Turret/Azimuth"
        );

    public static final String[] tunableKeys = 
        {"kP", "kI", "kD", "kA", "kV", "kS"};

    public static final VelocityMotorType FLYWHEEL_VELOCITY_MOTOR_TYPE =
        VelocityMotorType.CTRE_TALON_FX;
    public static final VelocityMotorType FEED_VELOCITY_MOTOR_TYPE =
        VelocityMotorType.CTRE_TALON_FX;
    public static final VelocityMotorType INTAKE_ROLLER_VELOCITY_MOTOR_TYPE =
        VelocityMotorType.CTRE_TALON_FX;
    public static final PositionMotorType INTAKE_DEPLOY_POSITION_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;
    public static final PositionMotorType AGITATOR_VELOCITY_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;
    public static final PositionMotorType CLIMBER_POSITION_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;
    public static final PositionMotorType TURRET_AIM_POSITION_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;
    public static final PositionMotorType TURRET_HOOD_POSITION_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;

    public static final class Controls {
        public static final int DRIVER_CONTROLLER_PORT = 0;

        // Raw Abs Encoder output
        public static final double INTAKE_DEPLOYED_DEG = 0.8 * 360;
        public static final double INTAKE_RETRACT_DEG  = 0.5 * 360;
        public static final double INTAKE_RUN_RPM = 4500.0;
        public static final double INTAKE_IDLE_RPM = 0.0;

        public static final double CLIMBER_DEPLOYED_DEG = 360;

        public static final double TURRET_AIM_DEG = 190.0;
        public static final double TURRET_HOOD_DEG = 10;

        public static final double AGITATOR_HOLD_RPM = 500.0;
        public static final double FEED_HOLD_RPM = 2500.0; // max speed
        public static final double FLYWHEEL_HOLD_RPM = 1500.0;

    }

    // NOTE: the pid values are not correct, nor are the limits

    public static final class Climber {
        public static final int EncoderChannel = 1;
        public static final double INERTIA = 0.01;
        public static final double GEAR_RATIO = 1.0;
        public static final double ENCODER_RATIO = 1.0;
        public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;

        // NOTE: Cuts off at 10 key-value pairs
        public static final Map<String, Object> config = new HashMap<>(
            Map.of(
                "motorId", 40,
                "kP", 0.0,
                "kI", 0.0,
                "kD", 0.0,
                "kV", 0.0,
                "kS", 0.0,
                "forwardLimitEnabled", false,
                "forwardLimitRotations", 100.0,
                "reverseLimitEnabled", false,
                "reverseLimitRotations", 100.0
        ));
        static {
            config.put("inverted", false);
            config.put("supplyCurrentLimit", 60.0);
            config.put("supplyCurrentLimitEnabled", true);
            config.put("statorCurrentLimit", 80.0);
            config.put("statorCurrentLimitEnabled", true);
            config.put("MotionMagicCruiseVelocity", 2.0);
            config.put("MotionMagicAcceleration", 1.0);
        }
    }

    public static final class Feed {
        public static final double INERTIA = 0.01;
        public static final double GEAR_RATIO = 1.0;
        public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;
        public static final Map<String, Object> config = new HashMap<>(Map.of(
            "motorId", 21,
            "kP", 0.04,
            "kV", 0.0097,
            "inverted", true
        ));
    }

    public static final class Flywheel {
        public static final double INERTIA = 0.01;
        public static final double GEAR_RATIO = 1.0;
        public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;
        public static final Map<String, Object> config = new HashMap<>(Map.of(
            "motorId", 50,
            "followerId", 51,
            "followerAligned", false,
            "kP", 0.04,
            "kV", 0.0097,
            "inverted", false
        ));
    }

    public static final class Turret {
        public static final int EncoderChannel = 0;

        public static final class Aim {
            public static final double INERTIA = 0.01;
            public static final double GEAR_RATIO = 5.0;
            public static final double ENCODER_RATIO = 1.0;
            public static final double ZERO_OFFSET = Conversions.degreesToRotations(77.0, GEAR_RATIO);
            public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X44;
            public static final Map<String, Object> config = new HashMap<>(Map.of(
                "motorId", 53,
                "kP", 0.15,
                "kI", 0.0,
                "kD", 0.0,
                "kA", 0.0,
                "kV", 0.12,
                "kS", 0.05
            ));
            static {
                config.put("MotionMagicCruiseVelocity", 2.0);
                config.put("MotionMagicAcceleration", 1.0);

                config.put("forwardLimitEnabled", true);
                config.put("forwardLimitRotations",
                    Conversions.degreesToRotations(200.0, GEAR_RATIO));
                config.put("reverseLimitEnabled", true);
                config.put("reverseLimitRotations",
                    Conversions.degreesToRotations(-200.0, GEAR_RATIO));
                
                config.put("inverted", false);
                config.put("supplyCurrentLimit", 60.0);
                config.put("supplyCurrentLimitEnabled", true);
                config.put("statorCurrentLimit", 80.0);
                config.put("statorCurrentLimitEnabled", true);
            }
        }

        public static final class Hood {
            public static final double INERTIA = 0.01;
            public static final double GEAR_RATIO = 1.0;
            public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X44;
            public static final Map<String, Object> config = new HashMap<>(Map.of(
                "motorId", 52,
                "kP", 0.15,
                "kI", 0.0,
                "kD", 0.0,
                "kA", 0.0,
                "kV", 0.12,
                "kS", 0.05
            ));
            static {
                config.put("MotionMagicCruiseVelocity", 2.0);
                config.put("MotionMagicAcceleration", 1.0);
                config.put("forwardLimitEnabled", true);
                config.put("reverseLimitEnabled", true);

                config.put("forwardLimitRotations",
                    Conversions.degreesToRotations(20.0, GEAR_RATIO));
                config.put("reverseLimitRotations",
                    Conversions.degreesToRotations(0.0, GEAR_RATIO));
                
                config.put("inverted", false);
                config.put("supplyCurrentLimit", 60.0);
                config.put("supplyCurrentLimitEnabled", true);
                config.put("statorCurrentLimit", 80.0);
                config.put("statorCurrentLimitEnabled", true);
            }
        }
    }

    public static final class Agitator {
        public static final double INERTIA = 0.001;
        public static final double GEAR_RATIO = 1.0;
        public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;

        public static final Map<String, Object> config = new HashMap<>(Map.of(
            "motorId", 20,
            "kP", 0.05,
            "kV", 0.0097,
            "inverted", false
        ));
    }

    public static final class Intake {
        public static final int EncoderChannel = 32;

        public static final class Roller {
            public static final double INERTIA = 0.001;
            public static final double GEAR_RATIO = 1.0;
            public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;
            public static final Map<String, Object> config = new HashMap<>(Map.of(
                "motorId", 31,
                "isRioCANBUS",true,
                "inverted", true,
                "kP", 0.032,
                "kV", 0.0097
            ));
        }

        public static final class Deploy {
            public static final double INERTIA = 0.001;
            public static final double GEAR_RATIO = 1.0;
            public static final double ENCODER_RATIO = 1.0;
            public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;
            
            // NOTE: Cuts off at 10 key-value pairs
            public static final Map<String, Object> config = new HashMap<>(Map.of(
                "motorId",30,
                "kP", 0.15,
                "kI", 0.0,
                "kD", 0.0,
                "kA", 0.0,
                "kV", 0.12,
                "kS", 0.05
            ));
            static {
                config.put("MotionMagicCruiseVelocity", 2.0);
                config.put("MotionMagicAcceleration", 1.0);
                config.put("forwardLimitEnabled", true);
                config.put("reverseLimitEnabled", true);

                // TalonFX outputted rotations
                config.put("forwardLimitRotations", 0.836);
                config.put("reverseLimitRotations", 0.45);
                
                config.put("inverted", false);
                config.put("supplyCurrentLimit", 60.0);
                config.put("supplyCurrentLimitEnabled", true);
                config.put("statorCurrentLimit", 80.0);
                config.put("statorCurrentLimitEnabled", true);
            }
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

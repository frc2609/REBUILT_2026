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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Conversions;
import frc.robot.util.ProjectileSimulator;
import frc.robot.util.ShotCalculator;

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
        CTRE_TALON_FX_MM,
        CTRE_TALON_FX_FOC,
        CTRE_TALON_FX_EXPO,
        REV_SPARK_MAX,
        SIM
    }

    public enum VelocityMotorType {
        CTRE_TALON_FX,
        CTRE_TALON_FX_FOC,
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
        {"kP", "kI", "kD", "kA", "kV", "kS", "kG"};

    public static final VelocityMotorType FLYWHEEL_VELOCITY_MOTOR_TYPE =
        VelocityMotorType.CTRE_TALON_FX_FOC;
    public static final VelocityMotorType FEED_VELOCITY_MOTOR_TYPE =
        VelocityMotorType.CTRE_TALON_FX;
    public static final PercentMotorType INTAKE_ROLLER_PERCENT_MOTOR_TYPE =
        PercentMotorType.CTRE_TALON_FX;
    public static final PositionMotorType INTAKE_DEPLOY_POSITION_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;
    public static final PositionMotorType AGITATOR_VELOCITY_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;
    public static final PositionMotorType CLIMBER_POSITION_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;
    public static final PositionMotorType TURRET_AIM_POSITION_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX_MM;
    public static final PositionMotorType TURRET_HOOD_POSITION_MOTOR_TYPE =
        PositionMotorType.CTRE_TALON_FX;

    public static final class Field
    {
        public static final Translation2d RED_HUB = new Translation2d(11.92, 4.033);
        public static final Translation2d RED_HUB_FORWARD = new Translation2d(-1.0, 0.0);

        public static final Translation2d BLUE_HUB = new Translation2d(4.625, 4.033);
        public static final Translation2d BLUE_HUB_FORWARD = new Translation2d(1.0, 0.0);

        public static final double FIELD_LENGTH = 16.54;
        public static final double FIELD_WIDTH = 8.07;
        public static final double CENTER_Y = FIELD_WIDTH / 2.0;

        // Zone boundaries - red derived as true field mirror of blue
        public static final double BLUE_ZONE_X = 4.5;
        public static final double BLUE_BLOCK_X = 5.5; // under trench, aim but don't shoot
        public static final double BLUE_CLOSE_ZONE_X = 7.25;
        public static final double RED_ZONE_X = FIELD_LENGTH - BLUE_ZONE_X;
        public static final double RED_BLOCK_X = FIELD_LENGTH - BLUE_BLOCK_X;
        public static final double RED_CLOSE_ZONE_X = FIELD_LENGTH - BLUE_CLOSE_ZONE_X;

        // Tune these two - all other pass targets are derived from them.
        // RIGHT = low Y side (near scoring table), LEFT = high Y side.
        public static final Translation2d BLUE_CLOSE_PASS_RIGHT = new Translation2d(4.0, 2.5);
        public static final Translation2d BLUE_PASS_RIGHT       = new Translation2d(2.0, 2.5);

        // Derived: Y mirror = FIELD_WIDTH - Y, X mirror = FIELD_LENGTH - X
        public static final Translation2d BLUE_CLOSE_PASS_LEFT = new Translation2d(
            BLUE_CLOSE_PASS_RIGHT.getX(), FIELD_WIDTH - BLUE_CLOSE_PASS_RIGHT.getY());
        public static final Translation2d RED_CLOSE_PASS_RIGHT = new Translation2d(
            FIELD_LENGTH - BLUE_CLOSE_PASS_RIGHT.getX(), BLUE_CLOSE_PASS_RIGHT.getY());
        public static final Translation2d RED_CLOSE_PASS_LEFT = new Translation2d(
            FIELD_LENGTH - BLUE_CLOSE_PASS_RIGHT.getX(), FIELD_WIDTH - BLUE_CLOSE_PASS_RIGHT.getY());

        public static final Translation2d BLUE_PASS_LEFT = new Translation2d(
            BLUE_PASS_RIGHT.getX(), FIELD_WIDTH - BLUE_PASS_RIGHT.getY());
        public static final Translation2d RED_PASS_RIGHT = new Translation2d(
            FIELD_LENGTH - BLUE_PASS_RIGHT.getX(), BLUE_PASS_RIGHT.getY());
        public static final Translation2d RED_PASS_LEFT = new Translation2d(
            FIELD_LENGTH - BLUE_PASS_RIGHT.getX(), FIELD_WIDTH - BLUE_PASS_RIGHT.getY());
    }

    public static final class Controls {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double SHOOTING_SPEED_PERCENT = 0.1;
        public static final double UNJAM_FACTOR = 10.0; // kP multiplier when unjamming
        public static final double SHOT_CONFIDENCE_MIN = 50.0; // out of 100
        public static final double STATIONARY_SPEED = 0.05;

        public static final double TURRET_READY_TOLERANCE = 4.0; // deg
        public static final double FLYWHEEL_PASS_TOLERANCE_RPM = 300.0; // rpm
        public static final double FLYWHEEL_TOLERANCE_RPM = 100.0; // rpm

        // Rotation values are OUTPUT degrees
        // RPM values are INPUT RPM, will be geared down


        public static final double INTAKE_DEPLOYED_DEG = 695.0;
        public static final double INTAKE_RUN_PERCENT = 0.7;
        public static final double INTAKE_SPIT_PERCENT = -0.5;

        public static final double TURRET_HOOD_DEG = 15.0;
        public static final double TURRET_HOOD_MAX_DEG = 15.0;

        public static final double TURRET_OVERRIDE_FRONT_DEG = 0.0;
        public static final double TURRET_OVERRIDE_RIGHT_DEG = 170.0;
        public static final double TURRET_OVERRIDE_LEFT_DEG = -170.0;

        public static final double AGITATOR_HOLD_RPM = 5000.0;
        public static final double FEED_HOLD_RPM = 5000.0;

        public static final double FLYWHEEL_LOB_RPM = 2000.0;
        public static final double LOB_DISTANCE = 2.0;
    }

    /** BLine FollowPath PID gains. Path constraints are in deploy/autos/config.json. */
    public static final class BLine {
        public static final double PID_TRANSLATION_KP = 10.0;
        public static final double PID_TRANSLATION_KI = 0.0;
        public static final double PID_TRANSLATION_KD = 0.0;
        public static final double PID_ROTATION_KP = 10.0;
        public static final double PID_ROTATION_KI = 0.0;
        public static final double PID_ROTATION_KD = 0.0;
        public static final double PID_CROSS_TRACK_KP = 2.0;
        public static final double PID_CROSS_TRACK_KI = 0.0;
        public static final double PID_CROSS_TRACK_KD = 0.0;
    }

    // NOTE: the pid values are not correct, nor are the limits

    // On the fly settings 

    public static final ProjectileSimulator.SimParameters simParameters = 
        new ProjectileSimulator.SimParameters(
            0.215,   // ball mass kg
            0.1501,  // ball diameter m
            0.47,    // drag coeff (smooth sphere)
            0.0,     // Magnus coeff
            1.225,   // air density
            0.353,    // exit height (m), floor to where the ball leaves the shooter
            0.0762,  // flywheel diameter, 0.0762
            1.91,    // target height (m), 1.83 from game manual
            0.9,     // slip factor (0=no grip, 1=perfect), tune this on the real robot
            71.0,    // launch angle from horizontal
            0.001,   // sim timestep
            1500, 6000, 25, 10.0  // RPM search range, iterations, max sim time
        );

    public static final ShotCalculator.Config shotConfig = new ShotCalculator.Config();
    static {
        shotConfig.launcherOffsetX = -0.119;  // how far forward the launcher is from robot center (m)
        shotConfig.launcherOffsetY = -0.152;   // how far left, 0 if centered
        //shotConfig.shooterAngleOffsetRad = Math.PI; // use LoggedNetworkNumber SOTM/HeadingOffset instead
        shotConfig.phaseDelayMs = 30.0;     // your vision pipeline latency
        shotConfig.mechLatencyMs = 20.0;    // how long the mechanism takes to respond
        shotConfig.maxTiltDeg = 5.0;        // suppress firing when chassis tilts past this (bumps/ramps)
        shotConfig.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to disable)
        shotConfig.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub
        shotConfig.maxScoringDistance = 20.0;
        shotConfig.tofMax = 10.0;
        shotConfig.maxSOTMSpeed = 10.0;
    }
    
    // Subsystems

    public static final class Feed {
        public static final double INERTIA = 0.01;
        public static final double GEAR_RATIO = 25.0/12.0;
        public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;
        public static final Map<String, Object> config = new HashMap<>(Map.of(
            "motorId", 21,
            "kP", 0.04,
            "kV", 0.0113,
            "inverted", true,
            "neutralMode", NeutralMode.COAST
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
            "kP", 110.0,
            "kS", 14.47,
            "kV", 0.17769,
            "peakForwardTorqueCurrent", 97.0,
            "peakReverseTorqueCurrent", -10.0
        ));
        static {
            // Trapezoidal profile config (used by MotionMagic firmware)
            config.put("inverted", currentMode != Mode.SIM);
            config.put("neutralMode", NeutralMode.COAST);
            config.put("statorCurrentLimit", 94.0);
            config.put("statorCurrentLimitEnabled", true);
        }
    }

    public static final class Turret {
        public static final int EncoderChannel = 32;

        public static final class Aim {
            public static final double INERTIA = 0.01;
            public static final double GEAR_RATIO = 137.5;
            public static final double ENCODER_RATIO = 1.0;
            public static final double RANGE_DEG = 100.0; // 160
            public static final double HEADING_OFFSET_DEG = -90.0; // robot front to turret zero
            public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;
            // MotionMagic trapezoidal profile limits (rotor rotations/sec, /sec^2, /sec^3)
            // Tuned via physics sim: 2.2x faster settling, 0.02° overshoot, lowest energy
            public static final double MAX_VELOCITY = 200.0;   // rotor rot/s (~600 deg/s mechanism)
            public static final double MAX_ACCEL = 500.0;      // rotor rot/s^2

            public static final Map<String, Object> config = new HashMap<>(Map.of(
                "motorId", 53,
                "kP", 5.0,
                "kD", 0.03,
                "kS", 0.005
            ));

            static {
                // Trapezoidal profile config (used by MotionMagic firmware)
                config.put("MotionMagicCruiseVelocity", MAX_VELOCITY);
                config.put("MotionMagicAcceleration", MAX_ACCEL);
                config.put("MotionMagicJerk", 0.0);  // 0 = unlimited = pure trapezoidal
                // Expo profile shape params (only used by DynamicMotionMagicExpoVoltage)
                config.put("MotionMagicExpo_kV", 0.124); // matches Slot0 kV (12V / 96.67 RPS)
                config.put("MotionMagicExpo_kA", 0.01);  // start low, tune up if response is sluggish

                config.put("forwardLimitEnabled", true);
                config.put("forwardLimitRotations",
                    Conversions.degreesToRotations(RANGE_DEG, GEAR_RATIO));
                config.put("reverseLimitEnabled", true);
                config.put("reverseLimitRotations",
                    Conversions.degreesToRotations(-RANGE_DEG, GEAR_RATIO));
                
                config.put("neutralMode", Constants.NeutralMode.BRAKE);
                config.put("inverted", true);
                config.put("supplyCurrentLimit", 30.0);
                config.put("supplyCurrentLimitEnabled", true);
                config.put("statorCurrentLimit", 30.0);
                config.put("statorCurrentLimitEnabled", true);
                config.put("useClosedLoopFFSign", true);
            }

            // FOC (TorqueCurrentFOC) config — gains in Amps, not Volts
            // Derived from voltage gains: kP_A = kP_V/R, kD_A = Kv*G/R, kS_A = kS_V/R
            // Kraken X60: R = 0.0248 Ω, Kv = 0.124 V·s/rot
            public static final Map<String, Object> configFOC = new HashMap<>(Map.of(
                "motorId", 53,
                "kP", 29.0,    // 0.72 V / 0.0248 Ω
                "kD", 5.0,     // Kv * G / R = back-EMF equivalent damping
                "kS", 2.4      // 0.06 V / 0.0248 Ω
            ));
            static {
                configFOC.put("MotionMagicCruiseVelocity", MAX_VELOCITY);
                configFOC.put("MotionMagicAcceleration", MAX_ACCEL);
                configFOC.put("MotionMagicJerk", 0.0);

                configFOC.put("forwardLimitEnabled", true);
                configFOC.put("forwardLimitRotations",
                    Conversions.degreesToRotations(RANGE_DEG, GEAR_RATIO));
                configFOC.put("reverseLimitEnabled", true);
                configFOC.put("reverseLimitRotations",
                    Conversions.degreesToRotations(-RANGE_DEG, GEAR_RATIO));

                configFOC.put("neutralMode", Constants.NeutralMode.BRAKE);
                configFOC.put("inverted", false);
                configFOC.put("supplyCurrentLimit", 30.0);
                configFOC.put("supplyCurrentLimitEnabled", true);
                configFOC.put("statorCurrentLimit", 30.0);
                configFOC.put("statorCurrentLimitEnabled", true);
                configFOC.put("useClosedLoopFFSign", true);
            }
        }

        public static final class Hood {
            public static final double INERTIA = 0.01;
            public static final double GEAR_RATIO = 19.0;

            // Homing: drive the hood past zero until current spikes, then zero there
            public static final double HOME_TARGET_DEG = -90.0;        // well below the down stop
            public static final double HOME_CURRENT_THRESHOLD_AMPS = 20.0; // stall spike threshold
            public static final int    HOME_CONFIRM_CYCLES = 3;        // cycles above threshold to confirm stall
            public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X44;
            public static final Map<String, Object> config = new HashMap<>(Map.of(
                "motorId", 52,
                "kP", 0.4,
                "kD", 0.0,
                "kS", 0.03
            ));
            static {
                // config.put("MotionMagicCruiseVelocity", 2.0);
                // config.put("MotionMagicAcceleration", 1.0);
                config.put("forwardLimitEnabled", false);
                config.put("reverseLimitEnabled", false);

                config.put("forwardLimitRotations", 0.92);
                config.put("reverseLimitRotations", 0.0);
                
                config.put("inverted", false);
                config.put("supplyCurrentLimit", 40.0);
                config.put("supplyCurrentLimitEnabled", true);
                config.put("statorCurrentLimit", 40.0);
                config.put("statorCurrentLimitEnabled", true);
                config.put("useClosedLoopFFSign", true);
                config.put("neutralMode", Constants.NeutralMode.BRAKE);
            }
        }
    }

    public static final class Agitator {
        public static final double INERTIA = 0.001;
        public static final double GEAR_RATIO = 80.0/3.0;
        public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;
        public static final double JAM_CURRENT = 100.0; // stator limit before unjam
        public static final double UNJAM_TIME  = 3.0; // seconds

        public static final Map<String, Object> config = new HashMap<>(Map.of(
            "motorId", 20,
            "kP", 0.05,
            "kV", 0.012,
            "inverted", true,
            "neutralMode", NeutralMode.COAST,
            "statorCurrentLimit", 120.0,
            "statorCurrentLimitEnabled", true
        ));
    }

    public static final class Intake {
        public static final int EncoderChannel = 0;

        public static final class Roller {
            public static final double INERTIA = 0.001;
            public static final double GEAR_RATIO = 3.0;
            public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;
            public static final Map<String, Object> config = new HashMap<>(Map.of(
                "motorId", 31,
                "followerId", 32,
                "followerAligned", false,
                "isRioCANBUS",true,
                "inverted", false,
                "kP", 0.032,
                "kV", 0.0097,
                "statorCurrentLimit", 50.0,
                "statorCurrentLimitEnabled", true
            ));
        }

        public static final class Deploy {
            public static final double INERTIA = 0.001;
            public static final double ZERO_OFFSET = 0;
            public static final double GEAR_RATIO = 12.0;
            public static final double ENCODER_RATIO = 1.0;
            public static final double OUT_ROTATIONS = 22.7;
            public static final SimMotor SIM_MOTOR = SimMotor.KRAKEN_X60;

            // Homing: drive past the deployed hard stop until current spikes, then zero there
            public static final int    HOME_CONFIRM_CYCLES = 3;

            public static final double HOME_TARGET_DEG = 690.0;
            public static final double HOME_CURRENT_THRESHOLD_AMPS = 3.0;
            public static final double HOME_STALL_MAX_VEL_DEG_PER_SEC = 8.0;
            
            // NOTE: Cuts off at 10 key-value pairs
            public static final Map<String, Object> config = new HashMap<>(Map.of(
                "motorId",30,
                "kP", 0.08,
                "kD", 0.0,
                "kG", 0.03,
                "kS", 0.06
            ));
            static {
                // config.put("MotionMagicCruiseVelocity", 2.0);
                // config.put("MotionMagicAcceleration", 1.0);
                config.put("forwardLimitEnabled", true);
                config.put("reverseLimitEnabled", false);

                // TalonFX outputted rotations
                config.put("forwardLimitRotations", 
                    Conversions.degreesToRotations(1340.0, GEAR_RATIO)
                );
                config.put("reverseLimitRotations", -1.5);

                config.put("neutralMode", Constants.NeutralMode.BRAKE);
                
                config.put("inverted", true);
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
        public static double linearStdDevBaseline = 0.06; // Meters
        public static double angularStdDevBaseline = 0.06; // Radians

        public static double[] cameraStdDevFactors = {1.0, 1.0};

        // Multipliers to apply for MegaTag 2 observations
        public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available

        public static final class Left {
            public static final String name = "limelight-left";
            // User: x=13in(left), y=-10.5in(back), z=8in(up), pitch=10deg(up), yaw=left(90deg)
            public static final Transform3d fromRobot = new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10.5),  // WPILib X = user Y (forward)
                    Units.inchesToMeters(13.0),   // WPILib Y = user X (left)
                    Units.inchesToMeters(8.0)),   // WPILib Z = user Z (up)
                new Rotation3d(0.0, Units.degreesToRadians(-10.0), Math.PI / 2.0));
        }

        public static final class Right {
            public static final String name = "limelight-right";
            // User: x=-13in(right), y=-10.375in(back), z=6in(up), pitch=10deg(up), yaw=right(-90deg)
            public static final Transform3d fromRobot = new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10.375), // WPILib X = user Y (forward)
                    Units.inchesToMeters(-13.0),   // WPILib Y = user X (left, negative=right)
                    Units.inchesToMeters(6.0)),    // WPILib Z = user Z (up)
                new Rotation3d(0.0, Units.degreesToRadians(-10.0), -Math.PI / 2.0));
        }

        public static final class Front {
            public static final String name = "limelight-front";
            // User: x=0(center), y=2.75in(forward), z=16.25in(up), pitch=0(deg), yaw=forward(0deg)
            public static final Transform3d fromRobot = new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(2.75),    // WPILib X = user Y (forward)
                    0.0,                           // WPILib Y = user X (center)
                    Units.inchesToMeters(16.25)),  // WPILib Z = user Z (up)
                new Rotation3d(0.0, 0.0, 0.0));
        }

    }

    public static final class LedConstants{
        public static final int Length = 84;
        public static final int Port = 0;
        public static final double travelTime = 1.25;//time to go from one end to the other in seconds
        public static final int ledGroup = 3;
    }
}

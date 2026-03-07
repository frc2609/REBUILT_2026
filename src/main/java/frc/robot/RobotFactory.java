package frc.robot;

import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;
import frc.robot.subsystems.io.encoder.impl.CANCoderIO;
import frc.robot.subsystems.io.encoder.impl.SimAbsEncoderIO;
import frc.robot.subsystems.io.encoder.impl.WpiDutyCycleEncoderIO;
import frc.robot.subsystems.io.motor.PercentMotorIO;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.subsystems.io.motor.VelocityMotorIO;
import frc.robot.subsystems.io.motor.CTRE.CtreTalonFxPercentIO;
import frc.robot.subsystems.io.motor.CTRE.CtreTalonFxPositionIO;
import frc.robot.subsystems.io.motor.CTRE.CtreTalonFxVelocityIO;
import frc.robot.subsystems.io.motor.CTRE.Sim.SimPercentMotorIO;
import frc.robot.subsystems.io.motor.CTRE.Sim.SimPositionMotorIO;
import frc.robot.subsystems.io.motor.CTRE.Sim.SimVelocityMotorIO;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotFactory {
    private final TurretSubsystem turretSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final FeedSubsystem feedSubsystem;
    private final ClimberSubsystem climberSubsystem;

    private final Mode currentMode;

    public RobotFactory() {
        currentMode = Constants.currentMode;

        driveSubsystem = new DriveSubsystem(
            buildGyroIO(), buildModuleIO()
        );
        visionSubsystem = new VisionSubsystem(
            driveSubsystem::addVisionMeasurement, buildVisionIO()
        );
        turretSubsystem = new TurretSubsystem(
            buildTurretAimIO(), buildTurretHoodIO(), buildTurretEncoderIO()
        );
        flywheelSubsystem = new FlywheelSubsystem(
            buildFlywheelIO()
        );
        intakeSubsystem = new IntakeSubsystem(
            buildIntakeEncoderIO(), buildIntakeDeployIO(), buildIntakeRollerIO()
        );
        climberSubsystem = new ClimberSubsystem(
            buildClimberEncoderIO(), buildClimberMotorIO()
        );
        feedSubsystem = new FeedSubsystem(
            buildAgitatorIO(), buildFeedIO()
        );
    }

    // CLIMBER

    public ClimberSubsystem getClimberSubsystem() {
        return this.climberSubsystem;
    }

    private AbsEncoderIO buildClimberEncoderIO() {
        if (currentMode == Mode.SIM) {
            return new SimAbsEncoderIO(0);
        }

        return new WpiDutyCycleEncoderIO(Constants.Climber.EncoderChannel);
    }

    private PositionMotorIO buildClimberMotorIO() {
        if (currentMode == Mode.SIM) {
            return new SimPositionMotorIO(
                Constants.Climber.config,
                Constants.Climber.INERTIA,
                Constants.Climber.GEAR_RATIO,
                Constants.Climber.ENCODER_RATIO, 
                Constants.Climber.SIM_MOTOR, 
                Constants.SIM_DELTA);
        }
        switch (Constants.CLIMBER_POSITION_MOTOR_TYPE) {
            case CTRE_TALON_FX:
                return new CtreTalonFxPositionIO(
                    Constants.Climber.config,
                    Constants.Climber.GEAR_RATIO,
                    Constants.Climber.ENCODER_RATIO);
            default:
                throw new IllegalStateException("Unsupported intake deploy motor type");
        }
    }

    // FEED / AGITATOR

    public FeedSubsystem getFeedSubsystem() {
        return this.feedSubsystem;
    }

    private VelocityMotorIO buildAgitatorIO() {
        if (currentMode == Mode.SIM) {
            return new SimVelocityMotorIO(
                Constants.Agitator.config,
                Constants.Agitator.INERTIA,
                Constants.Agitator.GEAR_RATIO,
                Constants.Agitator.SIM_MOTOR, 
                Constants.SIM_DELTA);
        }

        switch (Constants.AGITATOR_VELOCITY_MOTOR_TYPE) {
        case CTRE_TALON_FX:
            return new CtreTalonFxVelocityIO(Constants.Agitator.config);
        default:
            throw new IllegalStateException("Unsupported feed motor type");
        }
    }

    private VelocityMotorIO buildFeedIO() {
        if (currentMode == Mode.SIM) {
            return new SimVelocityMotorIO(
                Constants.Feed.config,
                Constants.Feed.INERTIA,
                Constants.Feed.GEAR_RATIO,
                Constants.Feed.SIM_MOTOR, 
                Constants.SIM_DELTA
            );
        }

        switch (Constants.FEED_VELOCITY_MOTOR_TYPE) {
            case CTRE_TALON_FX:
                return new CtreTalonFxVelocityIO(Constants.Feed.config);
            default:
                throw new IllegalStateException("Unsupported feed motor type");
        }
    }

    // FLYWHEEL

    public FlywheelSubsystem getFlywheelSubsystem()
    {
        return this.flywheelSubsystem;
    }

    private VelocityMotorIO buildFlywheelIO() {
        if (currentMode == Mode.SIM) {
            return new SimVelocityMotorIO(
                Constants.Flywheel.config,
                Constants.Flywheel.INERTIA,
                Constants.Flywheel.GEAR_RATIO,
                Constants.Flywheel.SIM_MOTOR, 
                Constants.SIM_DELTA
            );
        }

        switch (Constants.FLYWHEEL_VELOCITY_MOTOR_TYPE) {
            case CTRE_TALON_FX:
                return new CtreTalonFxVelocityIO(Constants.Flywheel.config);
            default:
                throw new IllegalStateException("Unsupported shooter motor type");
        }
    }

    // TURRET (AIM/HOOD)

    public TurretSubsystem getTurretSubsystem() {
        return turretSubsystem;
    }

    private PositionMotorIO buildTurretAimIO() {
        if (currentMode == Mode.SIM) {
            return new SimPositionMotorIO(
                Constants.Turret.Aim.config,
                Constants.Turret.Aim.INERTIA,
                Constants.Turret.Aim.GEAR_RATIO,
                Constants.Turret.Aim.ENCODER_RATIO, 
                Constants.Turret.Aim.SIM_MOTOR, 
                Constants.SIM_DELTA);
        }
        switch (Constants.TURRET_AIM_POSITION_MOTOR_TYPE) {
            case CTRE_TALON_FX:
                return new CtreTalonFxPositionIO(
                    Constants.Turret.Aim.config,
                    Constants.Turret.Aim.GEAR_RATIO,
                    Constants.Turret.Aim.ENCODER_RATIO);
            default:
                throw new IllegalStateException("Unsupported shooter motor type");
        }
    }

    private PositionMotorIO buildTurretHoodIO() {
        if (currentMode == Mode.SIM) {
            return new SimPositionMotorIO(
                Constants.Turret.Hood.config,
                Constants.Turret.Hood.INERTIA,
                Constants.Turret.Hood.GEAR_RATIO,
                Constants.Turret.Hood.ENCODER_RATIO, 
                Constants.Turret.Hood.SIM_MOTOR, 
                Constants.SIM_DELTA);
        }
        switch (Constants.TURRET_HOOD_POSITION_MOTOR_TYPE) {
            case CTRE_TALON_FX:
                return new CtreTalonFxPositionIO(
                    Constants.Turret.Hood.config,
                    Constants.Turret.Hood.GEAR_RATIO,
                    Constants.Turret.Hood.ENCODER_RATIO);
            default:
                throw new IllegalStateException("Unsupported shooter motor type");
        }
    }

    private AbsEncoderIO buildTurretEncoderIO() {
        if (currentMode == Mode.SIM) {
            return new SimAbsEncoderIO(0);
        }

        return new WpiDutyCycleEncoderIO(Constants.Turret.EncoderChannel);
    }

    // INTAKE

    public IntakeSubsystem getIntakeSubsystem() {
        return this.intakeSubsystem;
    }

    private AbsEncoderIO buildIntakeEncoderIO() {
        if (currentMode == Mode.SIM) {
        return new SimAbsEncoderIO(0);
        }

        return new CANCoderIO(Constants.Intake.EncoderChannel);
    }

    private PositionMotorIO buildIntakeDeployIO() {
        if (currentMode == Mode.SIM) {
            return new SimPositionMotorIO(
                Constants.Intake.Deploy.config,
                Constants.Intake.Deploy.INERTIA,
                Constants.Intake.Deploy.GEAR_RATIO,
                Constants.Intake.Deploy.ENCODER_RATIO, 
                Constants.Intake.Deploy.SIM_MOTOR, 
                Constants.SIM_DELTA);
        }
        switch (Constants.INTAKE_DEPLOY_POSITION_MOTOR_TYPE) {
        case CTRE_TALON_FX:
            return new CtreTalonFxPositionIO(
                Constants.Intake.Deploy.config,
                Constants.Intake.Deploy.GEAR_RATIO,
                Constants.Intake.Deploy.ENCODER_RATIO);
        default:
            throw new IllegalStateException("Unsupported intake deploy motor type");
        }
    }

    private VelocityMotorIO buildIntakeRollerIO() {
        if (currentMode == Mode.SIM) {
            return new SimVelocityMotorIO(
                Constants.Intake.Roller.config,
                Constants.Intake.Roller.INERTIA,
                Constants.Intake.Roller.GEAR_RATIO,
                Constants.Intake.Roller.SIM_MOTOR, 
                Constants.SIM_DELTA
            );
        }
        switch (Constants.INTAKE_ROLLER_VELOCITY_MOTOR_TYPE) {
        case CTRE_TALON_FX:
            return new CtreTalonFxVelocityIO(Constants.Intake.Roller.config);
        default:
            throw new IllegalStateException("Unsupported intake roller motor type");
        }
    }

    // DRIVE

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    private ModuleIO[] buildModuleIO() {
        switch (Constants.currentMode) {
        case SIM:
            return new ModuleIO[] {
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight)
            };
        case REAL:
            return new ModuleIO[] {
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight)
            };
        default:
            throw new IllegalStateException("Unsupported mode");
        }
    }

    private GyroIO buildGyroIO() {
        switch (Constants.currentMode) {
        case SIM:
            return new GyroIO() {};
        case REAL:
            return new GyroIOPigeon2();
        default:
            throw new IllegalStateException("Unsupported mode");
        }
    }

    // VISION

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    private VisionIO[] buildVisionIO() {
        switch (Constants.currentMode) {
        case SIM:
            // PhotonSim needs the drive pose for the camera positions
            return new VisionIO[] {
                new VisionIOPhotonVisionSim(
                    Constants.Vision.Left.name, 
                    Constants.Vision.Left.fromRobot, 
                    driveSubsystem::getPose
                ),
                new VisionIOPhotonVisionSim(
                    Constants.Vision.Right.name,
                    Constants.Vision.Right.fromRobot,
                    driveSubsystem::getPose
                )
            };
        case REAL:
            // Limelight needs gyro rotation for MegaTag2 
            return new VisionIO[] {
                new VisionIOLimelight(Constants.Vision.Left.name, driveSubsystem::getRotation),
                new VisionIOLimelight(Constants.Vision.Right.name, driveSubsystem::getRotation)
            };
        default:
            throw new IllegalStateException("Unsupported mode");
        }
    }
}

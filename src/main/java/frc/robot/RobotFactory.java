package frc.robot;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;
import frc.robot.subsystems.io.encoder.impl.SimAbsEncoderIO;
import frc.robot.subsystems.io.encoder.impl.WpiDutyCycleEncoderIO;
import frc.robot.subsystems.io.motor.CTRE.CtreTalonFxPositionIO;
import frc.robot.subsystems.io.motor.CTRE.CtreTalonFxVelocityIO;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.subsystems.io.motor.Sim.SimPositionMotorIO;
import frc.robot.subsystems.io.motor.Sim.SimVelocityMotorIO;
import frc.robot.subsystems.io.motor.VelocityMotorIO;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.VisionSubsystem;

public class RobotFactory {
private final ShooterSubsystem shooterSubsystem;
private final IntakeSubsystem intakeSubsystem;
private final DriveSubsystem driveSubsystem;
private final VisionSubsystem visionSubsystem;
private final AgitatorSubsystem agitatorSubsystem;
private final ClimberSubsystem climberSubsystem;

private final Mode currentMode;

public RobotFactory() {
    currentMode = Constants.currentMode;
    shooterSubsystem = new ShooterSubsystem(buildShooterMotorIO());
    driveSubsystem = new DriveSubsystem(buildGyroIO(), buildModuleIO());
    visionSubsystem = new VisionSubsystem(driveSubsystem::addVisionMeasurement, buildVisionIO());
    intakeSubsystem = new IntakeSubsystem(
        buildIntakeEncoderIO(), buildIntakeDeployIO(), buildIntakeRollerIO()
    );
    agitatorSubsystem = new AgitatorSubsystem(buildAgitatorIO());
    climberSubsystem = new ClimberSubsystem(buildClimberEncoderIO(), buildClimberMotorIO());
}

// CLIMBER

public ClimberSubsystem getClimberSubsystem() {
    return this.climberSubsystem;
}

private AbsEncoderIO buildClimberEncoderIO() {
    if (currentMode == Mode.SIM) {
        return new SimAbsEncoderIO(0);
    }

    return new WpiDutyCycleEncoderIO(new DutyCycleEncoder(Constants.Climber.EncoderChannel));
}

private PositionMotorIO buildClimberMotorIO() {
    if (currentMode == Mode.SIM) {
        return new SimPositionMotorIO(
            Constants.Climber.Config,
            Constants.Climber.INERTIA,
            Constants.Climber.GEAR_RATIO,
            Constants.Climber.ENCODER_RATIO, 
            Constants.Climber.SIM_MOTOR, 
            Constants.SIM_DELTA);
    }
    switch (Constants.CLIMBER_POSITION_MOTOR_TYPE) {
        case CTRE_TALON_FX:
            return new CtreTalonFxPositionIO(
                Constants.Climber.Config,
                Constants.Climber.GEAR_RATIO,
                Constants.Climber.ENCODER_RATIO);
        default:
            throw new IllegalStateException("Unsupported intake deploy motor type");
    }
}

// AGITATOR

private VelocityMotorIO buildAgitatorIO() {
    if (currentMode == Mode.SIM) {
        return new SimVelocityMotorIO(
            Constants.Agitator.Config,
            Constants.Agitator.INERTIA,
            Constants.Agitator.GEAR_RATIO,
            Constants.Agitator.SIM_MOTOR, 
            Constants.SIM_DELTA);
    }

    switch (Constants.AGITATOR_VELOCITY_MOTOR_TYPE) {
    case CTRE_TALON_FX:
        return new CtreTalonFxVelocityIO(Constants.Agitator.Config);
    default:
        throw new IllegalStateException("Unsupported agitator motor type");
    }
}

public AgitatorSubsystem getAgitatorSubsystem() {
    return this.agitatorSubsystem;
}

// SHOOTER

public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
}

private VelocityMotorIO buildShooterMotorIO() {
    if (currentMode == Mode.SIM) {
        return new SimVelocityMotorIO(
            Constants.Shooter.Config,
            Constants.Shooter.INERTIA,
            Constants.Shooter.GEAR_RATIO,
            Constants.Shooter.SIM_MOTOR, 
            Constants.SIM_DELTA);
    }

    switch (Constants.SHOOTER_VELOCITY_MOTOR_TYPE) {
    case CTRE_TALON_FX:
        return new CtreTalonFxVelocityIO(Constants.Shooter.Config);
    default:
        throw new IllegalStateException("Unsupported shooter motor type");
    }
}

// INTAKE

public IntakeSubsystem getIntakeSubsystem() {
    return this.intakeSubsystem;
}

private AbsEncoderIO buildIntakeEncoderIO() {
    if (currentMode == Mode.SIM) {
    return new SimAbsEncoderIO(0);
    }

    return new WpiDutyCycleEncoderIO(new DutyCycleEncoder(Constants.Intake.EncoderChannel));
}

private PositionMotorIO buildIntakeDeployIO() {
    if (currentMode == Mode.SIM) {
        return new SimPositionMotorIO(
            Constants.Intake.DeployConfig,
            Constants.Intake.Deploy_INERTIA,
            Constants.Intake.Deploy_GEAR_RATIO,
            Constants.Intake.Deploy_ENCODER_RATIO, 
            Constants.Intake.Deploy_SIM_MOTOR, 
            Constants.SIM_DELTA);
    }
    switch (Constants.INTAKE_DEPLOY_POSITION_MOTOR_TYPE) {
    case CTRE_TALON_FX:
        return new CtreTalonFxPositionIO(
            Constants.Intake.DeployConfig,
            Constants.Intake.Deploy_GEAR_RATIO,
            Constants.Intake.Deploy_ENCODER_RATIO);
    default:
        throw new IllegalStateException("Unsupported intake deploy motor type");
    }
}

private VelocityMotorIO buildIntakeRollerIO() {
    if (currentMode == Mode.SIM) {
        return new SimVelocityMotorIO(
            Constants.Intake.RollerConfig,
            Constants.Intake.Roller_INERTIA,
            Constants.Intake.Roller_GEAR_RATIO,
            Constants.Intake.Roller_SIM_MOTOR, 
            Constants.SIM_DELTA
        );
    }
    switch (Constants.INTAKE_ROLLER_VELOCITY_MOTOR_TYPE) {
    case CTRE_TALON_FX:
        return new CtreTalonFxVelocityIO(Constants.Intake.RollerConfig);
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
        return new VisionIO[] {
        new VisionIOPhotonVisionSim(
            Constants.Vision.Left.name, Constants.Vision.Left.fromRobot, driveSubsystem::getPose),
        new VisionIOPhotonVisionSim(
            Constants.Vision.Right.name,
            Constants.Vision.Right.fromRobot,
            driveSubsystem::getPose)
        };
    case REAL:
        return new VisionIO[] {
        new VisionIOLimelight(Constants.Vision.Left.name, driveSubsystem::getRotation),
        new VisionIOLimelight(Constants.Vision.Right.name, driveSubsystem::getRotation)
        };
    default:
        throw new IllegalStateException("Unsupported mode");
    }
}
}

package frc.robot;

import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.io.motor.CTRE.CtreTalonFxVelocityIO;
import frc.robot.subsystems.io.motor.Sim.SimVelocityMotorIO;
import frc.robot.subsystems.io.motor.VelocityMotorIO;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotFactory {
  private final ShooterSubsystem shooterSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final Mode currentMode;

  public RobotFactory() {
    currentMode = Constants.currentMode;
    shooterSubsystem = new ShooterSubsystem(buildShooterMotorIO());
    driveSubsystem = new DriveSubsystem(buildGyroIO(), buildModuleIO());
    visionSubsystem = new VisionSubsystem(driveSubsystem::addVisionMeasurement, buildVisionIO());
  }

  public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
  }

  private VelocityMotorIO buildShooterMotorIO() {
    if (currentMode == Mode.SIM) {
      return new SimVelocityMotorIO();
    }

    switch (Constants.SHOOTER_VELOCITY_MOTOR_TYPE) {
      case CTRE_TALON_FX:
        return new CtreTalonFxVelocityIO(
            Constants.shooterTalonFxVelocityConfig(),
            Constants.Shooter.MOTOR_ID,
            Constants.Shooter.FOLLOWER_ID);
      default:
        throw new IllegalStateException("Unsupported shooter motor type");
    }
  }

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

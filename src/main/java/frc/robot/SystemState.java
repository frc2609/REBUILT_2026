package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ProjectileSimulator;
import frc.robot.util.ShotCalculator;

public class SystemState {

    public static boolean 
        validShotDetected = false,
        flywheelAtSpeed = false,
        turretInPosition = false,
        isPassing = false,
        hubActive = false,
        trenchBlocked = false,
        hasVisionTarget = false;

    private static DriveSubsystem swerve;
    private static TurretSubsystem turret;

    // Mechanism Outputs

    public static double turretAngleDeg;
    public static double turretSOTMFF;

    public static double calculatedFlywheelRPM;


    public SystemState(DriveSubsystem swerve, TurretSubsystem turret) {
        SystemState.swerve = swerve;
        SystemState.turret = turret;
    }

    public void updateSOTMState() {
        Pose2d robotPose = swerve.getPose();
        
        // inverted on purpose, used for physical display and distance calc
        // does not match x and y used in SOTM docs
        Pose2d turretPose = robotPose.plus(new Transform2d(
            Constants.shotConfig.launcherOffsetY, 
            Constants.shotConfig.launcherOffsetX, 
            robotPose.getRotation()));

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (Constants.currentMode == Mode.SIM) { alliance = Alliance.Blue; }
        Translation2d target, targetForward;

        boolean isLeft = robotPose.getY() > Constants.Field.CENTER_Y;
        SystemState.trenchBlocked = false;

        if (alliance == Alliance.Blue) {
            if (robotPose.getX() <= Constants.Field.BLUE_BLOCK_X) {
                SystemState.isPassing = false;
                target = Constants.Field.BLUE_HUB;
                targetForward = new Translation2d(1,0);
                if (robotPose.getX() > Constants.Field.BLUE_ZONE_X) {
                    SystemState.trenchBlocked = true;
                } else {
                    SystemState.trenchBlocked = false;
                }
            } else {
                SystemState.isPassing = true;
                if (robotPose.getX() <= Constants.Field.BLUE_CLOSE_ZONE_X) {
                    target = isLeft ? Constants.Field.BLUE_CLOSE_PASS_LEFT : Constants.Field.BLUE_CLOSE_PASS_RIGHT;
                    targetForward = new Translation2d(-1,0);
                } else {
                    target = isLeft ? Constants.Field.BLUE_PASS_LEFT : Constants.Field.BLUE_PASS_RIGHT;
                    targetForward = new Translation2d(-1,0);
                }
            }
        } else {
            if (robotPose.getX() >= Constants.Field.RED_BLOCK_X) {
                SystemState.isPassing = false;
                target = Constants.Field.RED_HUB;
                targetForward = new Translation2d(-1,0);
                if (robotPose.getX() < Constants.Field.RED_ZONE_X) {
                    SystemState.trenchBlocked = true;
                } else {
                    SystemState.trenchBlocked = false;
                }
            } else {
                SystemState.isPassing = true;
                if (robotPose.getX() >= Constants.Field.RED_CLOSE_ZONE_X) {
                    target = isLeft ? Constants.Field.RED_CLOSE_PASS_LEFT : Constants.Field.RED_CLOSE_PASS_RIGHT;
                    targetForward = new Translation2d(1,0);
                } else {
                    target = isLeft ? Constants.Field.RED_PASS_LEFT : Constants.Field.RED_PASS_RIGHT;
                    targetForward = new Translation2d(1,0);
                }
            }
        }

        ChassisSpeeds fieldRelativeSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(
            swerve.getChassisSpeeds(),
            swerve.getRotation()
        );

        // do not count trim when calculating heading error
        Rotation2d trimlessTurretAim = Rotation2d.fromDegrees(turret.getAimPosition())
                .plus(Rotation2d.fromDegrees(headingOffset.getAsDouble()))
                .plus(swerve.getRotation());

        // do use it when displaying turret
        Rotation2d fieldTurretAim = Rotation2d.fromDegrees(turret.getAimPosition())
                .plus(Rotation2d.fromDegrees(180.0))
                .plus(swerve.getRotation());

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            robotPose,
            fieldRelativeSpeed,
            swerve.getChassisSpeeds(),
            target, 
            targetForward,
            0.9, // vision confidence, 0 to 1
            trimlessTurretAim.getRadians()
        );

        ShotCalculator.LaunchParameters shot = shotCalc.calculate(inputs);
        double targetDist = turretPose.getTranslation().getDistance(target);

        double turretAngleDeg = shot.driveAngle()
            //.minus(Rotation2d.fromDegrees(Constants.Turret.Aim.HEADING_OFFSET_DEG))
            .minus(Rotation2d.fromDegrees(headingOffset.getAsDouble()))
            .minus(swerve.getRotation())
            .getDegrees();
        turretPose = new Pose2d(turretPose.getTranslation(), fieldTurretAim);

        SystemState.validShotDetected = 
            shot.isValid() && 
            shot.confidence() > Constants.Controls.SHOT_CONFIDENCE_MIN;
    }

    public void updateHubActive() {

    }

    public void updateSOTMSim() {
        if (Constants.currentMode == Constants.Mode.SIM) {
            double ballSpeed = ProjectileSimulator.rpmToExitVelocity(
                shot.rpm(), 
                Constants.simParameters.wheelDiameterM(),
                Constants.simParameters.slipFactor()
            );
            Translation3d launchVector = new Translation3d(ballSpeed*1.4, new Rotation3d(
                0.0, 
                Constants.simParameters.fixedLaunchAngleDeg()*(Math.PI/180.0), 
                turretPose.getRotation().getRadians()
            ));
            Translation3d ballVel = new Translation3d(
                fieldRelativeSpeed.vxMetersPerSecond, 
                fieldRelativeSpeed.vyMetersPerSecond, 
                0
            ).plus(launchVector);

            flywheel.launchPosSim = new Translation3d(
                turretPose.getTranslation().getX(),
                turretPose.getTranslation().getY(),
                Constants.simParameters.exitHeightM()
            );
            flywheel.launchSpeedSim = ballVel;
        }
    }

    public void logSystemState() {
        Logger.recordOutput("SOTM/TurretPose", turretPose);
        Logger.recordOutput("SOTM/TurretSetpoint", turretAngleDeg);
        Logger.recordOutput("SOTM/Target", target);
        Logger.recordOutput("SOTM/TargetDistance", targetDist);
        Logger.recordOutput("SOTM/Flags/ValidShot", validShot);
        Logger.recordOutput("SOTM/Flags/HubActive", isHubActive());
        Logger.recordOutput("SOTM/Flags/TurretInRange", turretInLimits);
        Logger.recordOutput("SOTM/Flags/Confidence", shot.confidence());
        Logger.recordOutput("SOTM/Flags/TrenchBlock", disableShoot);
        Logger.recordOutput("SOTM/Flags/Passing", passing);
    }
}

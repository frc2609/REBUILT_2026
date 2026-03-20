package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ShotCalculator;

public class AutoAimTurret extends Command {
    private final TurretSubsystem turret;
    private final DriveSubsystem swerve;
    private final FlywheelSubsystem flywheel;
    private final ShotCalculator shotCalc;
    private final LoggedNetworkNumber kVTarget, headingOffset; 

    private boolean disableShoot = false;
    private boolean turretInLimits = false;
    private boolean validShot = false;

    public AutoAimTurret(
        DriveSubsystem swerve, TurretSubsystem turret,
        FlywheelSubsystem flywheel, ShotCalculator shotCalc
    ) {
        this.turret = turret;
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.shotCalc = shotCalc;

        kVTarget = new LoggedNetworkNumber("turretAimkV", -0.7);
        headingOffset = new LoggedNetworkNumber("headingOffset",188.0);

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();
        Pose2d turretPose = robotPose.plus(new Transform2d(-0.144, -0.177, robotPose.getRotation()));

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (Constants.currentMode == Mode.SIM) { alliance = Alliance.Blue; }
        Translation2d target, targetForward;

        boolean isLeft = robotPose.getY() > Constants.Field.CENTER_Y;
        disableShoot = false;

        if (alliance == Alliance.Blue) {
            if (robotPose.getX() <= Constants.Field.BLUE_BLOCK_X) {
                target = Constants.Field.BLUE_HUB;
                targetForward = new Translation2d(1,0);
                if (robotPose.getX() > Constants.Field.BLUE_ZONE_X) {
                    disableShoot = true;
                }
            } else if (robotPose.getX() <= Constants.Field.BLUE_CLOSE_ZONE_X) {
                target = isLeft ? Constants.Field.BLUE_CLOSE_PASS_LEFT : Constants.Field.BLUE_CLOSE_PASS_RIGHT;
                targetForward = new Translation2d(-1,0);
            } else {
                target = isLeft ? Constants.Field.BLUE_PASS_LEFT : Constants.Field.BLUE_PASS_RIGHT;
                targetForward = new Translation2d(-1,0);
            }
        } else {
            if (robotPose.getX() >= Constants.Field.RED_BLOCK_X) {
                target = Constants.Field.RED_HUB;
                targetForward = new Translation2d(-1,0);
                if (robotPose.getX() > Constants.Field.RED_ZONE_X) {
                    disableShoot = true;
                }
            } else if (robotPose.getX() >= Constants.Field.RED_CLOSE_ZONE_X) {
                target = isLeft ? Constants.Field.RED_CLOSE_PASS_LEFT : Constants.Field.RED_CLOSE_PASS_RIGHT;
                targetForward = new Translation2d(1,0);
            } else {
                target = isLeft ? Constants.Field.RED_PASS_LEFT : Constants.Field.RED_PASS_RIGHT;
                targetForward = new Translation2d(1,0);
            }
        }

        ChassisSpeeds fieldRelativeSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(
            swerve.getChassisSpeeds(),
            swerve.getRotation()
        );

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            turretPose,
            fieldRelativeSpeed,
            swerve.getChassisSpeeds(),
            target, targetForward,
            0.9 // vision confidence, 0 to 1
        );

        ShotCalculator.LaunchParameters shot = shotCalc.calculate(inputs);
        double targetDist = turretPose.getTranslation().getDistance(target);
        double turretAngleDeg = shot.launcherAngle()
            //.minus(Rotation2d.fromDegrees(Constants.Turret.Aim.HEADING_OFFSET_DEG))
            .minus(Rotation2d.fromDegrees(headingOffset.get()))
            .minus(swerve.getRotation())
            .getDegrees();

        turretInLimits = Math.abs(turretAngleDeg) <= Constants.Turret.Aim.RANGE_DEG;
        validShot = shot.isValid(); 

        if (turretInLimits) {
            turret.setAimPositionFF(
                turretAngleDeg, 
                kVTarget.get()*shot.driveAngularVelocityRadPerSec()
            );
        } else {
            turret.setAimPosition(0.0);
        }

        if (validShot && turretInLimits && !disableShoot) {
            // Set hood and flywheel target based on shot 
            if (targetDist <= Constants.Controls.LOB_DISTANCE) {
                turret.setHoodPosition(0.0);
                flywheel.setAutoSpeed(Constants.Controls.FLYWHEEL_LOB_RPM/60.0);
            } else {
                turret.setHoodPosition(Constants.Controls.TURRET_HOOD_DEG);
                flywheel.setAutoSpeed(shot.rpm()/60.0);
            }
        } else {
            flywheel.setAutoSpeed(0.0);
        }

        if (Constants.currentMode == Constants.Mode.SIM) {
            double ballSpeed = 1.25*(shot.rpm()/60.0)*Math.PI*Constants.simParameters.wheelDiameterM();
            Translation3d launchVector = new Translation3d(ballSpeed, new Rotation3d(
                0.0, 
                Constants.simParameters.fixedLaunchAngleDeg()*(Math.PI/180.0), 
                shot.launcherAngle().getRadians()
            ));
            Translation3d ballVel = new Translation3d(
                fieldRelativeSpeed.vxMetersPerSecond, 
                fieldRelativeSpeed.vyMetersPerSecond, 
                0
            ).plus(launchVector);

            flywheel.launchPosSim = new Translation3d(
                turretPose.getTranslation().getX(),
                turretPose.getTranslation().getY(),
                0.1
            );
            flywheel.launchSpeedSim = ballVel;
        }

        turretPose = new Pose2d(turretPose.getTranslation(), shot.launcherAngle());
        Logger.recordOutput("TurretPose", turretPose);
        Logger.recordOutput("TurretSetpoint", turretAngleDeg);
        Logger.recordOutput("Target", target);
        Logger.recordOutput("TargetDistance", targetDist);
        Logger.recordOutput("ValidShot", validShot);
        Logger.recordOutput("HubActive", isHubActive());
    }

    public boolean shouldRumble() {
        return validShot && turretInLimits && !disableShoot;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}


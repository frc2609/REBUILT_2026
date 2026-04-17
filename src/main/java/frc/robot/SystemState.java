package frc.robot;

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
import frc.robot.Constants.Mode;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ProjectileSimulator;
import frc.robot.util.ShotCalculator;

public class SystemState {

    public static boolean 
        validShotDetected = false,
        flywheelAtSpeed = false,
        isPassing = false,
        hubActive = false,
        trenchBlocked = false,
        hasVisionTarget = false;

    private static DriveSubsystem swerve;
    private static TurretSubsystem turret;
    private static ShotCalculator shotCalc;
    private static LoggedNetworkNumber headingOffset, adjustableAngle; 

    private static Pose2d turretPose;
    private static Translation2d target;
    private static double targetDist;
    
    // Mechanism/Sim Outputs
    
    public static double turretAngleDeg;
    public static double turretSOTMFF;
    public static double hoodAngleDeg;
    public static double calculatedFlywheelRPM;
    public static ShotCalculator.LaunchParameters shot;
    public static Translation3d launchPosSim, launchVelSim; 


    public SystemState(
        DriveSubsystem swerve, TurretSubsystem turret,
        ShotCalculator shotCalc, LoggedNetworkNumber headingOffset
    ) {
        SystemState.swerve = swerve;
        SystemState.turret = turret;
        SystemState.shotCalc = shotCalc;
        SystemState.headingOffset = headingOffset;

        SystemState.adjustableAngle = new LoggedNetworkNumber("SOTM/HoodAngle/", 15.0);
    }

    public void updateSOTMState() {
        Pose2d robotPose = swerve.getPose();
        
        // inverted on purpose, used for physical display and distance calc
        // does not match x and y used in SOTM docs
        SystemState.turretPose = robotPose.plus(new Transform2d(
            Constants.shotConfig.launcherOffsetY, 
            Constants.shotConfig.launcherOffsetX, 
            robotPose.getRotation()));

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (Constants.currentMode == Mode.SIM) { alliance = Alliance.Blue; }
        Translation2d targetForward;

        boolean isLeft = robotPose.getY() > Constants.Field.CENTER_Y;
        SystemState.trenchBlocked = false;

        if (alliance == Alliance.Blue) {
            if (robotPose.getX() <= Constants.Field.BLUE_BLOCK_X) {
                SystemState.isPassing = false;
                target = Constants.Field.BLUE_HUB;
                targetForward = new Translation2d(1,0);
                if (robotPose.getX() > Constants.Field.BLUE_ZONE_X ||
                    turretPose.getX() > Constants.Field.BLUE_ZONE_X-0.2) {
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
                if (robotPose.getX() < Constants.Field.RED_ZONE_X ||
                    turretPose.getX() < Constants.Field.RED_ZONE_X + 0.2) {
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
                .plus(Rotation2d.fromDegrees(Constants.Turret.Aim.HEADING_OFFSET_DEG))
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

        shot = shotCalc.calculate(inputs);
        SystemState.turretAngleDeg = shot.driveAngle()
            //.minus(Rotation2d.fromDegrees(Constants.Turret.Aim.HEADING_OFFSET_DEG))
            .minus(Rotation2d.fromDegrees(headingOffset.getAsDouble()))
            .minus(swerve.getRotation())
            .getDegrees();
        turretPose = new Pose2d(turretPose.getTranslation(), fieldTurretAim);

        SystemState.validShotDetected = 
            shot.isValid() && 
            shot.confidence() > Constants.Controls.SHOT_CONFIDENCE_MIN;
        
        SystemState.targetDist = turretPose.getTranslation().getDistance(target);

        if (SystemState.validShotDetected && 
            !SystemState.trenchBlocked
        ) {
            // Set hood and flywheel target based on shot 
            if (targetDist <= Constants.Controls.LOB_DISTANCE) {
                SystemState.hoodAngleDeg = 0.0;
                SystemState.calculatedFlywheelRPM = Constants.Controls.FLYWHEEL_LOB_RPM;
            } else {
                SystemState.hoodAngleDeg = adjustableAngle.get();
                SystemState.calculatedFlywheelRPM = shot.rpm();
            }
        } else {
            SystemState.calculatedFlywheelRPM = 0.0;
        }
    }

    public void updateHubActive() {
        hubActive = isHubActive();
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

    public void updateSOTMSim() {
        ChassisSpeeds fieldRelativeSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(
            swerve.getChassisSpeeds(),
            swerve.getRotation()
        );


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

            SystemState.launchPosSim = new Translation3d(
                turretPose.getTranslation().getX(),
                turretPose.getTranslation().getY(),
                Constants.simParameters.exitHeightM()
            );
            SystemState.launchVelSim = new Translation3d(
                fieldRelativeSpeed.vxMetersPerSecond, 
                fieldRelativeSpeed.vyMetersPerSecond, 
                0
            ).plus(launchVector);
        }
    }

    public void logSystemState() {
        Logger.recordOutput("SOTM/FlywheelSetpoint", SystemState.calculatedFlywheelRPM);
        Logger.recordOutput("SOTM/TurretPose", SystemState.turretPose);
        Logger.recordOutput("SOTM/TurretSetpoint", SystemState.turretAngleDeg);
        Logger.recordOutput("SOTM/Target", SystemState.target);
        Logger.recordOutput("SOTM/TargetDistance", SystemState.targetDist);
        Logger.recordOutput("SOTM/Flags/ValidShot", SystemState.validShotDetected);
        Logger.recordOutput("SOTM/Flags/HubActive", SystemState.hubActive);
        Logger.recordOutput("SOTM/Flags/Confidence", SystemState.shot.confidence());
        Logger.recordOutput("SOTM/Flags/TrenchBlock", SystemState.trenchBlocked);
        Logger.recordOutput("SOTM/Flags/Passing", SystemState.isPassing);
    }
}

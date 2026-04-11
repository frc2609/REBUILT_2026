package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class Targeting {
    public double 
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
        disableShoot = false;

        if (alliance == Alliance.Blue) {
            if (robotPose.getX() <= Constants.Field.BLUE_BLOCK_X) {
                passing = false;
                target = Constants.Field.BLUE_HUB;
                targetForward = new Translation2d(1,0);
                if (robotPose.getX() > Constants.Field.BLUE_ZONE_X) {
                    disableShoot = true;
                }
            } else {
                passing = true;
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
                passing = false;
                target = Constants.Field.RED_HUB;
                targetForward = new Translation2d(-1,0);
                if (robotPose.getX() < Constants.Field.RED_ZONE_X) {
                    disableShoot = true;
                }
            } else {
                passing = true;
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
            swerve.getPose(),
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
            .minus(Rotation2d.fromDegrees(headingOffset.get()))
            .minus(swerve.getRotation())
            .getDegrees();
        turretPose = new Pose2d(turretPose.getTranslation(), fieldTurretAim);
        validShot = shot.isValid() && shot.confidence() > Constants.Controls.SHOT_CONFIDENCE_MIN;

}

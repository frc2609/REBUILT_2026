package frc.robot.commands;

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

public class AimTurretField extends Command {
    private final TurretSubsystem turret;
    private final DriveSubsystem swerve;
    private final FlywheelSubsystem flywheel;
    private final ShotCalculator shotCalc;
    private final LoggedNetworkNumber power; 
    private final LoggedNetworkNumber kVTarget, headingOffset;

    private boolean inScoringZone = false;
    private boolean turretInLimits = false;

    public AimTurretField(
        DriveSubsystem swerve, TurretSubsystem turret,
        FlywheelSubsystem flywheel, ShotCalculator shotCalc
    ) {
        this.turret = turret;
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.shotCalc = shotCalc;

        // magic number
        power = new LoggedNetworkNumber("SimPower", .85);
        kVTarget = new LoggedNetworkNumber("turretAimkV", -0.7);
        headingOffset = new LoggedNetworkNumber("headingOffset",170.0);

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();
        Pose2d turretPose = robotPose.plus(new Transform2d(-0.144, -0.177, robotPose.getRotation()));

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d target, targetForward;

        boolean isLeft = robotPose.getY() > Constants.Field.CENTER_Y;

        if (alliance == Alliance.Blue)
        {
            if (robotPose.getX() <= Constants.Field.BLUE_ZONE_X) {
                inScoringZone = true;
                target = Constants.Field.BLUE_HUB;
                targetForward = new Translation2d(1,0);
            } else if (robotPose.getX() <= Constants.Field.BLUE_CLOSE_ZONE_X) {
                inScoringZone = false;
                target = isLeft ? Constants.Field.BLUE_CLOSE_PASS_LEFT : Constants.Field.BLUE_CLOSE_PASS_RIGHT;
                targetForward = new Translation2d(-1,0);
            } else {
                inScoringZone = false;
                target = isLeft ? Constants.Field.BLUE_PASS_LEFT : Constants.Field.BLUE_PASS_RIGHT;
                targetForward = new Translation2d(-1,0);
            }
        } else {
            if (robotPose.getX() >= Constants.Field.RED_ZONE_X) {
                inScoringZone = true;
                target = Constants.Field.RED_HUB;
                targetForward = new Translation2d(-1,0);
            } else if (robotPose.getX() >= Constants.Field.RED_CLOSE_ZONE_X) {
                inScoringZone = false;
                target = isLeft ? Constants.Field.RED_CLOSE_PASS_LEFT : Constants.Field.RED_CLOSE_PASS_RIGHT;
                targetForward = new Translation2d(1,0);
            } else {
                inScoringZone = false;
                target = isLeft ? Constants.Field.RED_PASS_LEFT : Constants.Field.RED_PASS_RIGHT;
                targetForward = new Translation2d(1,0);
            }
        }

        Logger.recordOutput("TARGET", target);

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

        // Set turret aim independant of shot
        
        double turretAngleDeg = shot.launcherAngle()
            //.minus(Rotation2d.fromDegrees(Constants.Turret.Aim.HEADING_OFFSET_DEG))
            .minus(Rotation2d.fromDegrees(headingOffset.get()))
            .minus(Rotation2d.fromDegrees(shotCalc.getAimOffset()))
            .minus(swerve.getRotation())
            .getDegrees();

        turretInLimits = Math.abs(turretAngleDeg) <= Constants.Turret.Aim.RANGE_DEG;
        if (turretInLimits) {
            turret.setAimPositionFF(
                turretAngleDeg, 
                kVTarget.get()*shot.driveAngularVelocityRadPerSec()
            );
        }

        // Set hood and flywheel target based on shot 

        double targetDist = turretPose.getTranslation().getDistance(target);
        Logger.recordOutput("TargetDistance", targetDist);

        if (shot.isValid()) {
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
            double ballSpeed = power.get()*(shot.rpm()/60.0)*Math.PI*Constants.simParameters.ballDiameterM();
            Translation3d launchVector = new Translation3d(ballSpeed, new Rotation3d(
                0.0, 
                (80.0-0.6*turret.getHoodPosition())*(Math.PI/180.0), 
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
        Logger.recordOutput("TurretAngle", turretAngleDeg);
    }

    public boolean isReadyToScore() {
        return inScoringZone && turretInLimits;
    }

    @Override
    public void end(boolean interrupted) {
        inScoringZone = false;
        turretInLimits = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

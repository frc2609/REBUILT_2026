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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FuelPhysicsSim;
import frc.robot.util.ShotCalculator;

public class AimTurretField extends Command {
    private final TurretSubsystem turret;
    private final DriveSubsystem swerve;
    private final FlywheelSubsystem flywheel;
    private final Translation2d target;
    private final ShotCalculator shotCalc;
    private final Trigger shootTrigger;
    private final LoggedNetworkNumber power; 
    private final FuelPhysicsSim ballSim;
    private double i = 0;

    public AimTurretField(
        DriveSubsystem swerve, TurretSubsystem turret,
        FlywheelSubsystem flywheel,
        Translation2d target, ShotCalculator shotCalc,
        Trigger shootTrigger, FuelPhysicsSim ballSim
    ) {
        this.turret = turret;
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.target = target;
        this.shotCalc = shotCalc;
        this.shootTrigger = shootTrigger;
        this.ballSim = ballSim;

        // magic number
        power = new LoggedNetworkNumber("shotPower", 0.67);

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();
        Pose2d turretPose = robotPose.plus(new Transform2d(-0.144, -0.177, robotPose.getRotation()));

        // Translation2d toHub = target.minus(turretPose.getTranslation());
        // Rotation2d angleToHub = toHub.getAngle();

        Translation2d hubCenter = this.target;
        Translation2d hubForward = new Translation2d(1, 0);       // which way the hub faces
        ChassisSpeeds fieldRelativeSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(
            swerve.getChassisSpeeds(),
            swerve.getRotation()
        );

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            turretPose,
            fieldRelativeSpeed,
            swerve.getChassisSpeeds(),
            hubCenter, hubForward,
            0.9 // vision confidence, 0 to 1
        );

        ShotCalculator.LaunchParameters shot = shotCalc.calculate(inputs);
        Translation3d launchVector = new Translation3d(
            (shot.rpm()*power.get())*Math.PI*0.1016, 
            new Rotation3d(0.0, (75.0/180)*Math.PI, 
                shot.launcherAngle().getRadians()
        ));
        Translation3d ballVel = new Translation3d(
            fieldRelativeSpeed.vxMetersPerSecond, 
            fieldRelativeSpeed.vyMetersPerSecond, 
            0
        ).plus(launchVector);

        turret.setHoodPosition();
        flywheel.setAutoSpeed(power.get()*shot.rpm()/60.0);
        double turretAngleDeg = shot.launcherAngle()
            .minus(new Rotation2d().fromDegrees(-145.0))
            .minus(swerve.getRotation())
            .getDegrees();

        if (Math.abs(turretAngleDeg) <= Constants.Turret.Aim.RANGE_DEG) {
            turret.setAimPosition(turretAngleDeg);
        }

        if (Constants.currentMode == Constants.Mode.SIM && shootTrigger.getAsBoolean()) {

            if (shootTrigger.getAsBoolean()) {
                if (i % 3 == 0) {
                    ballSim.launchBall(
                        new Translation3d(
                            turretPose.getTranslation().getX(),
                            turretPose.getTranslation().getY(),
                            0.1
                        ), ballVel, 0.0);
                }
                i++;
            }
            // shot.driveAngularVelocityRadPerSec() gives you a heading feedforward if you want it
        }

        Logger.recordOutput("TurretPose", turretPose);
        Logger.recordOutput("LaunchX", new Translation3d(turretPose.getX()+ballVel.getX(), turretPose.getY(), 0.0));
        Logger.recordOutput("LaunchY", new Translation3d(turretPose.getX(), turretPose.getY()+ballVel.getY(), 0.0));
        Logger.recordOutput("TurretAngle", new Rotation2d().fromDegrees(turretAngleDeg));
        //Logger.recordOutput("TurretAngle", new Rotation2d().fromDegrees(turretAngleDeg).plus(new Rotation2d().fromDegrees(180.0)).getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

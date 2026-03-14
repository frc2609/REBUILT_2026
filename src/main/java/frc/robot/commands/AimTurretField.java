package frc.robot.commands;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ShotCalculator;

public class AimTurretField extends Command {
    private final TurretSubsystem turret;
    private final DriveSubsystem swerve;
    private final Translation2d target;
    private final ShotCalculator shotCalc;
    private final Trigger shootTrigger;
    private final LoggedNetworkNumber power; 
    private double i = 0;

    public AimTurretField(
        DriveSubsystem swerve, TurretSubsystem turret,
        Translation2d target, ShotCalculator shotCalc,
        Trigger shootTrigger
    ) {
        this.turret = turret;
        this.swerve = swerve;
        this.target = target;
        this.shotCalc = shotCalc;
        this.shootTrigger = shootTrigger;

        power = new LoggedNetworkNumber("shotPower", 0.0135);

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();
        Pose2d turretPose = robotPose.plus(new Transform2d(-0.2, -0.2, robotPose.getRotation()));

        // Translation2d toHub = target.minus(turretPose.getTranslation());
        // Rotation2d angleToHub = toHub.getAngle();

        Translation2d hubCenter = this.target;
        Translation2d hubForward = new Translation2d(1, 0);       // which way the hub faces
        ChassisSpeeds fieldRelativeSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(
            swerve.getChassisSpeeds(),
            swerve.getRotation()
        );

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            turretPose,//robotPose.plus(new Transform2d(-0.2, -0.2, robotPose.getRotation())),
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

        if (shot.isValid()){// && shot.confidence() > 50) {
            //shooter.setRPM(shot.rpm());
            //turret.setAimPosition(shot.driveAngle().getDegrees());
            //if (Math.abs(shot.driveAngle().getDegrees()) <= 110.0) {

            if (shootTrigger.getAsBoolean()) {
                if (i % 3 == 0) {
                    turret.ballSim.launchBall(
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
        Logger.recordOutput("TurretAngle", shot.launcherAngle());
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

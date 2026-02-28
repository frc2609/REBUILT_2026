package frc.robot.commands;
import static frc.robot.Constants.Vision.angularStdDevBaseline;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurretField extends Command {
    private final TurretSubsystem turret;
    private final Supplier<Pose2d> poseSupplier;
    private final Translation2d target;

    public AimTurretField(
        Supplier<Pose2d> poseSupplier, TurretSubsystem turret,
        Translation2d target
    ) {
        this.turret = turret;
        this.poseSupplier = poseSupplier;
        this.target = target;
    }

    @Override
    public void execute() {
        Pose2d robotPose = poseSupplier.get();
        Translation2d toHub = target.minus(robotPose.getTranslation());
        Rotation2d angleToHub = toHub.getAngle();
        Logger.recordOutput("AngleToHub", angleToHub);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

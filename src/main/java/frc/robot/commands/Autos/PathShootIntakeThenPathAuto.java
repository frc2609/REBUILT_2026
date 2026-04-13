package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FuelPhysicsSim;

/**
 * Runs {@link PathShootIntakeAuto} on {@code shootIntakePathName}, then follows
 * {@code secondPathName} with drive only (no shoot / intake).
 */
public class PathShootIntakeThenPathAuto extends SequentialCommandGroup {

    public PathShootIntakeThenPathAuto(
            String shootIntakePathName,
            String secondPathName,
            String thirdPathName,
            DriveSubsystem drive,
            FlywheelSubsystem flywheel,
            FeedSubsystem feed,
            IntakeSubsystem intake,
            FuelPhysicsSim ballSim,
            TurretSubsystem turret) {
        addCommands(
                new PathShootIntakeAuto(shootIntakePathName, drive, flywheel, feed, intake),
                drive.followPath(new Path(secondPathName)).withTimeout(20.0),
                new SprintAuto(thirdPathName, drive, flywheel, feed, intake, ballSim, turret));
    }
}

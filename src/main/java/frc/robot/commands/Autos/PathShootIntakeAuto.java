package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.HoldIntakeDeployed;
import frc.robot.commands.SetRollerPercent;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Follow a path while running {@link MovingAutoShoot} and the intake in
 * parallel.
 */
public class PathShootIntakeAuto extends SequentialCommandGroup {

    public PathShootIntakeAuto(
        String pathName,
        DriveSubsystem drive,
        FlywheelSubsystem flywheel,
        FeedSubsystem feed,
        IntakeSubsystem intake) 
    {
        addCommands(
            Commands.deadline(
                drive.followPath(new Path(pathName)),
                new ParallelCommandGroup(
                    new MovingAutoShoot(flywheel, feed),
                    Commands.sequence(
                        new SetRollerPercent(intake, 1.0),
                        new HoldIntakeDeployed(intake)))
            ).withTimeout(20.0));
    }
}

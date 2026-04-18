package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoPushIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.HoldIntakeDeployed;
import frc.robot.commands.SetRollerPercent;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FuelPhysicsSim;

public class DifferentAuto extends SequentialCommandGroup {

    public DifferentAuto(
        String pathName,
        String path2Name,
        DriveSubsystem drive,
        FlywheelSubsystem flywheel,
        FeedSubsystem feed,
        IntakeSubsystem intake,
        FuelPhysicsSim ballSim,
        TurretSubsystem turret
    ) {
        addCommands(
            // Follow path while running intake to collect a ball
            Commands.deadline(
                drive.followPath(new Path(pathName)),
                Commands.sequence(
                    new SetRollerPercent(intake, 1.0),
                    new HoldIntakeDeployed(intake)
                )
            ).withTimeout(15.0),

            // Shoot collected ball
            new ParallelCommandGroup(
                new AutoShoot(flywheel, feed, ballSim),
                new SequentialCommandGroup(
                    new SetRollerPercent(intake, -0.5).withTimeout(1.0),
                    new AutoPushIntake(intake)
                )
            ).withTimeout(8.0),

            // Follow path while running intake to collect a ball
            Commands.deadline(
                drive.followPath(new Path(path2Name)),
                Commands.sequence(
                    new SetRollerPercent(intake, 1.0),
                    new HoldIntakeDeployed(intake)
                )
            ).withTimeout(10.0),

            // Shoot collected ball
            new ParallelCommandGroup(
                new AutoShoot(flywheel, feed, ballSim),
                new SequentialCommandGroup(
                    new SetRollerPercent(intake, -0.5).withTimeout(1.0),
                    new AutoPushIntake(intake)
                )
            ).withTimeout(3.5)
        );
    }
}

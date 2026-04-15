package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.HoldIntakeDeployed;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FuelPhysicsSim;

public class PreloadAuto extends SequentialCommandGroup {

    public PreloadAuto(
        String pathName,
        DriveSubsystem drive,
        FlywheelSubsystem flywheel,
        FeedSubsystem feed,
        IntakeSubsystem intake,
        FuelPhysicsSim ballSim,
        TurretSubsystem turret
    ) {
        addCommands(
            Commands.waitSeconds(3.0),

            // Shoot preloaded ball
            new AutoShoot(flywheel, feed, ballSim)
                .withTimeout(4.0)
        );
    }
}

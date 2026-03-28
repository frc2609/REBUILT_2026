package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Turret;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.HoldIntakeDeployed;
import frc.robot.commands.SetIntakeSpeedRPS;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FuelPhysicsSim;

public class SnowBlowAuto extends ParallelCommandGroup {

    public SnowBlowAuto(
        String pathName,
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
                    new SetIntakeSpeedRPS(intake, Constants.Controls.INTAKE_RUN_RPM / 60.0),
                    new HoldIntakeDeployed(intake, Constants.Controls.INTAKE_DEPLOYED_DEG)
                )
            ),
            new AutoShoot(
                flywheel,
                feed, 
                Constants.Controls.FEED_HOLD_RPM, 
                Constants.Controls.AGITATOR_HOLD_RPM, 
                ballSim, 
                turret::aimIsAtPosition
            )
        );
    }
}
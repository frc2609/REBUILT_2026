package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AimTurretField;
import frc.robot.commands.FullShoot;
import frc.robot.commands.HoldIntakeDeployed;
import frc.robot.commands.SetIntakeSpeedRPS;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FuelPhysicsSim;
import frc.robot.util.ShotCalculator;

public class LeftSweepAuto extends SequentialCommandGroup {

    public LeftSweepAuto(
        DriveSubsystem drive,
        TurretSubsystem turret,
        FlywheelSubsystem flywheel,
        FeedSubsystem feed,
        IntakeSubsystem intake,
        ShotCalculator shotCalc,
        FuelPhysicsSim ballSim
    ) {
        addCommands(
            // // Phase 1: Aim turret and shoot preloaded ball
            // Commands.parallel(
            //     new AimTurretField(drive, turret, flywheel, shotCalc),
            //     new FullShoot(
            //         flywheel, feed,
            //         Constants.Controls.FEED_HOLD_RPM / 60.0,
            //         Constants.Controls.AGITATOR_HOLD_RPM / 60.0,
            //         ballSim
            //     )
            // ).withTimeout(1.5),

            // // Phase 2: Follow leftSweep path while aiming and running intake
            // Commands.parallel(
            //     drive.followPath(new Path("leftSweep")),
            //     new AimTurretField(drive, turret, flywheel, shotCalc),
            //     new SetIntakeSpeedRPS(intake, Constants.Controls.INTAKE_RUN_RPM / 60.0),
            //     new HoldIntakeDeployed(intake, Constants.Controls.INTAKE_DEPLOYED_DEG)
            // ),

            // // Phase 3: Aim turret and shoot collected balls
            // Commands.parallel(
            //     new AimTurretField(drive, turret, flywheel, shotCalc),
            //     new FullShoot(
            //         flywheel, feed,
            //         Constants.Controls.FEED_HOLD_RPM / 60.0,
            //         Constants.Controls.AGITATOR_HOLD_RPM / 60.0,
            //         ballSim
            //     )
            // ).withTimeout(3.0)
        );
    }
}

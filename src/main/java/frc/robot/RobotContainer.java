// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimTurretField;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoldAgitatorSpeed;
import frc.robot.commands.HoldIntakeSpeed;
import frc.robot.commands.HoldFlywheelSpeed;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FuelPhysicsSim;
import frc.robot.util.ProjectileSimulator;
import frc.robot.util.ShotCalculator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController driverController =
        new CommandXboxController(Constants.Controls.DRIVER_CONTROLLER_PORT);
    private final Trigger xTrigger = driverController.x();
    private final Trigger holdShooterTrigger = driverController.rightBumper();
    private final Trigger holdIntakeTrigger = driverController.a();
    private final Trigger holdAgitatorTrigger = driverController.b();

    // Dashboard inputs (later)
    // private final LoggedDashboardChooser<Command> autoChooser;

    private final RobotFactory robotFactory = new RobotFactory();
    private final TurretSubsystem turretSubsystem = robotFactory.getTurretSubsystem();
    private final FlywheelSubsystem flywheelSubsystem = robotFactory.getFlywheelSubsystem();
    private final IntakeSubsystem intakeSubsystem = robotFactory.getIntakeSubsystem();
    private final DriveSubsystem driveSubsystem = robotFactory.getDriveSubsystem();
    private final FeedSubsystem agitatorSubsystem = robotFactory.getFeedSubsystem();
    private final ClimberSubsystem climberSubsystem = robotFactory.getClimberSubsystem();

    private final Command holdShooterCommand =
        new HoldFlywheelSpeed(flywheelSubsystem, Constants.Controls.SHOOTER_HOLD_RPS);

    private final Command holdIntakeCommand = 
        new HoldIntakeSpeed(intakeSubsystem, Constants.Controls.INTAKE_HOLD_RPS);

    private final Command holdAgitatorCommand = 
        new HoldAgitatorSpeed(agitatorSubsystem, Constants.Controls.AGITATOR_HOLD_RPS);

    private final Command autoAimHubCommand;
    private final ShotCalculator shotCalculator;
    private final FuelPhysicsSim ballSim = new FuelPhysicsSim("Sim/Fuel");

    public RobotContainer() {
        ProjectileSimulator sim = new ProjectileSimulator(Constants.simParameters);
        ProjectileSimulator.GeneratedLUT lut = sim.generateLUT();
        this.shotCalculator = new ShotCalculator(Constants.shotConfig);

        for (var entry : lut.entries()) {
            if (entry.reachable()) {
                System.out.printf("%.2fm -> %.0f RPM, %.3fs TOF%n",
                    entry.distanceM(), entry.rpm(), entry.tof());
                shotCalculator.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }

        autoAimHubCommand = new AimTurretField(
            driveSubsystem, turretSubsystem, Constants.Field.BLUE_HUB, 
            shotCalculator, driverController.rightTrigger(), ballSim
        );

        if (Constants.currentMode == Constants.Mode.SIM) {
            ballSim.enable();
            ballSim.placeFieldBalls(); 

            ballSim.configureRobot(0.5, 0.5, 0.01,
                () -> driveSubsystem.getPose(), () -> driveSubsystem.getChassisSpeeds());
        }

        configureBindings();
    }

    public void updateSim() {
        ballSim.tick();
    }
    
    /** Robot-wide init hook (called from {@link Robot#robotInit()}). */
    public void robotInit() {
        // Avoid syncing absolute encoders during construction; do it at a predictable time during boot.
        climberSubsystem.resetPositionToAbsolute();
        intakeSubsystem.resetDeployPositionToAbsolute();
        turretSubsystem.resetAimPositionToAbsolute();

        //intakeSubsystem.setDeployPosition(Constants.Controls.INTAKE_DEPLOYED_ROTATIONS);
    }

    private void configureBindings() {
        holdShooterTrigger.whileTrue(holdShooterCommand);
        holdIntakeTrigger.whileTrue(holdIntakeCommand);
        holdAgitatorTrigger.whileTrue(holdAgitatorCommand);
        turretSubsystem.setDefaultCommand(autoAimHubCommand);

        // Default command, normal field-relative drive
        driveSubsystem.setDefaultCommand(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

        // Switch to X pattern when X button is pressed
        xTrigger.onTrue(Commands.runOnce(driveSubsystem::stopWithX, driveSubsystem));

        // Reset gyro to 0° when B button is pressed
        // resetGyroTrigger.onTrue(
        //     Commands.runOnce(
        //             () ->
        //                 driveSubsystem.setPose(
        //                     new Pose2d(driveSubsystem.getPose().getTranslation(), Rotation2d.kZero)),
        //             driveSubsystem)
        //         .ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

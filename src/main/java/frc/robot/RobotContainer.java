// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoldShooterSpeed;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

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
  private final Trigger resetGyroTrigger = driverController.b();
  private final Trigger holdShooterTrigger = driverController.rightBumper();

  private final RobotFactory robotFactory = new RobotFactory();
  private final ShooterSubsystem shooterSubsystem = robotFactory.getShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = robotFactory.getIntakeSubsystem();
  private final DriveSubsystem driveSubsystem = robotFactory.getDriveSubsystem();
  private final VisionSubsystem visionSubsystem = robotFactory.getVisionSubsystem();
  private final AgitatorSubsystem agitatorSubsystem = robotFactory.getAgitatorSubsystem();

  // Dashboard inputs (later)
  // private final LoggedDashboardChooser<Command> autoChooser;

  private final Command holdShooterCommand =
      new HoldShooterSpeed(shooterSubsystem, Constants.Controls.SHOOTER_HOLD_RPS);

  public RobotContainer() {
    configureBindings();
  }

  /** Robot-wide init hook (called from {@link Robot#robotInit()}). */
  public void robotInit() {
    // Avoid syncing absolute encoders during construction; do it at a predictable time during boot.
    // armSubsystem.resetPositionToAbsolute();
  }

  private void configureBindings() {
    holdShooterTrigger.whileTrue(holdShooterCommand);

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
    resetGyroTrigger.onTrue(
        Commands.runOnce(
                () ->
                    driveSubsystem.setPose(
                        new Pose2d(driveSubsystem.getPose().getTranslation(), Rotation2d.kZero)),
                driveSubsystem)
            .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

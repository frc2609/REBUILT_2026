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
import frc.robot.commands.FullShoot;
import frc.robot.commands.PushIntake;
import frc.robot.commands.SetIntakeSpeedRPS;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

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
    private final Trigger resetGyroTrigger = driverController.back();

    private final Trigger shootTrigger = driverController.rightTrigger();
    private final Trigger pushIntakeTrigger = driverController.leftBumper();
    private final Trigger startIntakeTrigger = driverController.a();
    private final Trigger stopIntakeTrigger = driverController.start();

    // Tuning controls

    private final Trigger holdAgitatorTrigger = driverController.b();
    private final Trigger holdFeedTrigger = driverController.y();
    private final Trigger holdFlywheelTrigger = driverController.rightBumper();
    private final Trigger setHoodTrigger = driverController.povUp();
    private final Trigger setIntakeTrigger = driverController.povDown();

    // Dashboard inputs (later)
    // private final LoggedDashboardChooser<Command> autoChooser;

    private final RobotFactory robotFactory = new RobotFactory();
    private final TurretSubsystem turretSubsystem = robotFactory.getTurretSubsystem();
    private final FlywheelSubsystem flywheelSubsystem = robotFactory.getFlywheelSubsystem();
    private final IntakeSubsystem intakeSubsystem = robotFactory.getIntakeSubsystem();
    private final DriveSubsystem driveSubsystem = robotFactory.getDriveSubsystem();
    private final FeedSubsystem feedSubsystem = robotFactory.getFeedSubsystem();
    private final ClimberSubsystem climberSubsystem = robotFactory.getClimberSubsystem();
    // private final VisionSubsystem visionSubsystem = robotFactory.getVisionSubsystem();

    private final Command fullShooterCommand = new FullShoot(
        flywheelSubsystem, 
        feedSubsystem, 
        Constants.Controls.FEED_HOLD_RPM / 60.0, 
        Constants.Controls.AGITATOR_HOLD_RPM / 60.0
    );

    private final Command startIntakeCommand = new SetIntakeSpeedRPS(
        intakeSubsystem, 
        Constants.Controls.INTAKE_RUN_RPM / 60.0
    );
    
    private final Command intakeDefaultSpeed = new SetIntakeSpeedRPS(
        intakeSubsystem, 
        Constants.Controls.INTAKE_IDLE_RPM / 60.0
    );

    private final Command pushIntakeCommand = new PushIntake(
        intakeSubsystem, 
        driverController::getLeftTriggerAxis, 
        Constants.Controls.INTAKE_DEPLOYED_DEG, 
        Constants.Controls.INTAKE_RETRACT_DEG
    );

    public RobotContainer() {
        // Avoid syncing absolute encoders during construction; do it at a predictable time during boot.
        climberSubsystem.resetPositionToAbsolute();
        intakeSubsystem.resetDeployPositionToAbsolute(Constants.Intake.Deploy.ZERO_OFFSET);
        turretSubsystem.resetAimPositionToAbsolute(Constants.Turret.Aim.ZERO_OFFSET);

        configureBindings();
    }

    private void configureBindings() {
        // Main Control System

        shootTrigger.whileTrue(fullShooterCommand);

        startIntakeTrigger.onTrue(startIntakeCommand);
        stopIntakeTrigger.onTrue(intakeDefaultSpeed);
        pushIntakeTrigger.whileTrue(pushIntakeCommand);

        // Tuning commands

        feedSubsystem.setSetpoints(Constants.Controls.FEED_HOLD_RPM,Constants.Controls.AGITATOR_HOLD_RPM);
        flywheelSubsystem.setSetpoint(Constants.Controls.FLYWHEEL_HOLD_RPM);
        intakeSubsystem.setDeploySetpoint(0);
        holdAgitatorTrigger.whileTrue(Commands.runEnd(feedSubsystem::setAgitatorSpeed,feedSubsystem::stop,feedSubsystem));
        holdFeedTrigger.whileTrue(Commands.runEnd(feedSubsystem::setFeedSpeed,feedSubsystem::stop,feedSubsystem));
        holdFlywheelTrigger.whileTrue(Commands.runEnd(flywheelSubsystem::setSpeed,flywheelSubsystem::stop,flywheelSubsystem));

        setIntakeTrigger.onTrue(Commands.runOnce(intakeSubsystem::setDeployPosition, intakeSubsystem));
        turretSubsystem.setSetpoints(Constants.Controls.TURRET_AIM_DEG,Constants.Controls.TURRET_HOOD_DEG);
        setHoodTrigger.onTrue(Commands.runOnce(turretSubsystem::setHoodPosition,turretSubsystem));
        //turretSubsystem.setDefaultCommand(Commands.run(turretSubsystem::setAimPosition, turretSubsystem));
        
        // Drive commands

        driveSubsystem.setDefaultCommand(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

        xTrigger.onTrue(Commands.runOnce(driveSubsystem::stopWithX, driveSubsystem));
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

// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.subsystems.VisionSubsystem;
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
    private final Trigger aimTurretTrigger = driverController.leftBumper();

    // Dashboard inputs (later)
    // private final LoggedDashboardChooser<Command> autoChooser;

    private final RobotFactory robotFactory = new RobotFactory();
    private final TurretSubsystem turretSubsystem = robotFactory.getTurretSubsystem();
    private final FlywheelSubsystem flywheelSubsystem = robotFactory.getFlywheelSubsystem();
    private final IntakeSubsystem intakeSubsystem = robotFactory.getIntakeSubsystem();
    private final DriveSubsystem driveSubsystem = robotFactory.getDriveSubsystem();
    private final FeedSubsystem agitatorSubsystem = robotFactory.getFeedSubsystem();
    private final ClimberSubsystem climberSubsystem = robotFactory.getClimberSubsystem();
    // private final VisionSubsystem visionSubsystem = robotFactory.getVisionSubsystem();

    private final ShotCalculator shotCalc;

    private final Command holdShooterCommand =
        new HoldFlywheelSpeed(flywheelSubsystem, Constants.Controls.SHOOTER_HOLD_RPS);

    private final Command holdIntakeCommand = 
        new HoldIntakeSpeed(intakeSubsystem, Constants.Controls.INTAKE_HOLD_RPS);

    private final Command holdAgitatorCommand = 
        new HoldAgitatorSpeed(agitatorSubsystem, Constants.Controls.AGITATOR_HOLD_RPS);

    private final Command autoAimHubCommand;

    public RobotContainer() {
        
        ProjectileSimulator.SimParameters params = new ProjectileSimulator.SimParameters(
            0.215,   // ball mass kg
            0.1501,  // ball diameter m
            0.47,    // drag coeff (smooth sphere)
            0.2,     // Magnus coeff
            1.225,   // air density
            0.1,    // exit height (m), floor to where the ball leaves the shooter
            0.1016,  // flywheel diameter (m), measure with calipers
            1.83,    // target height (m), from game manual
            0.6,     // slip factor (0=no grip, 1=perfect), tune this on the real robot
            75.0,    // launch angle from horizontal, measure from CAD
            0.001,   // sim timestep
            1500, 6000, 25, 5.0  // RPM search range, iterations, max sim time
        );

        ProjectileSimulator sim = new ProjectileSimulator(params);
        ProjectileSimulator.GeneratedLUT lut = sim.generateLUT();

        // in RobotContainer or wherever you set stuff up
        ShotCalculator.Config config = new ShotCalculator.Config();
        config.launcherOffsetX = -0.2;  // how far forward the launcher is from robot center (m)
        config.launcherOffsetY = -0.2;   // how far left, 0 if centered
        config.phaseDelayMs = 30.0;     // your vision pipeline latency
        config.mechLatencyMs = 20.0;    // how long the mechanism takes to respond
        config.maxTiltDeg = 5.0;        // suppress firing when chassis tilts past this (bumps/ramps)
        config.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to disable)
        config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub

        this.shotCalc = new ShotCalculator(config);

        // load the LUT you generated
        for (var entry : lut.entries()) {
            if (entry.reachable()) {
                System.out.printf("%.2fm -> %.0f RPM, %.3fs TOF%n",
                    entry.distanceM(), entry.rpm(), entry.tof());
                shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }

        autoAimHubCommand = new AimTurretField(
            driveSubsystem, turretSubsystem, Constants.Field.BLUE_HUB, shotCalc,
            driverController.rightTrigger()
        );

        configureBindings();
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

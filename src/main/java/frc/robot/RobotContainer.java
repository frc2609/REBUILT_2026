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
import frc.robot.commands.FullShoot;
import frc.robot.commands.PushIntake;
import frc.robot.commands.SetIntakeSpeedRPS;
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
    private final Trigger resetGyroTrigger = driverController.back();

    private final Trigger shootTrigger = driverController.rightTrigger();
    private final Trigger pushIntakeTrigger = driverController.leftBumper();
    private final Trigger startIntakeTrigger = driverController.a();
    private final Trigger stopIntakeTrigger = driverController.start();

    // Tuning controls

    private final Trigger holdAgitatorTrigger = driverController.b();
    private final Trigger holdFeedTrigger = driverController.y();
    private final Trigger holdFlywheelTrigger = driverController.rightBumper();
    private final Trigger setIntakeTrigger = driverController.povDown();

    // Dashboard inputs (later)
    // private final LoggedDashboardChooser<Command> autoChooser;

    private final RobotFactory robotFactory = new RobotFactory();
    public final TurretSubsystem turretSubsystem;
    public final FlywheelSubsystem flywheelSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final DriveSubsystem driveSubsystem;
    public final FeedSubsystem feedSubsystem;
    public final ClimberSubsystem climberSubsystem;

    private final Command autoAimHubCommand;
    private final ShotCalculator shotCalculator;
    private final FuelPhysicsSim ballSim = new FuelPhysicsSim("Sim/Fuel");

    public RobotContainer() {
        turretSubsystem = robotFactory.getTurretSubsystem();
        flywheelSubsystem = robotFactory.getFlywheelSubsystem();
        intakeSubsystem = robotFactory.getIntakeSubsystem();
        driveSubsystem = robotFactory.getDriveSubsystem();
        feedSubsystem = robotFactory.getFeedSubsystem();
        climberSubsystem = robotFactory.getClimberSubsystem();
        
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
            driveSubsystem, turretSubsystem, flywheelSubsystem, 
            Constants.Field.BLUE_HUB, shotCalculator, shootTrigger, ballSim
        );

        if (Constants.currentMode == Constants.Mode.SIM) {
            ballSim.enable();
            ballSim.placeFieldBalls(); 

            ballSim.configureRobot(0.5, 0.5, 0.01,
                () -> driveSubsystem.getPose(), () -> driveSubsystem.getChassisSpeeds());
        }

        // Turret encoder: MagnetOffset + inversion are now baked into CANCoderIO constructor
        // in RobotFactory. FusedCANcoder reads 0 at mechanical zero without a software reset.
        intakeSubsystem.setEncoderInvert(true);

        climberSubsystem.resetPositionToAbsolute();
        intakeSubsystem.resetDeployPositionToAbsolute(Constants.Intake.Deploy.ZERO_OFFSET);

        configureBindings();
    }

    public void updateSim() {
        ballSim.tick();
    }
    

    private void configureBindings() {
        // Main controls

        turretSubsystem.setDefaultCommand(autoAimHubCommand);
        shootTrigger.whileTrue(new FullShoot(
            flywheelSubsystem,
            feedSubsystem,
            Constants.Controls.FEED_HOLD_RPM / 60.0,
            Constants.Controls.AGITATOR_HOLD_RPM / 60.0
        ));
        startIntakeTrigger.onTrue(new SetIntakeSpeedRPS(
            intakeSubsystem,
            Constants.Controls.INTAKE_RUN_RPM / 60.0
        ));
        stopIntakeTrigger.onTrue(new SetIntakeSpeedRPS(
            intakeSubsystem,
            Constants.Controls.INTAKE_IDLE_RPM / 60.0
        ));
        pushIntakeTrigger.whileTrue(new PushIntake(
            intakeSubsystem,
            driverController::getLeftTriggerAxis,
            Constants.Controls.INTAKE_DEPLOYED_DEG,
            Constants.Controls.INTAKE_RETRACT_DEG
        ));

        // Tuning commands

        feedSubsystem.setSetpoints(Constants.Controls.FEED_HOLD_RPM,Constants.Controls.AGITATOR_HOLD_RPM);
        flywheelSubsystem.setSetpoint(Constants.Controls.FLYWHEEL_HOLD_RPM);
        intakeSubsystem.setDeploySetpoint(0);
        holdAgitatorTrigger.whileTrue(Commands.runEnd(feedSubsystem::setAgitatorSpeed,feedSubsystem::stop,feedSubsystem));
        holdFeedTrigger.whileTrue(Commands.runEnd(feedSubsystem::setFeedSpeed,feedSubsystem::stop,feedSubsystem));
        holdFlywheelTrigger.whileTrue(Commands.runEnd(flywheelSubsystem::setSpeed,flywheelSubsystem::stop,flywheelSubsystem));

        setIntakeTrigger.onTrue(Commands.runOnce(intakeSubsystem::setDeployPosition, intakeSubsystem));
        turretSubsystem.setSetpoints(Constants.Controls.TURRET_AIM_DEG,Constants.Controls.TURRET_HOOD_DEG);
        //setHoodTrigger.onTrue(Commands.runOnce(turretSubsystem::setHoodPosition,turretSubsystem));

        // Drive commands

        driveSubsystem.setDefaultCommand(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

        xTrigger.onTrue(Commands.runOnce(driveSubsystem::stopWithX, driveSubsystem));
        resetGyroTrigger.onTrue(
            Commands.runOnce(driveSubsystem::zeroHeading, driveSubsystem).ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Turret;
import frc.robot.commands.AutoAimTurret;
import frc.robot.commands.AutoPushIntake;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.commands.Autos.DifferentAuto;
import frc.robot.commands.Autos.OneCycleAuto;
import frc.robot.commands.Autos.SprintAuto;
import frc.robot.commands.Autos.SprintDoubleAuto;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.HomeHood;
import frc.robot.commands.HoldIntakeDeployed;
import frc.robot.commands.PushIntake;
import frc.robot.commands.SetIntakeSpeedRPS;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.FuelPhysicsSim;
import frc.robot.util.ProjectileSimulator;
import frc.robot.util.ShotCalculator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(
            Constants.Controls.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(
            Constants.Controls.OPERATOR_CONTROLLER_PORT);

    private final Trigger xTrigger = driverController.x();
    private final Trigger resetGyroTrigger = driverController.back();

    private final Trigger shootTrigger = driverController.rightTrigger(0.1);
    private final Trigger autoIntakeTrigger = driverController.rightTrigger(0.95);
    private final Trigger manualShootTrigger = driverController.rightBumper();
    private final Trigger pushIntakeTrigger = driverController.leftBumper();
    private final Supplier<Double> pushIntakeAxis = driverController::getLeftTriggerAxis;
    private final Trigger startIntakeTrigger = driverController.a();
    private final Trigger stopIntakeTrigger = driverController.start();
    private final Trigger zeroEncodersTrigger = driverController.b();

    private final Trigger turretOverrideFrontTrigger = driverController.povUp();
    private final Trigger turretOverrideRightTrigger = driverController.povRight();
    private final Trigger turretOverrideLeftTrigger = driverController.povLeft();
    private final Trigger turretAutoAimTrigger = driverController.povDown();

    private final Trigger rpmUpTrigger = operatorController.povUp();
    private final Trigger rpmDownTrigger = operatorController.povDown();
    private final Trigger aimLeftTrigger = operatorController.povLeft();
    private final Trigger aimRightTrigger = operatorController.povRight();

    // Tuning controls

    // private final Trigger holdAgitatorTrigger = driverController.b();
    // private final Trigger holdFeedTrigger = driverController.y();
    // private final Trigger holdFlywheelTrigger = driverController.rightBumper();
    // private final Trigger setHoodTrigger = driverController.povUp();
    // private final Trigger setIntakeTrigger = driverController.povDown();

    private final RobotFactory robotFactory = new RobotFactory();
    public final TurretSubsystem turretSubsystem;
    public final FlywheelSubsystem flywheelSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final DriveSubsystem driveSubsystem;
    public final FeedSubsystem feedSubsystem;
    public final ClimberSubsystem climberSubsystem;
    public final LedSubsystem ledSubsystem; boolean hasRun;
    

    private final AutoAimTurret autoAimCommand;
    private final ShotCalculator shotCalculator;
    private final FuelPhysicsSim ballSim = new FuelPhysicsSim("Sim/Fuel");
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");
    private AutoShoot autoShootCommand;
    private SetIntakeSpeedRPS startRollerCommand;
    private AutoPushIntake autoIntakePushCommand;

    private SlewRateLimiter filterX = new SlewRateLimiter(3.0);
    private SlewRateLimiter filterY = new SlewRateLimiter(3.0);            
                
    public RobotContainer() {
        turretSubsystem = robotFactory.getTurretSubsystem();
        flywheelSubsystem = robotFactory.getFlywheelSubsystem();
        intakeSubsystem = robotFactory.getIntakeSubsystem();
        driveSubsystem = robotFactory.getDriveSubsystem();
        feedSubsystem = robotFactory.getFeedSubsystem();
        climberSubsystem = robotFactory.getClimberSubsystem();
        ledSubsystem = new LedSubsystem(Constants.LedConstants.Length, Constants.LedConstants.Port,
                Constants.LedConstants.travelTime,Constants.LedConstants.ledGroup);
        hasRun = false;

        // SOTM Setup

        ProjectileSimulator sim = new ProjectileSimulator(Constants.simParameters);
        ProjectileSimulator.GeneratedLUT lut = sim.generateLUT(2.0, 45, 0.5);
        this.shotCalculator = new ShotCalculator(Constants.shotConfig);

        for (var entry : lut.entries()) {
            if (entry.reachable()) {
                System.out.printf("%.2fm -> %.0f RPM, %.3fs TOF%n",
                        entry.distanceM(), entry.rpm(), entry.tof());
                shotCalculator.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }

        autoAimCommand = new AutoAimTurret(
                driveSubsystem, turretSubsystem, flywheelSubsystem, shotCalculator);

        turretSubsystem.setEncoderInvert(true);
        intakeSubsystem.setEncoderInvert(true);

        climberSubsystem.resetPositionToAbsolute();
        intakeSubsystem.zeroCurrentDeployPosition(); // TEMP
        // intakeSubsystem.resetDeployPositionToAbsolute(Constants.Intake.Deploy.ZERO_OFFSET);
        turretSubsystem.resetAimPositionToAbsolute(Constants.Turret.Aim.ZERO_OFFSET);

        autoShootCommand = new AutoShoot(
                flywheelSubsystem,
                feedSubsystem,
                Constants.Controls.FEED_HOLD_RPM / 60.0,
                Constants.Controls.AGITATOR_HOLD_RPM / 60.0,
                ballSim,
                turretSubsystem::aimIsAtPosition);

        FollowPath.registerEventTrigger("autoShoot", autoShootCommand);

        startRollerCommand = new SetIntakeSpeedRPS(
                intakeSubsystem,
                Constants.Controls.INTAKE_RUN_RPM / 60.0);
        FollowPath.registerEventTrigger("startRoller", startRollerCommand);

        autoIntakePushCommand = new AutoPushIntake(
                intakeSubsystem,
                Constants.Controls.INTAKE_DEPLOYED_DEG,
                Constants.Controls.INTAKE_RETRACT_DEG);
        FollowPath.registerEventTrigger("autoIntakePush", autoIntakePushCommand);

        configureAutoChooser();
        configureBindings();
    }

    private void configureAutoChooser() {
        autoChooser.addDefaultOption("None", Commands.none());
        autoChooser.addOption("Left Sweep", new OneCycleAuto(
                "leftSweep", driveSubsystem, flywheelSubsystem, feedSubsystem, intakeSubsystem, ballSim,
                turretSubsystem));
        autoChooser.addOption("Right Sweep", new OneCycleAuto(
                "rightSweep", driveSubsystem, flywheelSubsystem, feedSubsystem, intakeSubsystem, ballSim,
                turretSubsystem));
        autoChooser.addOption("Left Sprint", new SprintAuto(
                "leftSweep", driveSubsystem, flywheelSubsystem, feedSubsystem, intakeSubsystem, ballSim,
                turretSubsystem));
        autoChooser.addOption("Right Sprint", new SprintAuto(
                "rightSweep", driveSubsystem, flywheelSubsystem, feedSubsystem, intakeSubsystem, ballSim,
                turretSubsystem));
        autoChooser.addOption("Left SprintD", new SprintDoubleAuto(
                "leftSweep", driveSubsystem, flywheelSubsystem, feedSubsystem, intakeSubsystem, ballSim,
                turretSubsystem));
        autoChooser.addOption("Right SprintD", new SprintDoubleAuto(
                "rightSweep", driveSubsystem, flywheelSubsystem, feedSubsystem, intakeSubsystem, ballSim,
                turretSubsystem));
        autoChooser.addOption("Back Sprint", new SprintAuto(
            "centerback", driveSubsystem, flywheelSubsystem, feedSubsystem, intakeSubsystem, ballSim, turretSubsystem
        ));
        autoChooser.addOption("Left Diff", new DifferentAuto(
            "leftSweep", "leftClose", driveSubsystem, flywheelSubsystem, feedSubsystem, intakeSubsystem, ballSim, turretSubsystem
        ));
        autoChooser.addOption("Right Diff", new DifferentAuto(
            "rightSweep", "rightClose", driveSubsystem, flywheelSubsystem, feedSubsystem, intakeSubsystem, ballSim, turretSubsystem
        ));
        Logger.registerDashboardInput(autoChooser);
        SmartDashboard.putData("Auto Routine", autoChooser.getSendableChooser());
    }

    private void configureBindings() {
        // Main controls

        turretSubsystem.setDefaultCommand(autoAimCommand);
        shootTrigger.whileTrue(autoShootCommand);

        manualShootTrigger.whileTrue(new Shoot(
                flywheelSubsystem,
                feedSubsystem,
                ballSim));

        startIntakeTrigger.onTrue(startRollerCommand);
        stopIntakeTrigger.onTrue(new SetIntakeSpeedRPS(
                intakeSubsystem,
                Constants.Controls.INTAKE_IDLE_RPM / 60.0));

        intakeSubsystem.setDefaultCommand(new HoldIntakeDeployed(
                intakeSubsystem,
                Constants.Controls.INTAKE_DEPLOYED_DEG));
        autoIntakeTrigger.whileTrue(autoIntakePushCommand);
        pushIntakeTrigger.whileTrue(new PushIntake(
                intakeSubsystem,
                pushIntakeAxis,
                Constants.Controls.INTAKE_DEPLOYED_DEG,
                Constants.Controls.INTAKE_RETRACT_DEG)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        // RPM trim (POV up/down)
        rpmUpTrigger.onTrue(Commands.runOnce(() -> shotCalculator.adjustOffset(50)));
        rpmDownTrigger.onTrue(Commands.runOnce(() -> shotCalculator.adjustOffset(-50)));

        // Aim angle trim (POV left/right)
        aimLeftTrigger.onTrue(Commands.runOnce(() -> shotCalculator.adjustAimOffset(10.0)));
        aimRightTrigger.onTrue(Commands.runOnce(() -> shotCalculator.adjustAimOffset(-10.0)));

        // Turret manual override (driver POV up/left/right) — holds turret at a fixed
        // robot-relative angle
        turretOverrideFrontTrigger.onTrue(
                Commands.run(() -> turretSubsystem.setAimPosition(Constants.Controls.TURRET_OVERRIDE_FRONT_DEG),
                        turretSubsystem));
        turretOverrideRightTrigger.onTrue(
                Commands.run(() -> turretSubsystem.setAimPosition(Constants.Controls.TURRET_OVERRIDE_RIGHT_DEG),
                        turretSubsystem));
        turretOverrideLeftTrigger.onTrue(
                Commands.run(() -> turretSubsystem.setAimPosition(Constants.Controls.TURRET_OVERRIDE_LEFT_DEG),
                        turretSubsystem));

        // Driver POV down — cancel override and restore auto-aim default command
        turretAutoAimTrigger.onTrue(Commands.runOnce(() -> {
            Command current = turretSubsystem.getCurrentCommand();
            if (current != null && current != autoAimCommand)
                current.cancel();
        }));

        new Trigger(autoAimCommand::shouldRumble).whileTrue(Commands.startEnd(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5),
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

        // Zero encoders to current positions (B button)
        // zeroEncodersTrigger.onTrue(Commands.runOnce(() -> {
        // climberSubsystem.zeroCurrentPosition();
        // //intakeSubsystem.zeroCurrentDeployPosition();
        // turretSubsystem.zeroCurrentAimPosition();
        // }).ignoringDisable(true));

        operatorController.b().onTrue(Commands.runOnce(() -> {
            turretSubsystem.resetAimPositionToAbsolute(Constants.Turret.Aim.ZERO_OFFSET);
        }));

        // Tuning commands

        feedSubsystem.setSetpoints(Constants.Controls.FEED_HOLD_RPM, Constants.Controls.AGITATOR_HOLD_RPM);
        flywheelSubsystem.setSetpoint(Constants.Controls.FLYWHEEL_LOB_RPM);
        // intakeSubsystem.setDeploySetpoint(0);
        // holdAgitatorTrigger.whileTrue(Commands.runEnd(feedSubsystem::setAgitatorSpeed,feedSubsystem::stop,feedSubsystem));
        // holdFeedTrigger.whileTrue(Commands.runEnd(feedSubsystem::setFeedSpeed,feedSubsystem::stop,feedSubsystem));
        // holdFlywheelTrigger.whileTrue(Commands.runEnd(flywheelSubsystem::setSpeed,flywheelSubsystem::stop,flywheelSubsystem));
        // setIntakeTrigger.onTrue(Commands.runOnce(intakeSubsystem::setDeployPosition,
        // intakeSubsystem));
        // setHoodTrigger.onTrue(Commands.runOnce(turretSubsystem::setHoodPosition,turretSubsystem));

        // Drive commands

        driveSubsystem.setDefaultCommand(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> filterY.calculate(-driverController.getLeftY()),
                () -> filterX.calculate(-driverController.getLeftX()),
                () -> -driverController.getRightX(),
                1.0));
        shootTrigger.whileTrue(
            DriveCommands.joystickDrive(
                driveSubsystem,
                () -> filterY.calculate(-driverController.getLeftY()),
                () -> filterX.calculate(-driverController.getLeftX()),
                () -> -driverController.getRightX(),
                Constants.Controls.SHOOTING_SPEED_PERCENT)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        xTrigger.onTrue(Commands.runOnce(driveSubsystem::stopWithX, driveSubsystem));
        resetGyroTrigger.onTrue(
                Commands.runOnce(driveSubsystem::zeroHeading, driveSubsystem).ignoringDisable(true));
    }

    // Called from Robot.java

    public void simInit() {
        ballSim.enable();
        // ballSim.placeFieldBalls();

        ballSim.configureRobot(0.5, 0.5, 0.01,
                () -> driveSubsystem.getPose(), () -> driveSubsystem.getChassisSpeeds());
    }

    public void updateSim() {
        ballSim.tick();
    }
    public Command getHoodHomeCommand() {
        return new HomeHood(turretSubsystem);
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void disabledPeriodic() {
        if (!hasRun && ledSubsystem.deployedWait.get() > 5) {
            ledSubsystem.SignalEndDeploy();
            hasRun = true;
        }
        ;

        ledSubsystem.pattern();
    }
}

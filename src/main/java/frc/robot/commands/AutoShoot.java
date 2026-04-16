package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SystemState;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.util.FuelPhysicsSim;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class AutoShoot extends Command {
    private final FlywheelSubsystem flywheel;
    private final FeedSubsystem agitator;
    private final FuelPhysicsSim ballSim;
    private final LoggedNetworkNumber flywheelTolerance;
    private int i = 0;

    public AutoShoot(
        FlywheelSubsystem flywheel, FeedSubsystem agitator, FuelPhysicsSim ballSim
    ) {
        this.flywheel = flywheel;
        this.agitator = agitator;
        this.ballSim = ballSim;

        flywheelTolerance = 
            new LoggedNetworkNumber(
                "SOTM/FlywheelToleranceRPM", 
                Constants.Controls.FLYWHEEL_TOLERANCE_RPM
            );

        addRequirements(flywheel, agitator);
    }

    @Override
    public void execute() {
        if (SystemState.calculatedFlywheelRPM != 0.0) {
            flywheel.setSpeed(SystemState.calculatedFlywheelRPM/60.0);
        }

        boolean flywheelAtSpeed = flywheel.isAtSpeed(
            SystemState.isPassing ? 
            (Constants.Controls.FLYWHEEL_PASS_TOLERANCE_RPM/60.0) : 
            (flywheelTolerance.getAsDouble()/60.0)
        );

        if (SystemState.validShotDetected && 
            SystemState.trenchBlocked &&
            flywheelAtSpeed
        ) {
            agitator.setAgitatorSpeed();
            agitator.setFeedSpeed();

            if (Constants.currentMode == Constants.Mode.SIM) {
                if (i%4 == 0) {
                    ballSim.launchBall(SystemState.launchPosSim, SystemState.launchVelSim, 0.0);
                    i++;
                    return;
                }
                i++;
            }
        } else {
            agitator.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        agitator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

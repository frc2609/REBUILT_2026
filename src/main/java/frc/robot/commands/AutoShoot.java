package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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
    private int i = 0;

    public AutoShoot(
        FlywheelSubsystem flywheel, FeedSubsystem agitator, FuelPhysicsSim ballSim
    ) {
        this.flywheel = flywheel;
        this.agitator = agitator;
        this.ballSim = ballSim;

        addRequirements(flywheel, agitator);
    }

    @Override
    public void execute() {
        flywheel.useAutoSpeed();

        if (SystemState.validShotDetected && flywheel.isAtSpeed(
            SystemState.isPassing ? 
                (Constants.Controls.FLYWHEEL_PASS_TOLERANCE_RPM/60.0) : 
                (Constants.Controls.FLYWHEEL_TOLERANCE_RPM/60.0)
        )) {
            agitator.setAgitatorSpeed();
            agitator.setFeedSpeed();

            if (Constants.currentMode == Constants.Mode.SIM) {
                if (i%4 == 0) {
                    ballSim.launchBall(flywheel.launchPosSim, flywheel.launchSpeedSim, 0.0);
                    i++;
                    return;
                }
                i++;
            }
        }
        else {
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

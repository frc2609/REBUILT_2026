package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.util.FuelPhysicsSim;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class AutoShoot extends Command {
    private final FlywheelSubsystem flywheel;
    private final FeedSubsystem agitator;
    private final double feedRPS;
    private final double agitatorRPS;
    private final FuelPhysicsSim ballSim;
    private final Supplier<Boolean> turretInPose;
    private int i = 0;

    public AutoShoot(
        FlywheelSubsystem flywheel, FeedSubsystem agitator, 
        double feedRPS, double agitatorRPS, FuelPhysicsSim ballSim,
        Supplier<Boolean> turretInPose
    ) {
        this.flywheel = flywheel;
        this.agitator = agitator;
        this.ballSim = ballSim;
        this.turretInPose = turretInPose;
        
        this.feedRPS = feedRPS;
        this.agitatorRPS = agitatorRPS;

        addRequirements(flywheel, agitator);
    }

    @Override
    public void execute() {
        //System.out.println("SHOOTING>"+flywheel.validShotDetected())

        flywheel.useAutoSpeed();

        Logger.recordOutput("turretReady", turretInPose.get());
        Logger.recordOutput("flywheelReady", flywheel.isAtSpeed());

        if (flywheel.validShotDetected() && flywheel.isAtSpeed()) {
            agitator.setAgitatorSpeed();
            agitator.setFeedSpeed();

            // if (flywheel.isAtSpeed(1.0)) // coast or brake feed to not shoot

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

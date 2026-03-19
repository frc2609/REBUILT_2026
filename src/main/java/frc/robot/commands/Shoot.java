package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.util.FuelPhysicsSim;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class Shoot extends Command {
    private final FlywheelSubsystem flywheel;
    private final FeedSubsystem agitator;
    private final FuelPhysicsSim ballSim;
    private int i = 0;
    private final Consumer<Double> controllerRumble;

    public Shoot(
        FlywheelSubsystem flywheel, FeedSubsystem agitator, 
        FuelPhysicsSim ballSim, Consumer<Double> controllerRumble
    ) {
        this.flywheel = flywheel;
        this.agitator = agitator;
        this.ballSim = ballSim;
        this.controllerRumble = controllerRumble;

        addRequirements(flywheel, agitator);
    }

    @Override
    public void execute() {
        flywheel.setSpeed();
        agitator.setAgitatorSpeed();
        agitator.setFeedSpeed();

        if (Constants.currentMode == Constants.Mode.SIM) {
            if (i%4 == 0) {
                ballSim.launchBall(flywheel.launchPosSim, flywheel.launchSpeedSim, 0.0);
                controllerRumble.accept(0.5);
                i++;
                return;
            }
            i++;
        }
        controllerRumble.accept(0.0);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        agitator.stop();
        controllerRumble.accept(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class FullShoot extends Command {
    private final FlywheelSubsystem flywheel;

    private final FeedSubsystem agitator;
    private final double feedRPS;
    private final double agitatorRPS;
    private double flywheelRPS;

    public FullShoot(
        FlywheelSubsystem flywheel, FeedSubsystem agitator, 
        double feedRPS, double agitatorRPS
    ) {
        this.flywheel = flywheel;
        this.agitator = agitator;

        this.feedRPS = feedRPS;
        this.agitatorRPS = agitatorRPS;
        flywheelRPS = flywheel.getSetpointRPS();
 
        addRequirements(flywheel, agitator);
    }

    @Override
    public void execute() {
        flywheel.bangBang(flywheelRPS, 0.1);
        agitator.setAgitatorSpeed(agitatorRPS);
        agitator.setFeedSpeed(feedRPS);
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

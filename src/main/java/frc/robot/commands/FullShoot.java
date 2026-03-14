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

    public FullShoot(
        FlywheelSubsystem flywheel, FeedSubsystem agitator, 
        double feedPercent, double agitatorRPS
    ) {
        this.flywheel = flywheel;
        this.agitator = agitator;

        this.feedRPS = feedPercent;
        this.agitatorRPS = agitatorRPS;
 
        addRequirements(flywheel, agitator);
    }

    @Override
    public void execute() {
        flywheel.setSpeed(2200.0/60.0); // Variable
        agitator.setAgitatorSpeed(2000/60);
        agitator.setFeedSpeed(2500/60);
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

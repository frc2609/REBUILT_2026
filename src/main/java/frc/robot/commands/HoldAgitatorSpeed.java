package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeedSubsystem;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class HoldAgitatorSpeed extends Command {
    private final FeedSubsystem feedSubsystem;
    private final double rps;

    public HoldAgitatorSpeed(FeedSubsystem feedSubsystem, double rps) {
        this.feedSubsystem = feedSubsystem;
        this.rps = rps;
        addRequirements(feedSubsystem);
    }

    @Override
    public void execute() {
        feedSubsystem.setAgitatorSpeed(rps);
        feedSubsystem.setFeedSpeed(rps);
    }

    @Override
    public void end(boolean interrupted) {
        feedSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class HoldAgitatorSpeed extends Command {
    private final AgitatorSubsystem agitator;
    private final double rps;

    public HoldAgitatorSpeed(AgitatorSubsystem agitator, double rps) {
        this.agitator = agitator;
        this.rps = rps;
        addRequirements(agitator);
    }

    @Override
    public void execute() {
        agitator.setSpeed(rps);
    }

    @Override
    public void end(boolean interrupted) {
        agitator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

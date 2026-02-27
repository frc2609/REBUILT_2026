package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class HoldFlywheelSpeed extends Command {
    private final FlywheelSubsystem flywheel;
    private final double rps;

    public HoldFlywheelSpeed(FlywheelSubsystem flywheel, double rps) {
        this.flywheel = flywheel;
        this.rps = rps;
        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        flywheel.setSpeed(rps);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

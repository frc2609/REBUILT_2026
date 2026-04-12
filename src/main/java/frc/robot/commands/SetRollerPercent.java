package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class SetRollerPercent extends Command {
    private final IntakeSubsystem intake;
    private final double direction;

    public SetRollerPercent(IntakeSubsystem intake, double direction) {
        this.intake = intake;
        this.direction = direction;
        addRequirements(intake);
    }
    
    @Override
    public void execute() {
        intake.setRollerPercent(direction);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class SetRollerPercent extends Command {
    private final IntakeSubsystem intake;
    private final double percent;

    public SetRollerPercent(IntakeSubsystem intake, double percent) {
        this.intake = intake;
        this.percent = percent;
        addRequirements(intake);
    }
    
    @Override
    public void execute() {
        intake.setRollerPercent(percent);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

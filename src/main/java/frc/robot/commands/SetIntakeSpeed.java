package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class SetIntakeSpeed extends Command {
    private final IntakeSubsystem intake;
    private final double rps;

    public SetIntakeSpeed(IntakeSubsystem intake, double rps) {
        this.intake = intake;
        this.rps = rps;
        addRequirements(intake);
    }
    @Override
    public void execute() {
        intake.setRollerSpeed(rps);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class HoldIntakeDeployed extends Command {
    private final IntakeSubsystem intake;
    
    public HoldIntakeDeployed(IntakeSubsystem intake){
        this.intake = intake; 
        addRequirements(intake);   
    }

    @Override
    public void execute() {
        // Dynamic holding (PID slots) can go here
        intake.setDeployPosition();
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
}
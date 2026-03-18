package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class HoldIntakeDeployed extends Command {
    private final IntakeSubsystem intake;
    private double degrees;
    
    public HoldIntakeDeployed(IntakeSubsystem intake, double degrees){
        this.intake = intake; 
        this.degrees = degrees;
        addRequirements(intake);   
    }

    @Override
    public void execute() {
        // Dynamic holding (PID slots) can go here
        intake.setDeployPosition(degrees);
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
}
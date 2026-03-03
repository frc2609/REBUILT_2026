package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class HoldIntakePos extends Command {
    private final IntakeSubsystem intake;
    private double pos;
    
    public HoldIntakePos(IntakeSubsystem intake, double pos){
        this.intake = intake; 
        this.pos = pos;
        addRequirements(intake);   
    }
    @Override
    public void execute (){
        intake.setDeployPosition (pos);
    }
    @Override 
    public boolean isFinished(){
        return false;
    }
}
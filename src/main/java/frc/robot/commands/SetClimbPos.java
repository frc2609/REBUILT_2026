package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimbPos extends Command {
    private final ClimberSubsystem climber;
    private double degrees;
    
    public SetClimbPos(ClimberSubsystem climber, double degrees){
        this.climber = climber; 
        this.degrees = degrees;
        addRequirements(climber);
    }
    @Override
    public void execute() {
        climber.setToPosition(degrees);
    }
    @Override 
    public boolean isFinished(){
        return true;
    }
}
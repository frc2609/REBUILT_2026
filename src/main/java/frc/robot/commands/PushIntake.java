package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class PushIntake extends Command {
    private final IntakeSubsystem intake;
    private Supplier<Double> triggerSupplier;
    
    public PushIntake(
        IntakeSubsystem intake, Supplier<Double> triggerSupplier
    ){
        this.intake = intake; 
        this.triggerSupplier = triggerSupplier;
        addRequirements(intake);   
    }
    @Override
    public void execute() {
        double triggerValue = triggerSupplier.get();
        intake.setDeployPosition(MathUtil.interpolate(intake.getDeploySetpoint(), 0.0, triggerValue));
    }
    @Override 
    public boolean isFinished(){
        return false;
    }
}
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class PushIntake extends Command {
    private final IntakeSubsystem intake;
    private Supplier<Double> triggerSupplier;
    private final double min, max;
    
    public PushIntake(
        IntakeSubsystem intake, Supplier<Double> triggerSupplier,
        double minPosition, double maxPosition
    ){
        this.intake = intake; 
        this.triggerSupplier = triggerSupplier;
        this.min = minPosition;
        this.max = maxPosition;
        addRequirements(intake);   
    }
    @Override
    public void execute() {
        double triggerValue = triggerSupplier.get();
        intake.setDeployPosition(MathUtil.interpolate(min, max, triggerValue));
    }
    @Override 
    public boolean isFinished(){
        return false;
    }
}
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoPushIntake extends Command {
    private final IntakeSubsystem intake;
    private final Timer timer = new Timer();
    private final double maxTime;
    private final double min, max;
    
    public AutoPushIntake(
        IntakeSubsystem intake, double minPosition, double maxPosition,
        double maxTime
    ){
        this.intake = intake; 
        this.min = minPosition;
        this.max = maxPosition;
        this.maxTime = maxTime;
        addRequirements(intake);   
    }

    private double curve(double t) {
        // return 0.8-Math.pow(2.6, -t-0.1)+0.2*Math.sin(2.8*t-7.0);
        
        if (t < maxTime) {
            return t/maxTime;
        } else {
            return 0.8+0.2*Math.sin(2.8*(0.55+t-maxTime));
        }
    }

    @Override
    public void initialize() {
        timer.restart();
    }
    @Override
    public void execute() {
        intake.setDeployPosition(
            MathUtil.interpolate(min, max, curve(timer.get()))
        );
    }
    @Override 
    public boolean isFinished(){
        return false;
    }
}
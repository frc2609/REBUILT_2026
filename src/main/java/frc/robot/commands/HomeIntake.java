package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class HomeIntake extends Command {

    private final IntakeSubsystem intake;
    private final Timer timer;
    private int stallCycles;

    public HomeIntake(IntakeSubsystem intake) {
        this.intake = intake;
        this.timer = new Timer();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        stallCycles = 0;
        timer.reset();
        timer.start();
        intake.setDeployPosition(Constants.Intake.Deploy.HOME_TARGET_DEG);
    }

    @Override
    public void execute() {
        double current = intake.getDeployCurrentAmps();
        if (current >= Constants.Intake.Deploy.HOME_CURRENT_THRESHOLD_AMPS) {
            stallCycles++;
        } else {
            stallCycles = 0;
        }
    }

    @Override
    public boolean isFinished() {
        if (Constants.currentMode == Constants.Mode.SIM || timer.get() > 3.0) {
            return true;
        }
        return stallCycles >= Constants.Intake.Deploy.HOME_CONFIRM_CYCLES;
    }

    @Override
    public void end(boolean interrupted) {
        boolean stalled = stallCycles >= Constants.Intake.Deploy.HOME_CONFIRM_CYCLES;
        boolean sim = Constants.currentMode == Constants.Mode.SIM;
        if (!interrupted && (stalled || sim)) {
            intake.zeroDeployPosition();
            if (sim) {
                System.out.println("HomeIntake: sim mode, zeroed without stall");
            } else {
                System.out.println("HomeIntake: zeroed at stall, current="+ intake.getDeployCurrentAmps() + "A");
            }
        } else if (interrupted) {
            System.out.println("HomeIntake: interrupted before stall detected");
        } else {
            System.out.println("HomeIntake: timed out before stall detected, encoder not zeroed");
        }
        intake.stopDeploy();
    }
}

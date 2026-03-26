package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class HomeIntake extends Command {

    private final IntakeSubsystem intake;
    private final Timer timer;
    private int stallCycles;
    private boolean stallConfirmed;

    public HomeIntake(IntakeSubsystem intake) {
        this.intake = intake;
        this.timer = new Timer();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        stallCycles = 0;
        stallConfirmed = false;
        timer.reset();
        timer.start();
        intake.setDeployPosition(Constants.Intake.Deploy.HOME_TARGET_DEG);
    }

    @Override
    public void execute() {
        if (timer.get() < Constants.Intake.Deploy.HOME_STALL_GUARD_SECS) {
            return;
        }
        double current = intake.getDeployCurrentAmps();
        if (current >= Constants.Intake.Deploy.HOME_CURRENT_THRESHOLD_AMPS) {
            stallCycles++;
            if (stallCycles >= Constants.Intake.Deploy.HOME_CONFIRM_CYCLES) {
                stallConfirmed = true;
            }
        } else {
            stallCycles = 0;
        }
    }

    @Override
    public boolean isFinished() {
        if (Constants.currentMode == Constants.Mode.SIM) {
            return true;
        }
        if (timer.get() > Constants.Intake.Deploy.HOME_TIMEOUT_SECS) {
            return true;
        }
        return stallConfirmed;
    }

    @Override
    public void end(boolean interrupted) {
        boolean sim = Constants.currentMode == Constants.Mode.SIM;
        double lastCurrentA = intake.getDeployCurrentAmps();

        // Clear homing setpoint before setPosition(0) — avoids a torque spike / hop off the stop.
        intake.stopDeploy();

        if (!interrupted && (stallConfirmed || sim)) {
            intake.zeroDeployPosition();
            intake.setDeployPosition(Constants.Controls.INTAKE_DEPLOYED_DEG);
            System.out.println("HomeIntake: zeroed at stall, current was ~" + lastCurrentA + "A");
        } else if (!interrupted) {
            System.out.println(
                "HomeIntake: timeout without stall — encoder not zeroed (raise timeout or lower current threshold)");
        } else {
            System.out.println("HomeIntake: interrupted before stall detected");
        }
    }
}

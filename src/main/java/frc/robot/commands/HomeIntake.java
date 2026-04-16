package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

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
        Logger.recordOutput("MechanismOutput/HomeIntake/Running", true);
        Logger.recordOutput("MechanismOutput/HomeIntake/StalledThisCycle", false);
        Logger.recordOutput("MechanismOutput/HomeIntake/StallCycles", 0);
    }

    @Override
    public void execute() {
        double current = intake.getDeployCurrentAmps();
        double velAbs = Math.abs(intake.getDeployVelocityDegPerSec());
        boolean stalled =
            current >= Constants.Intake.Deploy.HOME_CURRENT_THRESHOLD_AMPS
                && velAbs <= Constants.Intake.Deploy.HOME_STALL_MAX_VEL_DEG_PER_SEC;
        if (stalled) {
            stallCycles++;
        } else {
            stallCycles = 0;
        }
        Logger.recordOutput("MechanismOutput/HomeIntake/StalledThisCycle", stalled);
        Logger.recordOutput("MechanismOutput/HomeIntake/StallCycles", stallCycles);
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
                System.out.println(
                    "HomeIntake: zeroed at stall, current="
                        + intake.getDeployCurrentAmps()
                        + "A vel="
                        + intake.getDeployVelocityDegPerSec()
                        + " deg/s");
            }
        } else if (interrupted) {
            System.out.println("HomeIntake: interrupted before stall detected");
        } else {
            System.out.println("HomeIntake: timed out before stall detected, encoder not zeroed");
        }
        Logger.recordOutput("MechanismOutput/HomeIntake/Running", false);
        Logger.recordOutput("MechanismOutput/HomeIntake/StalledThisCycle", false);
        Logger.recordOutput("MechanismOutput/HomeIntake/StallCycles", 0);
        intake.stopDeploy();
    }
}

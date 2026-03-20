package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Drives the hood backwards toward the down hard stop, waits for the stator
 * current to spike (indicating a stall against the stop), then zeros the
 * encoder at that position.
 *
 * <p>Run this on every enable (teleop + auto) so the hood is always zeroed
 * even if it got stuck up during the disabled period.
 */
public class HomeHood extends Command {

    private final TurretSubsystem turret;
    private final Timer timer;
    private int stallCycles;

    public HomeHood(TurretSubsystem turret) {
        this.turret = turret;
        this.timer = new Timer();
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        stallCycles = 0;
        timer.start();
        turret.setHoodPosition(Constants.Turret.Hood.HOME_TARGET_DEG);
    }

    @Override
    public void execute() {
        double current = turret.getHoodCurrentAmps();
        if (current >= Constants.Turret.Hood.HOME_CURRENT_THRESHOLD_AMPS) {
            stallCycles++;
        } else {
            stallCycles = 0;
        }
    }

    @Override
    public boolean isFinished() {
        if (Constants.currentMode == Constants.Mode.SIM || timer.get() > 3.0){
            return true;
        }
        return stallCycles >= Constants.Turret.Hood.HOME_CONFIRM_CYCLES;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            turret.zeroHoodPosition();
            System.out.println("HomeHood: zeroed at stall, current=" + turret.getHoodCurrentAmps() + "A");
        } else {
            System.out.println("HomeHood: interrupted before stall detected");
        }
        turret.stop();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** Holds the shooter at a requested speed (RPS) while scheduled. */
public class HoldShooterSpeed extends Command {
    private final ShooterSubsystem shooter;
    private final double rps;

    public HoldShooterSpeed(ShooterSubsystem shooter, double rps) {
        this.shooter = shooter;
        this.rps = rps;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setSpeed(rps);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


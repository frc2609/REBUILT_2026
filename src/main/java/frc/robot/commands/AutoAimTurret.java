package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SystemState;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAimTurret extends Command {
    private final TurretSubsystem turret;
    private final LoggedNetworkNumber kVTarget;

    public AutoAimTurret(TurretSubsystem turret) {
        this.turret = turret;
        kVTarget = new LoggedNetworkNumber("/Tuning/SOTM/turretAimkV", -0.7);

        addRequirements(turret);
    }

    @Override
    public void execute() {
        boolean turretInLimits = Math.abs(SystemState.turretAngleDeg) <= (Constants.Turret.Aim.RANGE_DEG);
        if (turretInLimits) {
            turret.setAimPositionFF(
                SystemState.turretAngleDeg, 
                kVTarget.get()*SystemState.shot.driveAngularVelocityRadPerSec()
            );
        } else {
            //turret.setAimPosition(0.0);
        }

        turret.setHoodPosition(SystemState.hoodAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


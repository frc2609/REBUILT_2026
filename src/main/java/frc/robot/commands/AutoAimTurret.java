package frc.robot.commands;

import java.util.Optional;

import javax.swing.plaf.synth.SynthTextAreaUI;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SystemState;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.ProjectileSimulator;
import frc.robot.util.ShotCalculator;

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
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


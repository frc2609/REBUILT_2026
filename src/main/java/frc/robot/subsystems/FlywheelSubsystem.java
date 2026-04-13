package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class FlywheelSubsystem extends SubsystemBase {
    private final VelocityMotorIO flywheelMotor;
    private double lastSetpoint = 0.0;

    public FlywheelSubsystem(
        VelocityMotorIO flywheelMotor
    ) {
        this.flywheelMotor = flywheelMotor;
    }

    public void setSetpoint(double rps) {
        flywheelMotor.setSetpoint(rps);
    }
    public double getSetpointRPS() {
        return flywheelMotor.getSetpointRPM()/60.0;
    }

    public void setSpeed() {
        setSpeed(getSetpointRPS());
    }
    public void setSpeed(double rotationsPerSecond) {
        lastSetpoint = rotationsPerSecond;
        flywheelMotor.setVelocityRps(rotationsPerSecond);
    }

    public boolean isAtSpeed(double toleranceRPS) {
        boolean isAtSpeed = 
            Math.abs(lastSetpoint-flywheelMotor.getVelocityRps()) < toleranceRPS;
        Logger.recordOutput("SOTM/Flags/FlywheelReady", isAtSpeed);
        return isAtSpeed;
    }

    public void stop() {
        flywheelMotor.stop();
    }

    @Override
    public void periodic()
    {
        flywheelMotor.logMotorPID();
        flywheelMotor.updateFromTunables();
    }
}

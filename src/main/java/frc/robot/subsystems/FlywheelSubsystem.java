package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class FlywheelSubsystem extends SubsystemBase {
    private final VelocityMotorIO flywheelMotor;

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
        flywheelMotor.setVelocityRps(getSetpointRPS());
    }
    public void setSpeed(double rotationsPerSecond) {
        flywheelMotor.setVelocityRps(rotationsPerSecond);
    }

    public void bangBang(double speedRPS, double kF){
        if (flywheelMotor.getVelocityRps() < speedRPS) {
            flywheelMotor.set(1.0);
        } else {
            flywheelMotor.set(kF);
        }
    }

    public boolean isAtSpeed(double tolerance) {
        return flywheelMotor.isAtSpeed(tolerance);
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

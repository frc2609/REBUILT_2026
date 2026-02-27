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

    public void setSpeed(double rotationsPerSecond) {
        flywheelMotor.setVelocityRps(rotationsPerSecond);
    }

    public boolean isAtSpeed(double tolerance) {
        return flywheelMotor.isAtSpeed(tolerance);
    }

    public void stop() {
        flywheelMotor.stop();
    }
}

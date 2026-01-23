package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

/**
 * Shooter Subsystem using velocity control (rotations per second).
 */
public class ShooterSubsystem extends SubsystemBase {
	private final VelocityMotorIO shooterMotor;

	public ShooterSubsystem(VelocityMotorIO shooterMotor) {
		this.shooterMotor = shooterMotor;
	}

	public void setSpeed(double rotationsPerSecond) {
		shooterMotor.setVelocityRps(rotationsPerSecond);
	}

	public boolean isAtSpeed(double tolerance) {
		return shooterMotor.isAtSpeed(tolerance);
	}

	public boolean isReady() {
		return isAtSpeed(2.0);
	}

	public void stop() {
		shooterMotor.stop();
	}
}


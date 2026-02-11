package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class AgitatorSubsystem extends SubsystemBase {
  private final VelocityMotorIO motor;

  public AgitatorSubsystem(VelocityMotorIO motor) {
    this.motor = motor;
  }

  public void setSpeed(double rotationsPerSecond) {
    motor.setVelocityRps(rotationsPerSecond);
  }

  public boolean isAtSpeed(double toleranceDegrees) {
    return motor.isAtSpeed(toleranceDegrees);
  }

  public void stop() {
    motor.stop();
  }
}

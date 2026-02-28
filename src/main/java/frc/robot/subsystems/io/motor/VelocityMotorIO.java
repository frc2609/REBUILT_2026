package frc.robot.subsystems.io.motor;

/** Hardware-agnostic velocity control (rotations per second). */
public interface VelocityMotorIO {
  void setVelocityRps(double rotationsPerSecond);

  double getVelocityRps();

  boolean isAtSpeed(double toleranceRps);

  void stop();
}

package frc.robot.subsystems.io.motor;

import java.util.Map;

/** Hardware-agnostic velocity control (rotations per second). */
public interface VelocityMotorIO {
  void setVelocityRps(double rotationsPerSecond);

  double getVelocityRps();

  void setConfiguration(Map<String, Object> cfg);

  boolean isAtSpeed(double toleranceRps);

  void stop();
}

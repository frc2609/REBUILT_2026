package frc.robot.subsystems.io.motor;

import java.util.Map;

/** Hardware-agnostic position control (degrees). */
public interface PositionMotorIO {
  void setTargetPositionDegrees(double degrees);

  double getPositionDegrees();

  boolean isAtPosition(double toleranceDegrees);

  void setConfiguration(Map<String, Object> cfg);

  void resetToAbsolute(double absolutePositionRotations);

  void stop();
}

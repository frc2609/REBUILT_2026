package frc.robot.subsystems.io.motor;

/** Hardware-agnostic position control (degrees). */
public interface PositionMotorIO {
  void setTargetPositionDegrees(double degrees);

  double getPositionDegrees();

  boolean isAtPosition(double toleranceDegrees);

  void resetToAbsolute(double absolutePositionRotations);

  void stop();
}

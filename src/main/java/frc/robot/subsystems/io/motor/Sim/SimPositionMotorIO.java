package frc.robot.subsystems.io.motor.Sim;

import frc.robot.subsystems.io.motor.PositionMotorIO;

public class SimPositionMotorIO implements PositionMotorIO {
  private double positionDegrees = 0.0;
  private double targetDegrees = 0.0;

  @Override
  public void setTargetPositionDegrees(double degrees) {
    targetDegrees = degrees;
    positionDegrees = degrees;
  }

  @Override
  public void updateFromTunables() {
      // This satisfies the PositionMotorIO interface
  }

  @Override
  public void logMotorPID() {
      // This satisfies the VelocityMotorIO interface
  }


  @Override
  public double getPositionDegrees() {
    return positionDegrees;
  }

  @Override
  public boolean isAtPosition(double toleranceDegrees) {
    return Math.abs(targetDegrees - positionDegrees) <= toleranceDegrees;
  }

  @Override
  public void resetToAbsolute(double absolutePositionRotations) {
    positionDegrees = absolutePositionRotations * 360.0;
    targetDegrees = positionDegrees;
  }

  @Override
  public void stop() {
    // No-op for simple sim.
  }
}

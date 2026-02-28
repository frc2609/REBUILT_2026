package frc.robot.subsystems.io.motor.CTRE.Sim;

import frc.robot.subsystems.io.motor.PercentMotorIO;

public class SimPercentMotorIO implements PercentMotorIO {
  private double percent = 0.0;

  @Override
  public void setPercent(double percent) {
    this.percent = percent;
  }

  @Override
  public void stop() {
    this.percent = 0.0;
  }

  public double getPercent() {
    return percent;
  }
}

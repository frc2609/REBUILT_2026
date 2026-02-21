package frc.robot.subsystems.io.encoder.impl;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;

public class WpiDutyCycleEncoderIO implements AbsEncoderIO {
  private final DutyCycleEncoder encoder;

  public WpiDutyCycleEncoderIO(int encoderPort) {
    encoder = new DutyCycleEncoder(encoderPort);
  }

  @Override
  public double getRotations() {
    return encoder.get();
  }

  @Override
  public void close() {
    encoder.close();
  }

  @Override
  public void setRange(double minimum, double maximum) {
    encoder.setDutyCycleRange(minimum,maximum);
  }

  @Override
  public void setInverted(boolean isInverted) {
    encoder.setInverted(isInverted);
  }
}

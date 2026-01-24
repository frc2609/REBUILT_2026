package frc.robot.subsystems.io.encoder.impl;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;

public class WpiDutyCycleEncoderIO implements AbsEncoderIO {
  private final DutyCycleEncoder encoder;

  public WpiDutyCycleEncoderIO(DutyCycleEncoder encoder) {
    this.encoder = encoder;
  }

  @Override
  public double getAbsolutePositionRotations() {
    return encoder.get();
  }
}

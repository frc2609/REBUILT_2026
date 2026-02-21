package frc.robot.subsystems.io.encoder.impl;

import frc.robot.subsystems.io.encoder.AbsEncoderIO;

public class SimAbsEncoderIO implements AbsEncoderIO {
  private final double absolutePositionRotations;

  public SimAbsEncoderIO(double absolutePositionRotations) {
    this.absolutePositionRotations = absolutePositionRotations;
  }

  @Override
  public double getRotations() {
    return absolutePositionRotations;
  }

  @Override
  public void close() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }

  @Override
  public void setRange(double minimum, double maximum) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setRange'");
  }

  @Override
  public void setInverted(boolean isInverted) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
  }
}

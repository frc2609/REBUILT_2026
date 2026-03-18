package frc.robot.subsystems.io.encoder.impl;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.subsystems.io.encoder.AbsEncoderIO;
import frc.robot.Constants;

public class CANCoderIO implements AbsEncoderIO {
  private final CANcoder encoder;

  public CANCoderIO(int encoderPort) {
    this(encoderPort, 0.0, false);
  }

  /** Configures MagnetOffset and direction in one hardware apply — use for FusedCANcoder motors
   *  so the absolute position reads 0 at mechanical zero without any software reset at boot. */
  public CANCoderIO(int encoderPort, double magnetOffset, boolean inverted) {
    encoder = new CANcoder(encoderPort, Constants.CANBUS);

    MagnetSensorConfigs config = new MagnetSensorConfigs();
    encoder.getConfigurator().refresh(config);
    config.withAbsoluteSensorDiscontinuityPoint(1.0);
    config.withMagnetOffset(magnetOffset);
    config.withSensorDirection(inverted
        ? SensorDirectionValue.CounterClockwise_Positive
        : SensorDirectionValue.Clockwise_Positive);
    encoder.getConfigurator().apply(config);
  }

  @Override
  public double getRotations() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void close() {
    encoder.close();
  }

  @Override
  public void setRange(double minimum, double maximum) {
    return;
  }

  @Override
  public void setInverted(boolean isInverted) {
    CANcoderConfigurator configurator =  encoder.getConfigurator();
    SensorDirectionValue directionValue = isInverted ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
    MagnetSensorConfigs config = new MagnetSensorConfigs();
    
    configurator.refresh(config);
    config.withSensorDirection(directionValue);
    configurator.apply(config);
  }
}

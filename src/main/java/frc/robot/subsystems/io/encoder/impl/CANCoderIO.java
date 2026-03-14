package frc.robot.subsystems.io.encoder.impl;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;
import frc.robot.Constants;

public class CANCoderIO implements AbsEncoderIO {
  private final CANcoder encoder;

  public CANCoderIO(int encoderPort) {
    encoder = new CANcoder(encoderPort, Constants.CANBUS);
  }

  @Override
  public double getRotations() {
    return encoder.getPosition().getValueAsDouble();
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
    MagnetSensorConfigs config = new MagnetSensorConfigs();
    encoder.getConfigurator().refresh(config);
    config.SensorDirection = isInverted
        ? SensorDirectionValue.Clockwise_Positive
        : SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(config);
  }
}

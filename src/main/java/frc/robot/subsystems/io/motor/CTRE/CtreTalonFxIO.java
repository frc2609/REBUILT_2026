package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.Constants.NeutralMode;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class CtreTalonFxIO {
  private final Map<String, Consumer<Object>> setters = new HashMap<>();

  public final TalonFX motor;
  private int motorId;
  private TalonFXConfiguration config;

  public TalonFX followerMotor = null;
  private MotorAlignmentValue followerAligned;
  private int followerId = -1;
  public boolean hasFollower = false;

  public CtreTalonFxIO(Map<String, Object> cfg) {

    // Maps input config to TalonFx config but doesn't set it

    setters.put("leaderId", value -> this.motorId = (int) value);
    setters.put("kP", value -> this.config.Slot0.kP = (double) value);
    setters.put("kI", value -> this.config.Slot0.kI = (double) value);
    setters.put("kD", value -> this.config.Slot0.kD = (double) value);
    setters.put("kV", value -> this.config.Slot0.kV = (double) value);
    setters.put("kS", value -> this.config.Slot0.kS = (double) value);
    setters.put("kG", value -> this.config.Slot0.kG = (double) value);

    setters.put(
        "forwardLimitEnabled",
        value -> this.config.SoftwareLimitSwitch.ForwardSoftLimitEnable = (boolean) value);
    setters.put(
        "reverseLimitEnabled",
        value -> this.config.SoftwareLimitSwitch.ReverseSoftLimitEnable = (boolean) value);
    setters.put(
        "forwardLimitRotations",
        value -> this.config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (double) value);
    setters.put(
        "forwardLimitRotations",
        value -> this.config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (double) value);

    //
    // config.MotionMagic.MotionMagicCruiseVelocity = cfg.cruiseVelocityRps();
    // config.MotionMagic.MotionMagicAcceleration = cfg.accelerationRpsSq();
    // config.MotionMagic.MotionMagicJerk = cfg.jerk();

    // config.CurrentLimits.SupplyCurrentLimit = cfg.supplyCurrentLimit();
    // config.CurrentLimits.SupplyCurrentLimitEnable = cfg.supplyCurrentLimitEnabled();
    // config.CurrentLimits.StatorCurrentLimit = cfg.statorCurrentLimit();
    // config.CurrentLimits.StatorCurrentLimitEnable = cfg.statorCurrentLimitEnabled();

    setters.put(
        "inverted",
        value -> this.config.MotorOutput.withInverted(toPhoenixInverted((Boolean) value)));
    setters.put(
        "neutralMode",
        value ->
            this.config.MotorOutput.withNeutralMode(toPhoenixNeutralMode((NeutralMode) value)));

    setters.put("followerId", value -> this.followerId = (int) value);
    setters.put(
        "followerAlignment",
        value -> this.followerAligned = toPhoenixFollowerAlignment((Boolean) value));
    motor = new TalonFX(this.motorId, Constants.CANBUS);

    setConfiguration(cfg);
  }

  private static com.ctre.phoenix6.signals.NeutralModeValue toPhoenixNeutralMode(
      Constants.NeutralMode mode) {
    return mode == Constants.NeutralMode.BRAKE
        ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
        : com.ctre.phoenix6.signals.NeutralModeValue.Coast;
  }

  private static com.ctre.phoenix6.signals.InvertedValue toPhoenixInverted(boolean inverted) {
    return inverted
        ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
        : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
  }

  public void setConfiguration(Map<String, Object> fields) {
    if (fields == null) return;

    fields.forEach(
        (key, value) -> {
          Consumer<Object> setter = setters.get(key);
          if (setter != null) {
            setter.accept(value);
          } else {
            System.out.println("Unknown field: " + key);
          }
        });

    motor.getConfigurator().apply(config);

    if (followerId > -1) {
      followerMotor = new TalonFX(followerId, Constants.CANBUS);
      followerMotor.setControl(new Follower(this.motorId, followerAligned));
      followerMotor.getConfigurator().apply(config);
      hasFollower = true;
    }
  }

  private MotorAlignmentValue toPhoenixFollowerAlignment(Boolean followerAlignment) {
    return followerAlignment ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed;
  }
}

package frc.robot.subsystems.io.motor.CTRE;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;
import frc.robot.Constants.NeutralMode;

public class CtreTalonFxIO {
    private final Map<String, Consumer<Object>> setters = new HashMap<>();

    public final TalonFX motor;
    public int motorId;
    private TalonFXConfiguration config;

    public TalonFX followerMotor = null;
    private MotorAlignmentValue followerAligned;
    private int followerId = -1;
    public boolean hasFollower = false;

    public CtreTalonFxIO(Map<String, Object> cfg) {

        config = new TalonFXConfiguration();

        // Maps input config to TalonFx config but doesn't set it

        setters.put("motorId", value -> this.motorId = (int) value);
        setters.put("followerId", value -> this.followerId = (int) value);
        setters.put("followerAlignment",
            value -> this.followerAligned = toPhoenixFollowerAlignment((Boolean) value));

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
            "reverseLimitRotations",
            value -> this.config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (double) value);
        setters.put(
            "inverted",
            value -> this.config.MotorOutput.withInverted(toPhoenixInverted((Boolean) value)));
        setters.put(
            "neutralMode",
            value -> this.config.MotorOutput.withNeutralMode(toPhoenixNeutralMode((NeutralMode) value)));


        // NOT IMPLEMENTED
        
        // config.MotionMagic.MotionMagicCruiseVelocity = cfg.cruiseVelocityRps();
        // config.MotionMagic.MotionMagicAcceleration = cfg.accelerationRpsSq();
        // config.MotionMagic.MotionMagicJerk = cfg.jerk();

        // config.CurrentLimits.SupplyCurrentLimit = cfg.supplyCurrentLimit();
        // config.CurrentLimits.SupplyCurrentLimitEnable = cfg.supplyCurrentLimitEnabled();
        // config.CurrentLimits.StatorCurrentLimit = cfg.statorCurrentLimit();
        // config.CurrentLimits.StatorCurrentLimitEnable = cfg.statorCurrentLimitEnabled();

        setConfiguration(cfg);

        // MotorID and Follower settings are now available

        motor = new TalonFX(motorId, Constants.CANBUS);
        if (followerId > -1) {
            followerMotor = new TalonFX(followerId, Constants.CANBUS);
            followerMotor.setControl(new Follower(this.motorId, followerAligned));
            followerMotor.getConfigurator().apply(config);
            hasFollower = true;
        }

        applyConfiguration();
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

    private static MotorAlignmentValue toPhoenixFollowerAlignment(Boolean followerAlignment) {
        return followerAlignment ? MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed;
    }

    public void setConfiguration(Map<String, Object> fields) {
        if (fields == null) return;

        // Always apply incoming fields to the local config/state. 

        fields.forEach((key, value) -> {
            Consumer<Object> setter = setters.get(key);
            if (setter != null) {
                setter.accept(value);
            } else {
                throw new Error("Unknown field: " + key);
            }
        });
    }

    public void applyConfiguration()
    {
        //System.out.println("APPLIED KP="+config.Slot0.kP);
        motor.getConfigurator().apply(config);
        if (hasFollower) {
            followerMotor.getConfigurator().apply(config);
        }
    }
}

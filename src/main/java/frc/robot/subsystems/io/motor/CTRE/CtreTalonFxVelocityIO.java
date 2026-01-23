package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.io.motor.VelocityMotorIO;
import frc.robot.Constants;

public class CtreTalonFxVelocityIO implements VelocityMotorIO {
    private final TalonFX motor;
    private VelocityDutyCycle control = new VelocityDutyCycle(0).withSlot(0);
    private final TalonFX followerMotor;
    private final boolean hasFollower;
    private double targetRotationsPerSecond = 0.0;

    public CtreTalonFxVelocityIO(Constants.CtreTalonFxVelocityConfig cfg, int motorId) {
        this(cfg, motorId, -1);
    }

    public CtreTalonFxVelocityIO(Constants.CtreTalonFxVelocityConfig cfg, int motorId, int followerId) {
        TalonFXConfiguration config = toPhoenixConfig(cfg);

        motor = new TalonFX(motorId, Constants.CANBUS);
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(toPhoenixNeutralMode(cfg.neutralMode()));

        if (followerId != -1) {
            followerMotor = new TalonFX(followerId, Constants.CANBUS);
            followerMotor.getConfigurator().apply(config);
            followerMotor.setNeutralMode(toPhoenixNeutralMode(cfg.neutralMode()));
            hasFollower = true;
        } else {
            followerMotor = null;
            hasFollower = false;
        }
    }

    @Override
    public void setVelocityRps(double rotationsPerSecond) {
        targetRotationsPerSecond = rotationsPerSecond;
        control = control.withVelocity(rotationsPerSecond);
        motor.setControl(control);
        if (hasFollower) {
            followerMotor.setControl(control);
        }
    }

    @Override
    public double getVelocityRps() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public boolean isAtSpeed(double toleranceRps) {
        return Math.abs(targetRotationsPerSecond - getVelocityRps()) <= toleranceRps;
    }

    @Override
    public void stop() {
        motor.stopMotor();
        if (hasFollower) {
            followerMotor.stopMotor();
        }
        targetRotationsPerSecond = 0.0;
    }

    private static TalonFXConfiguration toPhoenixConfig(Constants.CtreTalonFxVelocityConfig cfg) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = cfg.kP();
        config.Slot0.kI = cfg.kI();
        config.Slot0.kD = cfg.kD();
        config.Slot0.kV = cfg.kV();
        config.Slot0.kS = cfg.kS();

        config.CurrentLimits.SupplyCurrentLimit = cfg.supplyCurrentLimit();
        config.CurrentLimits.SupplyCurrentLimitEnable = cfg.supplyCurrentLimitEnabled();
        config.CurrentLimits.StatorCurrentLimit = cfg.statorCurrentLimit();
        config.CurrentLimits.StatorCurrentLimitEnable = cfg.statorCurrentLimitEnabled();
        return config;
    }

    private static com.ctre.phoenix6.signals.NeutralModeValue toPhoenixNeutralMode(
            Constants.NeutralMode mode) {
        return mode == Constants.NeutralMode.BRAKE
                ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                : com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    }
}

package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.Constants;

public class CtreTalonFxPositionIO implements PositionMotorIO {
    private final TalonFX motor;
    private MotionMagicDutyCycle control = new MotionMagicDutyCycle(0).withSlot(0);
    private final double gearRatio;
    private final double encoderRatio;
    private final TalonFX followerMotor;
    private final boolean hasFollower;
    private double targetDegrees = 0.0;

    private final boolean forwardLimitEnabled;
    private final boolean reverseLimitEnabled;
    private final double forwardLimitRotations;
    private final double reverseLimitRotations;

    public CtreTalonFxPositionIO(
            Constants.CtreTalonFxPositionConfig cfg,
            int motorId,
            double gearRatio,
            double encoderRatio) {
        this(cfg, motorId, -1, gearRatio, encoderRatio);
    }

    public CtreTalonFxPositionIO(
            Constants.CtreTalonFxPositionConfig cfg,
            int motorId,
            int followerId,
            double gearRatio,
            double encoderRatio) {
        this.gearRatio = gearRatio;
        this.encoderRatio = encoderRatio;

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

        forwardLimitEnabled = config.SoftwareLimitSwitch.ForwardSoftLimitEnable;
        reverseLimitEnabled = config.SoftwareLimitSwitch.ReverseSoftLimitEnable;
        forwardLimitRotations = config.SoftwareLimitSwitch.ForwardSoftLimitThreshold;
        reverseLimitRotations = config.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
    }

    @Override
    public void setTargetPositionDegrees(double degrees) {
        double targetRotations = degreesToRotations(degrees);

        if (forwardLimitEnabled) {
            targetRotations = Math.min(targetRotations, forwardLimitRotations);
        }
        if (reverseLimitEnabled) {
            targetRotations = Math.max(targetRotations, reverseLimitRotations);
        }

        // Keep a vendor-agnostic setpoint for consistent "at position" semantics across implementations.
        targetDegrees = rotationsToDegrees(targetRotations);

        control = control.withPosition(targetRotations);
        motor.setControl(control);
        if (hasFollower) {
            followerMotor.setControl(control);
        }
    }

    @Override
    public double getPositionDegrees() {
        return rotationsToDegrees(motor.getPosition().getValueAsDouble());
    }

    @Override
    public boolean isAtPosition(double toleranceDegrees) {
        // Compare measured position to the last commanded setpoint in degrees (matches SparkMax + sim behavior).
        return Math.abs(targetDegrees - getPositionDegrees()) <= toleranceDegrees;
    }

    @Override
    public void resetToAbsolute(double absolutePositionRotations) {
        double offset = absolutePositionRotations;
        if (offset > 0.5) {
            offset -= 1.0;
        }
        offset *= encoderRatio;

        motor.setPosition(offset);
        if (hasFollower) {
            followerMotor.setPosition(offset);
        }
        targetDegrees = getPositionDegrees();
    }

    @Override
    public void stop() {
        motor.stopMotor();
        if (hasFollower) {
            followerMotor.stopMotor();
        }
        targetDegrees = getPositionDegrees();
    }

    private double degreesToRotations(double degrees) {
        return degrees / (360.0 / gearRatio);
    }

    private double rotationsToDegrees(double rotations) {
        return rotations * (360.0 / gearRatio);
    }

    private static TalonFXConfiguration toPhoenixConfig(Constants.CtreTalonFxPositionConfig cfg) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = cfg.kP();
        config.Slot0.kI = cfg.kI();
        config.Slot0.kD = cfg.kD();
        config.Slot0.kS = cfg.kS();
        config.Slot0.kV = cfg.kV();
        config.Slot0.kG = cfg.kG();

        config.MotionMagic.MotionMagicCruiseVelocity = cfg.cruiseVelocityRps();
        config.MotionMagic.MotionMagicAcceleration = cfg.accelerationRpsSq();
        config.MotionMagic.MotionMagicJerk = cfg.jerk();

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = cfg.forwardSoftLimitEnabled();
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = cfg.forwardSoftLimitRotations();
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = cfg.reverseSoftLimitEnabled();
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = cfg.reverseSoftLimitRotations();

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

package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.io.motor.PositionMotorIO;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.util.Conversions;

/**
 * Position motor IO using DynamicMotionMagicVoltage control.
 * Uses a trapezoidal motion profile with per-request dynamic velocity,
 * acceleration, and jerk limits. Output is in Voltage (not DutyCycle).
 *
 * Compared to the Expo variant: simpler profile, predictable deceleration,
 * lower overshoot under current limiting, no Pro license required.
 */
public class CtreTalonDynamicMotionMagicVoltageIO extends CtreTalonFxIO implements PositionMotorIO {
    private DynamicMotionMagicVoltage control;
    private final double gearRatio;

    public double targetDegrees = 0.0;

    private boolean forwardLimitEnabled;
    private boolean reverseLimitEnabled;
    private final double forwardLimitRotations;
    private final double reverseLimitRotations;

    private final LoggedNetworkNumber absRotationsLogged;
    public final LoggedNetworkNumber rotationsLogged;
    private final LoggedNetworkNumber positionErrorLogged;

    /**
     * @param cfg         Motor config map
     * @param gearRatio   Gear ratio for degree/rotation conversion
     * @param encoderRatio Encoder ratio (unused, kept for interface consistency)
     * @param maxVelocity Max cruise velocity in rotor rotations/sec
     * @param maxAccel    Max acceleration in rotor rotations/sec^2
     */
    public CtreTalonDynamicMotionMagicVoltageIO(
            Map<String, Object> cfg, double gearRatio, double encoderRatio,
            double maxVelocity, double maxAccel) {
        super(cfg);

        forwardLimitEnabled = (boolean) cfg.get("forwardLimitEnabled");
        reverseLimitEnabled = (boolean) cfg.get("reverseLimitEnabled");
        forwardLimitRotations = (double) cfg.get("forwardLimitRotations");
        reverseLimitRotations = (double) cfg.get("reverseLimitRotations");

        absRotationsLogged = new LoggedNetworkNumber(NTPath + "/AbsRotations");
        rotationsLogged = new LoggedNetworkNumber(NTPath + "/Rotations");
        positionErrorLogged = new LoggedNetworkNumber(NTPath + "/Error (deg)");

        this.gearRatio = gearRatio;

        control = new DynamicMotionMagicVoltage(0, maxVelocity, maxAccel)
            .withSlot(0);
    }

    public CtreTalonDynamicMotionMagicVoltageIO(
            Map<String, Object> cfg, double gearRatio,
            double maxVelocity, double maxAccel) {
        this(cfg, gearRatio, 1.0, maxVelocity, maxAccel);
    }

    @Override
    public void setTargetPositionDegrees(double degrees, double ff) {
        double targetRotations = Conversions.degreesToRotations(degrees, gearRatio);

        if (forwardLimitEnabled) {
            targetRotations = Math.min(targetRotations, forwardLimitRotations);
        }
        if (reverseLimitEnabled) {
            targetRotations = Math.max(targetRotations, reverseLimitRotations);
        }

        targetDegrees = Conversions.rotationsToDegrees(targetRotations, gearRatio);

        control = control.withPosition(targetRotations).withFeedForward(ff);
        motor.setControl(control);
    }

    @Override
    public void setTargetPositionDegrees(double degrees) {
        setTargetPositionDegrees(degrees, 0.0); // TODO: tune in ff based on gyro angular velocity
    }

    @Override
    public double getPositionDegrees() {
        return Conversions.rotationsToDegrees(motor.getPosition().getValueAsDouble(), gearRatio);
    }

    @Override
    public boolean isAtPosition(double toleranceDegrees) {
        return Math.abs(targetDegrees - getPositionDegrees()) <= toleranceDegrees;
    }

    @Override
    public void resetToAbsolute(double absRotations) {
        double motorRotations = absRotations * gearRatio;
        System.out.println(NTPath+": ENCODER RESET, absReading="+absRotations+", gear:"+gearRatio+" rotations:"+motorRotations);
        this.resetToRotations(motorRotations);
    }

    @Override
    public void resetToRotations(double rotations) {
        motor.setPosition(rotations);
        targetDegrees = getPositionDegrees(); // 0?
        if (hasFollower) {
            followerMotor.setPosition(rotations);
        }
    }

    @Override
    public void resetToZero() {
        motor.setPosition(0.0);
    }

    @Override
    public void logMotorPID() {
        measuredLogged.set(getPositionDegrees());
        rotationsLogged.set(motor.getPosition().getValueAsDouble());
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
        statorLogged.set(motor.getStatorCurrent().getValueAsDouble());
        positionErrorLogged.set(Conversions.rotationsToDegrees(motor.getClosedLoopError().getValueAsDouble(), gearRatio));
    }

    @Override
    public void logMotorPID(double absRotations) {
        measuredLogged.set(getPositionDegrees());
        absRotationsLogged.set(absRotations);
        rotationsLogged.set(motor.getPosition().getValueAsDouble());
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
        statorLogged.set(motor.getStatorCurrent().getValueAsDouble());
        positionErrorLogged.set(Conversions.rotationsToDegrees(motor.getClosedLoopError().getValueAsDouble(), gearRatio));
    }

    @Override
    public double getStatorCurrentAmps() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getRotorVelocityRps() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setCoastMode(boolean coast) {
        MotorOutputConfigs cfg = new MotorOutputConfigs();
        motor.getConfigurator().refresh(cfg);
        cfg.NeutralMode = coast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        motor.getConfigurator().apply(cfg);
    }

    @Override
    public void restoreConfiguredNeutralMode() {
        super.restoreConfiguredNeutralMode();
    }

    @Override
    public void stop() {
        motor.stopMotor();
        targetDegrees = getPositionDegrees();
    }
}

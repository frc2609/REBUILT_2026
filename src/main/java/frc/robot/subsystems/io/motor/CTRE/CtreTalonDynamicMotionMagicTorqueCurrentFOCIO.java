package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.io.motor.PositionMotorIO;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.util.Conversions;

/**
 * Position motor IO using DynamicMotionMagicTorqueCurrentFOC control (Phoenix Pro).
 * Uses a trapezoidal motion profile with direct torque current control.
 * Lowest peak current and best power efficiency among profiled controllers.
 *
 * Gains (kP, kD, kS) are in Amps (not Voltage). Convert from voltage gains:
 *   kP_A ≈ kP_V / R,  kD_A ≈ Kv * G / R (back-EMF equivalent damping)
 */
public class CtreTalonDynamicMotionMagicTorqueCurrentFOCIO extends CtreTalonFxIO implements PositionMotorIO {
    private DynamicMotionMagicTorqueCurrentFOC control;
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
     * @param cfg         Motor config map (kP/kD/kS in Amps for Slot0)
     * @param gearRatio   Gear ratio for degree/rotation conversion
     * @param encoderRatio Encoder ratio (unused, kept for interface consistency)
     * @param maxVelocity Max cruise velocity in rotor rotations/sec
     * @param maxAccel    Max acceleration in rotor rotations/sec^2
     */
    public CtreTalonDynamicMotionMagicTorqueCurrentFOCIO(
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

        control = new DynamicMotionMagicTorqueCurrentFOC(0, maxVelocity, maxAccel)
            .withSlot(0);
    }

    public CtreTalonDynamicMotionMagicTorqueCurrentFOCIO(
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
        setTargetPositionDegrees(degrees, 0.0);
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
    public void stop() {
        motor.stopMotor();
        targetDegrees = getPositionDegrees();
    }
}

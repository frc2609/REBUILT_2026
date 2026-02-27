package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import java.util.Map;
import frc.robot.util.Conversions;

public class CtreTalonFxPositionIO extends CtreTalonFxIO implements PositionMotorIO {
    private MotionMagicDutyCycle control = new MotionMagicDutyCycle(0).withSlot(0);
    private final double gearRatio;
    private final double encoderRatio;

    public double targetDegrees = 0.0;

    private final boolean forwardLimitEnabled;
    private final boolean reverseLimitEnabled;
    private final double forwardLimitRotations;
    private final double reverseLimitRotations;

    public CtreTalonFxPositionIO(Map<String, Object> cfg, double gearRatio, double encoderRatio) {
        super(cfg);

        forwardLimitEnabled = (boolean) cfg.get("forwardLimitEnabled");
        reverseLimitEnabled = (boolean) cfg.get("reverseLimitEnabled");
        forwardLimitRotations = (double) cfg.get("forwardLimitRotations");
        reverseLimitRotations = (double) cfg.get("reverseLimitRotations");

        this.gearRatio = gearRatio;
        this.encoderRatio = encoderRatio;
    }

    @Override
    public void setTargetPositionDegrees(double degrees) {
        double targetRotations = Conversions.degreesToRotations(degrees,gearRatio);

        if (forwardLimitEnabled) {
            targetRotations = Math.min(targetRotations, forwardLimitRotations);
        }
        if (reverseLimitEnabled) {
            targetRotations = Math.max(targetRotations, reverseLimitRotations);
        }

        // Keep a vendor-agnostic setpoint for consistent "at position" semantics across
        // implementations.
        targetDegrees = Conversions.rotationsToDegrees(targetRotations, gearRatio);

        control = control.withPosition(targetRotations);
        motor.setControl(control);
    }

    @Override
    public double getPositionDegrees() {
        return Conversions.rotationsToDegrees(motor.getPosition().getValueAsDouble(),gearRatio);
    }

    @Override
    public boolean isAtPosition(double toleranceDegrees) {
        // Compare measured position to the last commanded setpoint in degrees 
        // (matches SparkMax + sim behavior)
        return Math.abs(targetDegrees - getPositionDegrees()) <= toleranceDegrees;
    }

    @Override
    public void resetToAbsolute(double absolutePositionRotations) {
        double offset = absolutePositionRotations * encoderRatio;
        motor.setPosition(offset);
        targetDegrees = getPositionDegrees();

        // if (hasFollower) {
        //     followerMotor.setPosition(offset);
        // }
    }

    @Override
    public void stop() {
        motor.stopMotor();
        if (hasFollower) {
            followerMotor.stopMotor();
        }
        targetDegrees = getPositionDegrees();
    }
}

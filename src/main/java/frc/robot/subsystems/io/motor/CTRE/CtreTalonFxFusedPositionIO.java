package frc.robot.subsystems.io.motor.CTRE;


import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import frc.robot.subsystems.io.motor.PositionMotorIO;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.util.Conversions;

public class CtreTalonFxFusedPositionIO extends CtreTalonFxIO implements PositionMotorIO {
    private PositionDutyCycle control = new PositionDutyCycle(0).withSlot(0);
    private final DutyCycleOut zeroOutput = new DutyCycleOut(0);
    private final double gearRatio;
    private final double encoderRatio;
    // When FusedCANcoder is active, motor.getPosition() returns sensor (output-shaft) rotations,
    // not rotor rotations. positionRatio = 1.0 in that case; otherwise = gearRatio.
    private final double positionRatio;
    private final boolean usesRemoteFeedback;

    public double targetDegrees = 0.0;

    private boolean forwardLimitEnabled;
    private boolean reverseLimitEnabled;
    private final double forwardLimitRotations;
    private final double reverseLimitRotations;

    private final LoggedNetworkNumber absRotationsLogged;
    private final LoggedNetworkNumber absDegreesLogged;
    public final LoggedNetworkNumber rotationsLogged;

    public CtreTalonFxFusedPositionIO(Map<String, Object> cfg, double gearRatio, double encoderRatio) {
        super(cfg);

        forwardLimitEnabled = (boolean) cfg.get("forwardLimitEnabled");
        reverseLimitEnabled = (boolean) cfg.get("reverseLimitEnabled");
        forwardLimitRotations = (double) cfg.get("forwardLimitRotations");
        reverseLimitRotations = (double) cfg.get("reverseLimitRotations");

        absRotationsLogged = new LoggedNetworkNumber(NTPath+"/AbsRotations");
        absDegreesLogged = new LoggedNetworkNumber(NTPath+"/AbsDegrees");
        rotationsLogged = new LoggedNetworkNumber(NTPath+"/Rotations");

        this.gearRatio = gearRatio;
        this.encoderRatio = encoderRatio;
        this.usesRemoteFeedback = cfg.containsKey("feedbackSensorId");
        this.positionRatio = usesRemoteFeedback ? 1.0 : gearRatio;
    }

    public CtreTalonFxFusedPositionIO(Map<String, Object> cfg, double gearRatio) {
        this(cfg, gearRatio, 1.0);
    }

    @Override
    public void setTargetPositionDegrees(double degrees, double ff) {
        double targetRotations = Conversions.degreesToRotations(degrees, positionRatio);

        if (forwardLimitEnabled) {
            targetRotations = Math.min(targetRotations, forwardLimitRotations);
        }
        if (reverseLimitEnabled) {
            targetRotations = Math.max(targetRotations, reverseLimitRotations);
        }

        // Keep a vendor-agnostic setpoint for consistent "at position" semantics across
        // implementations.
        targetDegrees = Conversions.rotationsToDegrees(targetRotations, positionRatio);
        setpointLogged.set(targetDegrees);
        
        control = control.withPosition(targetRotations).withFeedForward(ff);
        motor.setControl(control);
        
    }

    @Override
    public void setTargetPositionDegrees(double degrees) {
        setTargetPositionDegrees(degrees, 0.0);
    }

    @Override
    public double getPositionDegrees() {
        return Conversions.rotationsToDegrees(motor.getPosition().getValueAsDouble(), positionRatio);
    }

    @Override
    public boolean isAtPosition(double toleranceDegrees) {
        // Compare measured position to the last commanded setpoint in degrees
        // (matches SparkMax + sim behavior)
        return Math.abs(targetDegrees - getPositionDegrees()) <= toleranceDegrees;
    }

    @Override
    public void resetToAbsolute(double absRotations) {
        // With FusedCANcoder the TalonFX needs at least one CANCoder CAN frame before
        // setPosition() can compute the correct persistent offset. Without this wait,
        // applyConfiguration() has just switched to FusedCANcoder but the first frame
        // hasn't arrived, so the offset is stored as 0 and position = raw CANCoder reading.
        // Not needed (and avoided) for rotor-encoder motors.
        if (usesRemoteFeedback) motor.getPosition().waitForUpdate(0.5);
        double motorRotations = absRotations * positionRatio;
        System.out.println(NTPath+": ENCODER RESET, absReading="+absRotations+", posRatio:"+positionRatio+" rotations:"+motorRotations);
        motor.setPosition(motorRotations);
        targetDegrees = getPositionDegrees();
        if (hasFollower) {
            followerMotor.setPosition(motorRotations);
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
        Logger.recordOutput(NTPath + "/ForwardLimitHit", motor.getFault_ForwardSoftLimit().getValue());
        Logger.recordOutput(NTPath + "/ReverseLimitHit", motor.getFault_ReverseSoftLimit().getValue());
        Logger.recordOutput(NTPath + "/ClosedLoopReference", motor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput(NTPath + "/ClosedLoopError", motor.getClosedLoopError().getValueAsDouble());
    }

    @Override
    public void logMotorPID(double absRotations) {
        measuredLogged.set(getPositionDegrees());
        absRotationsLogged.set(absRotations);
        double absRotationsNorm = absRotations > 0.5 ? absRotations - 1.0 : absRotations;
        absDegreesLogged.set(absRotationsNorm * 360.0);
        rotationsLogged.set(motor.getPosition().getValueAsDouble());
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(NTPath + "/ForwardLimitHit", motor.getFault_ForwardSoftLimit().getValue());
        Logger.recordOutput(NTPath + "/ReverseLimitHit", motor.getFault_ReverseSoftLimit().getValue());
        Logger.recordOutput(NTPath + "/ClosedLoopReference", motor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput(NTPath + "/ClosedLoopError", motor.getClosedLoopError().getValueAsDouble());
    }

    @Override
    public void setOutputZero() {
        motor.setControl(zeroOutput);
    }

    @Override
    public void stop() {
        motor.stopMotor();
        // if (hasFollower) {
        //     followerMotor.stopMotor();
        // }
        targetDegrees = getPositionDegrees();
    }
}

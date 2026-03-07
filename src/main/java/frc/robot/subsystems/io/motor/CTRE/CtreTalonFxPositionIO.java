package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;

import frc.robot.subsystems.io.motor.PositionMotorIO;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.util.Conversions;

public class CtreTalonFxPositionIO extends CtreTalonFxIO implements PositionMotorIO {
    private MotionMagicDutyCycle control = new MotionMagicDutyCycle(0).withSlot(0);
    private final double gearRatio;
    private final double encoderRatio;

    public double targetDegrees = 0.0;

    private boolean forwardLimitEnabled;
    private boolean reverseLimitEnabled;
    private final double forwardLimitRotations;
    private final double reverseLimitRotations;

    private final LoggedNetworkNumber absRotationsLogged;
    public final LoggedNetworkNumber rotationsLogged;

    public CtreTalonFxPositionIO(Map<String, Object> cfg, double gearRatio, double encoderRatio) {
        super(cfg);

        forwardLimitEnabled = (boolean) cfg.get("forwardLimitEnabled");
        reverseLimitEnabled = (boolean) cfg.get("reverseLimitEnabled");
        forwardLimitRotations = (double) cfg.get("forwardLimitRotations");
        reverseLimitRotations = (double) cfg.get("reverseLimitRotations");

        absRotationsLogged = new LoggedNetworkNumber(NTPath+"/AbsRotations");
        rotationsLogged = new LoggedNetworkNumber(NTPath+"/Rotations");

        this.gearRatio = gearRatio;
        this.encoderRatio = encoderRatio;
    }

    public CtreTalonFxPositionIO(Map<String, Object> cfg, double gearRatio) {
        this(cfg, gearRatio, 1.0);
    } 

    @Override
    public void setTargetPositionDegrees(double degrees) {
        double targetRotations = Conversions.degreesToRotations(degrees, gearRatio);

        if (forwardLimitEnabled) {
            targetRotations = Math.min(targetRotations, forwardLimitRotations);
        }
        if (reverseLimitEnabled) {
            targetRotations = Math.max(targetRotations, reverseLimitRotations);
        }

        // Keep a vendor-agnostic setpoint for consistent "at position" semantics across
        // implementations.
        targetDegrees = Conversions.rotationsToDegrees(targetRotations, gearRatio);
        System.out.println("POSITION COMMAND: "+degrees+" -> "+targetRotations);

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
        System.out.println("ENCODER RESET: "+absolutePositionRotations);
        motor.setPosition(absolutePositionRotations);
        targetDegrees = getPositionDegrees(); // 0?
        if (hasFollower) {
            followerMotor.setPosition(absolutePositionRotations);
        }
    }

    @Override
    public void logMotorPID() {
        measuredLogged.set(getPositionDegrees());
        Logger.recordOutput(NTPath+"/Rotations", motor.getPosition().getValueAsDouble());
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void logMotorPID(double absRotations) {
        measuredLogged.set(getPositionDegrees());
        absRotationsLogged.set(absRotations);
        rotationsLogged.set(motor.getPosition().getValueAsDouble());
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
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

package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.subsystems.io.motor.PositionMotorIO;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.util.Conversions;

public class CtreTalonFxPositionIO extends CtreTalonFxIO implements PositionMotorIO {
    //private MotionMagicDutyCycle control = new MotionMagicDutyCycle(0).withSlot(0);
    private PositionDutyCycle control = new PositionDutyCycle(0).withSlot(0).withEnableFOC(true);
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
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
        //System.out.println("POSITION COMMAND: "+degrees+" -> "+targetRotations);

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
    public void resetToAbsolute(double absRotations) {
        double motorRotations = absRotations * gearRatio;
        System.out.println("ENCODER RESET, rotations="+motorRotations);
        motor.setPosition(motorRotations);
        targetDegrees = getPositionDegrees(); // 0?
        if (hasFollower) {
            followerMotor.setPosition(motorRotations);
        }
    }

    @Override
    public void logMotorPID() {
        measuredLogged.set(getPositionDegrees());
        rotationsLogged.set(motor.getPosition().getValueAsDouble());
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void logMotorPID(double absRotations) {
        measuredLogged.set(getPositionDegrees());
        absRotationsLogged.set(absRotations);
        rotationsLogged.set(motor.getPosition().getValueAsDouble());
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
    }


    /**
     * Drives the motor with raw voltage for SysID characterization.
     * Respects forward and reverse soft limits (if enabled) in addition to
     * the hardware-enforced limits already configured in TalonFXConfiguration.
     */
    public void runVolts(double volts) {
        double currentRotations = motor.getPosition().getValueAsDouble();

        if (forwardLimitEnabled && volts > 0 && currentRotations >= forwardLimitRotations) {
            volts = 0;
        }
        if (reverseLimitEnabled && volts < 0 && currentRotations <= reverseLimitRotations) {
            volts = 0;
        }

        motor.setControl(voltageOut.withOutput(volts));
    }

    /**
     * Drives the motor with raw voltage for SysID characterization with predictive limit stopping.
     * In addition to the hard limit checks in {@link #runVolts}, this predicts the position
     * 2 robot loops ahead (40 ms) using current velocity and zeroes the voltage if the
     * mechanism would reach a soft-limit boundary, giving the loop time to react before
     * the limit is actually breached.
     */
    public void runVoltsSysid(double volts) {
        double currentRotations = motor.getPosition().getValueAsDouble();
        double velocityRPS = motor.getVelocity().getValueAsDouble();
        // Predict position 2 x 20 ms loops into the future
        double predictedRotations = currentRotations + velocityRPS * 0.040;

        if (forwardLimitEnabled && volts > 0
                && (currentRotations >= forwardLimitRotations || predictedRotations >= forwardLimitRotations)) {
            volts = 0;
        }
        if (reverseLimitEnabled && volts < 0
                && (currentRotations <= reverseLimitRotations || predictedRotations <= reverseLimitRotations)) {
            volts = 0;
        }

        motor.setControl(voltageOut.withOutput(volts));
    }

    /**
     * Creates a SysIdRoutine for this motor.
     *
     * <p>Bind the returned quasistatic/dynamic commands to buttons in RobotContainer.
     * Example usage in a subsystem:
     * <pre>
     *   public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
     *       return io.getSysIdRoutine(this).quasistatic(dir);
     *   }
     *   public Command sysIdDynamic(SysIdRoutine.Direction dir) {
     *       return io.getSysIdRoutine(this).dynamic(dir);
     *   }
     * </pre>
     *
     * @param subsystem the subsystem that owns this motor (used for command requirements)
     */
    public SysIdRoutine getSysIdRoutine(SubsystemBase subsystem) {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null,
                (state) -> Logger.recordOutput(NTPath + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runVoltsSysid(voltage.in(Volts)),
                (log) -> log.motor(NTPath)
                    .voltage(Volts.of(motor.getMotorVoltage().getValueAsDouble()))
                    .angularPosition(Rotations.of(motor.getPosition().getValueAsDouble()))
                    .angularVelocity(RotationsPerSecond.of(motor.getVelocity().getValueAsDouble())),
                subsystem));
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

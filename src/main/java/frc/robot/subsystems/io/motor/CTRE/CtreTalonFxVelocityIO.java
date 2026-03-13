package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.io.motor.VelocityMotorIO;
import java.util.Map;

public class CtreTalonFxVelocityIO extends CtreTalonFxIO implements VelocityMotorIO {

    private VelocityDutyCycle control;
    //private TorqueCurrentFOC torqueControl;
    public double setpointRps = 0.0;

    public CtreTalonFxVelocityIO(Map<String, Object> cfg) {
        super(cfg);
        control = new VelocityDutyCycle(0.0);
        //torqueControl = new TorqueCurrentFOC(0.0);
    }

    @Override
    public void setVelocityRps(double velocity) {
        if (setpointRps != velocity) {
            setpointRps = velocity;
            control = control.withVelocity(setpointRps);
            motor.setControl(control);
        }
    }

    @Override
    public void set(double percent) {
        motor.set(percent);
    }

    @Override
    public double getVelocityRps() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getSetpointRPM() {
        double input = setpointLogged.get();
        return input;
    }

    @Override
    public boolean isAtSpeed(double toleranceRps) {
        return Math.abs(setpointRps - getVelocityRps()) <= toleranceRps;
    }

    @Override
    public void logMotorPID() {
        measuredLogged.set(getVelocityRps()*60.0);
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void stop() {
        setpointRps = 0.0;
        motor.stopMotor();
    }

    @Override
    public SysIdRoutine getSysIdRoutine(SubsystemBase subsystem) {
        // Delegate to the base CTRE IO implementation, which already wires SysId correctly.
        return super.getSysIdRoutine(subsystem);
    }

}

package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

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
            // if (hasFollower)
            // {
            //     followerMotor.setControl(control);
            // }
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
        // setpointLogged.set(input);
        return input;
    }

    @Override
    public boolean isAtSpeed(double toleranceRps) {
        return Math.abs(setpointRps - getVelocityRps()) <= toleranceRps;
    }

    @Override
    public void logMotorPID() {
        measuredLogged.set(getVelocityRps()*60.0);
        //setpointLogged.set(setpointRps*60.0);
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void stop() {
        setpointRps = 0.0;
        motor.stopMotor();
        // if (hasFollower) {
        //     followerMotor.stopMotor();
        // }
    }
}

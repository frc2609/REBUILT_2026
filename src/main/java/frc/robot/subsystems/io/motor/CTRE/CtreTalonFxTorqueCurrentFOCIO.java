package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.math.controller.BangBangController;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

import java.util.Map;

public class CtreTalonFxTorqueCurrentFOCIO extends CtreTalonFxIO implements VelocityMotorIO {

    //public BangBangController bangBangController = new BangBangController();
    public VelocityTorqueCurrentFOC control = new VelocityTorqueCurrentFOC(0.0);
    double setpointRPS = 0.0;

    public CtreTalonFxTorqueCurrentFOCIO(Map<String, Object> cfg) {
        super(cfg);
    }

    @Override
    public double getVelocityRps() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getStatorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVelocityRps(double setpoint) {
        //bangBangController.setSetpoint(setpoint);
        control = control.withVelocity(setpoint);
        // control = control.withOutput(
        //     80.0*bangBangController.calculate(getVelocityRps())
        //     + config.Slot0.kV * setpoint
        // );
        motor.setControl(control);
    }

    @Override
    public void logMotorPID() {
        measuredLogged.set(getVelocityRps()*60.0);
        setpointLogged.set(getSetpointRPM());
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
        statorLogged.set(getStatorCurrent());
    }

    @Override
    public double getSetpointRPM() {
        double input = setpointLogged.get();
        return input;
    }

    @Override
    public boolean isAtSpeed(double toleranceRps) {
        return Math.abs(setpointRPS - getVelocityRps()) <= toleranceRps;
    }

    @Override
    public void setIsUnjamSlot(boolean isUnjam){
        return;
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setCoastMode(boolean coast) {
        super.setCoastMode(coast);
    }

    @Override
    public void restoreConfiguredNeutralMode() {
        super.restoreConfiguredNeutralMode();
    }
}

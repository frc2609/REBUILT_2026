package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import edu.wpi.first.math.controller.BangBangController;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

import java.util.Map;

public class CtreTalonFxTorqueCurrentFOCIO extends CtreTalonFxIO implements VelocityMotorIO {

    public BangBangController bangBangController = new BangBangController();
    public TorqueCurrentFOC control = new TorqueCurrentFOC(0);

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
        bangBangController.setSetpoint(setpoint);
        control = control.withOutput(
            80.0*bangBangController.calculate(getVelocityRps())
            + config.Slot0.kV * setpoint
        );
        motor.setControl(control);
    }

    @Override
    public void logMotorPID() {
        measuredLogged.set(getVelocityRps()*60.0);
        setpointLogged.set(bangBangController.getSetpoint()*60.0);
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
        statorLogged.set(getStatorCurrent());
    }

    @Override
    public double getSetpointRPM() {
        return bangBangController.getSetpoint();
    }

    @Override
    public boolean isAtSpeed(double toleranceRps) {
        //bangBangController.setTolerance(toleranceRps);
        return bangBangController.atSetpoint();
    }

    @Override
    public void setIsUnjamSlot(boolean isUnjam){
        return;
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}

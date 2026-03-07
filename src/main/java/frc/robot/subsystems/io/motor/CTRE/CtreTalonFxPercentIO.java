package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.subsystems.io.motor.PercentMotorIO;
import java.util.Map;

public class CtreTalonFxPercentIO extends CtreTalonFxIO implements PercentMotorIO {

    private DutyCycleOut control = new DutyCycleOut(0);
    public double percent = 0.0;

    public CtreTalonFxPercentIO(Map<String, Object> cfg) {
        super(cfg);
    }

    @Override
    public void setPercent(double percent) {
        this.percent = Math.max(-1.0, Math.min(1.0, percent));
        control = control.withOutput(this.percent);
        motor.setControl(control);
        // if (hasFollower) {
        // followerMotor.setControl(control);
        // }
    }

    @Override
    public void logMotorPID() {
        measuredLogged.set(motor.getVelocity().getValueAsDouble()*60.0);
        setpointLogged.set(percent);
        voltageLogged.set(motor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void stop() {
        motor.stopMotor();
        // if (hasFollower) {
        //     followerMotor.stopMotor();
        // }
    }
}

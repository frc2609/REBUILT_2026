package frc.robot.subsystems.io.motor.CTRE;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.io.motor.PercentMotorIO;
import frc.robot.Constants;

public class CtreTalonFxPercentIO implements PercentMotorIO {
    private final TalonFX motor;
    private DutyCycleOut control = new DutyCycleOut(0);
    private final TalonFX followerMotor;
    private final boolean hasFollower;

    public CtreTalonFxPercentIO(Constants.CtreTalonFxPercentConfig cfg, int motorId) {
        this(cfg, motorId, -1);
    }

    public CtreTalonFxPercentIO(Constants.CtreTalonFxPercentConfig cfg, int motorId, int followerId) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor = new TalonFX(motorId, Constants.CANBUS);
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(toPhoenixNeutralMode(cfg.neutralMode()));

        if (followerId != -1) {
            followerMotor = new TalonFX(followerId, Constants.CANBUS);
            followerMotor.getConfigurator().apply(config);
            followerMotor.setNeutralMode(toPhoenixNeutralMode(cfg.neutralMode()));
            hasFollower = true;
        } else {
            followerMotor = null;
            hasFollower = false;
        }
    }

    @Override
    public void setPercent(double percent) {
        percent = Math.max(-1.0, Math.min(1.0, percent));
        control = control.withOutput(percent);
        motor.setControl(control);
        if (hasFollower) {
            followerMotor.setControl(control);
        }
    }

    @Override
    public void stop() {
        motor.stopMotor();
        if (hasFollower) {
            followerMotor.stopMotor();
        }
    }

    private static com.ctre.phoenix6.signals.NeutralModeValue toPhoenixNeutralMode(
            Constants.NeutralMode mode) {
        return mode == Constants.NeutralMode.BRAKE
                ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                : com.ctre.phoenix6.signals.NeutralModeValue.Coast;
    }
}

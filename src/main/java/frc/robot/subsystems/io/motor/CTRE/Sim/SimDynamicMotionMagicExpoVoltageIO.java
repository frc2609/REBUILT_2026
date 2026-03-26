package frc.robot.subsystems.io.motor.CTRE.Sim;

import java.util.Map;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.SimMotor;
import frc.robot.subsystems.io.motor.CTRE.CtreTalonDynamicMotionMagicExpoVoltageIO;

public class SimDynamicMotionMagicExpoVoltageIO extends CtreTalonDynamicMotionMagicExpoVoltageIO {
    private DCMotorSim motorSim;
    private DCMotor gearbox;
    private TalonFXSimState talonFXSim;
    private Notifier simNotifier;
    private double motorVoltage;

    private double kGearRatio;
    private double kSimDelta;

    public SimDynamicMotionMagicExpoVoltageIO(
        Map<String, Object> cfg, double inertia, double gearRatio, double encoderRatio,
        SimMotor simMotor, double simDelta,
        double maxVelocity, double maxAccel
    ) {
        super(cfg, gearRatio, encoderRatio, maxVelocity, maxAccel);

        kGearRatio = gearRatio;
        kSimDelta = simDelta;
        MotorType controllerType;

        switch (simMotor) {
            case KRAKEN_X60:
                gearbox = DCMotor.getKrakenX60Foc(1);
                controllerType = MotorType.KrakenX60;
                break;
            case KRAKEN_X44:
                gearbox = DCMotor.getKrakenX44Foc(1);
                controllerType = MotorType.KrakenX44;
                break;
            default:
                throw new Error("Unknown Sim Motor Type id=" + motorId);
        }

        motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearbox, inertia, gearRatio),
            gearbox
        );

        talonFXSim = motor.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType(controllerType);

        simNotifier = new Notifier(this::updateSim);
        simNotifier.startPeriodic(kSimDelta);
    }

    public SimDynamicMotionMagicExpoVoltageIO(
        Map<String, Object> cfg, double inertia, double gearRatio,
        SimMotor simMotor, double simDelta,
        double maxVelocity, double maxAccel
    ) {
        this(cfg, inertia, gearRatio, 1.0, simMotor, simDelta, maxVelocity, maxAccel);
    }

    public void updateSim() {
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorVoltage = talonFXSim.getMotorVoltage();

        motorSim.setInputVoltage(motorVoltage);
        motorSim.update(kSimDelta);

        talonFXSim.setRawRotorPosition(motorSim.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(motorSim.getAngularVelocity().times(kGearRatio));
    }

    @Override
    public void logMotorPID() {
        measuredLogged.set(getPositionDegrees());
        rotationsLogged.set(motorSim.getAngularPositionRotations());
        voltageLogged.set(talonFXSim.getMotorVoltage());
        statorLogged.set(talonFXSim.getTorqueCurrent());
    }

    @Override
    public void logMotorPID(double rotations) {
        logMotorPID();
    }

    @Override
    public double getStatorCurrentAmps() {
        return talonFXSim.getTorqueCurrent();
    }

    @Override
    public void setCoastMode(boolean coast) {
        // no-op in sim
    }

    @Override
    public void stop() {
        // no-op in sim
    }
}

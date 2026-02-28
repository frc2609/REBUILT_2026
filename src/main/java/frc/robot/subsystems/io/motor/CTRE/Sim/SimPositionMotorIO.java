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
import frc.robot.subsystems.io.motor.CTRE.CtreTalonFxPositionIO;

public class SimPositionMotorIO extends CtreTalonFxPositionIO {
    private DCMotorSim motorSim;
    private DCMotor gearbox;
    private TalonFXSimState talonFXSim;
    private Notifier simNotifier;
    private double motorVoltage;

    private double kGearRatio;
    private double kSimDelta;

    public SimPositionMotorIO(
        Map<String, Object> cfg, double inertia, double gearRatio, double encoderRatio, 
        SimMotor simMotor, double simDelta
    ) {
        super(cfg, gearRatio, encoderRatio); // create the motor from CTRE implementation

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
                throw new Error("Unknown Sim Motor Type id="+motorId);
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

    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/simulation/simulation-intro.html

    public void updateSim() {
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        motorVoltage = talonFXSim.getMotorVoltage();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        motorSim.setInputVoltage(motorVoltage);
        motorSim.update(kSimDelta);

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(motorSim.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(motorSim.getAngularVelocity().times(kGearRatio));
    }

    @Override
    public double getPositionDegrees() {
        return motorSim.getAngularPositionRad() * (180.0 / Math.PI);
    }

    @Override
    public void logMotorPID() {
        measuredLogged.set(getPositionDegrees());
        setpointLogged.set(targetDegrees);
        voltageLogged.set(talonFXSim.getMotorVoltage());
    }

    @Override
    public void stop() {
        // No-op for simple sim.
    }
}

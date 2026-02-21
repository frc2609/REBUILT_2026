package frc.robot.subsystems.io.motor.Sim;

import java.util.Map;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SimMotor;
import frc.robot.subsystems.io.motor.CTRE.CtreTalonFxPositionIO;

public class SimPositionMotorIO extends CtreTalonFxPositionIO {
    private DCMotorSim motorSim;
    private DCMotor gearbox;
    private TalonFXSimState talonFXSim;
    private Notifier simNotifier;

    private double kGearRatio;
    private double kSimDelta;
    private int id;

    public SimPositionMotorIO(
        Map<String, Object> cfg, double inertia, double gearRatio, double encoderRatio, 
        SimMotor simMotor, double simDelta
    ) {
        super(cfg, gearRatio, encoderRatio); // create the motor from CTRE implementation
        
        id = (int) cfg.get("motorId");
        kGearRatio = gearRatio;
        kSimDelta = simDelta;   

        gearbox = DCMotor.getKrakenX60Foc(1); // NOTE: currently this is tailored to Kraken X60s
        motorSim =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, inertia, gearRatio), gearbox);

        talonFXSim = super.motor.getSimState();
        talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
        talonFXSim.setMotorType((
            simMotor == SimMotor.KRAKEN_X60
                ? MotorType.KrakenX60
                : MotorType.KrakenX44
        ));

        simNotifier = new Notifier(this::updateSim);
        simNotifier.startPeriodic(kSimDelta);
    }

    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/simulation/simulation-intro.html

    public void updateSim() {
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        var motorVoltage = talonFXSim.getMotorVoltage();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        motorSim.setInputVoltage(motorVoltage);
        motorSim.update(kSimDelta);

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(motorSim.getAngularPosition().times(kGearRatio));
        talonFXSim.setRotorVelocity(motorSim.getAngularVelocity().times(kGearRatio));

        SmartDashboard.putNumber("Position/" + id + " Measure (deg)", getPositionDegrees());
        SmartDashboard.putNumber("Position/" + id + " Setpoint (deg)", super.targetDegrees);
        SmartDashboard.putNumber("Position/" + id + " PIDOutput (V)", motorVoltage);
    }

    @Override
    public double getPositionDegrees() {
        return motorSim.getAngularPositionRad() * (180.0 / Math.PI);
    }

    @Override
    public void stop() {
        // No-op for simple sim.
    }
}

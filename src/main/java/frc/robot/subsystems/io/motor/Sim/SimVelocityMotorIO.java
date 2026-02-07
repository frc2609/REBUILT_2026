package frc.robot.subsystems.io.motor.Sim;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.io.motor.CTRE.CtreTalonFxVelocityIO;
import java.util.Map;

public class SimVelocityMotorIO extends CtreTalonFxVelocityIO {
  private DCMotorSim motorSim;
  private DCMotor gearbox;
  private TalonFXSimState talonFXSim;
  private Notifier simNotifier;

  private double kGearRatio;
  private double kSimDelta;
  private int id;

  public SimVelocityMotorIO(
      Map<String, Object> cfg, double inertia, double gearRatio, double simDelta) {
    super(cfg); // create the motor from CTRE implementation
    kGearRatio = gearRatio;
    id = (int) cfg.get("motorId");
    kSimDelta = simDelta;

    gearbox = DCMotor.getKrakenX60Foc(1); // NOTE: currently this is tailored to Kraken X60s
    motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, inertia, gearRatio), gearbox);

    talonFXSim = super.motor.getSimState();
    talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

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

    SmartDashboard.putNumber("Shooter/" + id + " Measure (RPM)", motorSim.getAngularVelocityRPM());
    SmartDashboard.putNumber("Shooter/" + id + " Setpoint (RPM)", super.setpointRps * 60.0);
    SmartDashboard.putNumber("Shooter/" + id + " PIDOutput (V)", motorVoltage);
  }

  @Override
  public double getVelocityRps() {
    return motorSim.getAngularVelocityRPM() / 60.0;
  }

  @Override
  public void stop() {
    setVelocityRps(0.0);
  }
}

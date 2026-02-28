package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

// Import Redux Robotics classes
import com.reduxrobotics.sensors.canandcolor.*;

/** Shooter Subsystem using velocity control (rotations per second). */
public class ShooterSubsystem extends SubsystemBase {
  private final VelocityMotorIO shooterMotor;
  private final Canandcolor colorSensor;

  public ShooterSubsystem(VelocityMotorIO shooterMotor) {
    this.shooterMotor = shooterMotor;
    // Initialize the sensor, we should probably change the ID I'm not so sure about this
    this.colorSensor = new Canandcolor(0);
  }

  public void setSpeed(double rotationsPerSecond) {
    shooterMotor.setVelocityRps(rotationsPerSecond);
  }

  public boolean detectColor() {
      double proximity = colorSensor.getProximity();

      double hue = colorSensor.getHSVHue();
      double saturation = colorSensor.getHSVSaturation();

      boolean isYellow = (hue > 0.11 && hue < 0.18);
      
      boolean isVivid = (saturation > 0.4); 

      boolean isClose = (proximity > 0.5);

      return isYellow && isVivid && isClose;
  }

  public boolean isAtSpeed(double tolerance) {
    return shooterMotor.isAtSpeed(tolerance);
  }

  public boolean isReady() {
    return isAtSpeed(2.0);
  }

  public void stop() {
    shooterMotor.stop();
  }
}

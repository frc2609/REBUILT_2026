package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.VelocityMotorIO;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class ShooterSubsystem extends SubsystemBase {
  private final VelocityMotorIO shooterMotor;
  private final PositionMotorIO turretpositionMotor;
  private final PositionMotorIO hoodPositionMotor;
  private final AbsEncoderIO turretEncoder;


  public ShooterSubsystem(VelocityMotorIO shooterMotor, PositionMotorIO turretpositionMotor, PositionMotorIO hoodPositionMotor,AbsEncoderIO turrretEncoder) {
    this.shooterMotor = shooterMotor;
    this.turretpositionMotor = turretpositionMotor;
    this.hoodPositionMotor = hoodPositionMotor;
    this.turretEncoder = turrretEncoder;
  }

  public void setSpeed(double rotationsPerSecond) {
    shooterMotor.setVelocityRps(rotationsPerSecond);
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


  // public void setturretPosition(double degrees){
  //   turretEncoder.;
  // }

  public boolean turretisAtPosition(double toleranceDegrees) {
    return turretpositionMotor.isAtPosition(toleranceDegrees);
  }
}

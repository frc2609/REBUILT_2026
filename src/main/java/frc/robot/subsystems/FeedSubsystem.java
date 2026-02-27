package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class FeedSubsystem extends SubsystemBase {
    private final VelocityMotorIO agitatorMotor;
    private final VelocityMotorIO feedMotor;

    public FeedSubsystem(VelocityMotorIO agitatorMotor, VelocityMotorIO feedMotor) {
        this.agitatorMotor = agitatorMotor;
        this.feedMotor = feedMotor;
    }

    public void setAgitatorSpeed(double rotationsPerSecond) {
        agitatorMotor.setVelocityRps(rotationsPerSecond);
    }

    public boolean agitatorIsAtSpeed(double toleranceDegrees) {
        return agitatorMotor.isAtSpeed(toleranceDegrees);
    }
    
    public void setFeedSpeed(double rotationsPerSecond) {
        feedMotor.setVelocityRps(rotationsPerSecond);
    }

    public boolean feedIsAtSpeed(double toleranceDegrees) {
        return feedMotor.isAtSpeed(toleranceDegrees);
    }

    public void stop() {
        agitatorMotor.stop();
        feedMotor.stop();
    }
}

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

    public void setSetpoints(double agitatorRPM, double feedRPM) {
        agitatorMotor.setSetpoint(agitatorRPM);
        feedMotor.setSetpoint(feedRPM);
    }

    public void setAgitatorSpeed() {
        agitatorMotor.setVelocityRps(agitatorMotor.getSetpointRPM()/60.0);
    }
    public void setFeedSpeed() {
        feedMotor.setVelocityRps(feedMotor.getSetpointRPM()/60.0);
    }
    public void setAgitatorSpeed(double rotationsPerSecond) {
        agitatorMotor.setVelocityRps(rotationsPerSecond);
    }
    public void setFeedSpeed(double rotationsPerSecond) {
        feedMotor.setVelocityRps(rotationsPerSecond);
    }

    public boolean agitatorIsAtSpeed(double toleranceDegrees) {
        return agitatorMotor.isAtSpeed(toleranceDegrees);
    }

    public boolean feedIsAtSpeed(double toleranceDegrees) {
        return feedMotor.isAtSpeed(toleranceDegrees);
    }

    public void stop() {
        agitatorMotor.stop();
        feedMotor.stop();
    }

    @Override
    public void periodic()
    {
        agitatorMotor.logMotorPID();
        feedMotor.logMotorPID();
        agitatorMotor.updateFromTunables();
        feedMotor.updateFromTunables();
    }
}

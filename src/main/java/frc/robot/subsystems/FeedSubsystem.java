package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class FeedSubsystem extends SubsystemBase {
    private final VelocityMotorIO agitatorMotor;
    private final VelocityMotorIO feedMotor;
    private final Timer unjamTimer;
    private boolean unjamming = false;

    public FeedSubsystem(VelocityMotorIO agitatorMotor, VelocityMotorIO feedMotor) {
        this.agitatorMotor = agitatorMotor;
        this.feedMotor = feedMotor;
        unjamTimer = new Timer();
        unjamTimer.start();
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

    public boolean isFeedRunning() {
        return feedMotor.getVelocityRps() > 0.1;
    }

    public boolean agitatorIsAtSpeed(double toleranceDegrees) {
        return agitatorMotor.isAtSpeed(toleranceDegrees);
    }

    public boolean feedIsAtSpeed(double toleranceDegrees) {
        return feedMotor.isAtSpeed(toleranceDegrees);
    }

    public void spindexerSetUnjam(boolean unjamming) {
        agitatorMotor.setIsUnjamSlot(unjamming);
        Logger.recordOutput("Feed Unjamming", unjamming);
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

        // if (unjamming) {
        //     if (unjamTimer.get() > Constants.Agitator.UNJAM_TIME) {
        //         unjamTimer.reset();
        //         unjamming = false;
        //         agitatorMotor.setIsUnjamSlot(false);
        //         Logger.recordOutput("Feed Unjamming", false);
        //     }
        // } else if (
        //     agitatorMotor.getStatorCurrent() > Constants.Agitator.JAM_CURRENT &&
        //     unjamTimer.get() > Constants.Agitator.UNJAM_TIME // debounce
        // ){
        //     unjamTimer.reset();
        //     unjamming = true;
        //     agitatorMotor.setIsUnjamSlot(true);
        //     Logger.recordOutput("Feed Unjamming", true);
        // }
    }
}

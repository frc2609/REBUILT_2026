package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

    /** SysId characterization for the agitator motor. Hold the button for the duration of the test. */
    public Command sysIdAgitatorCommand(frc.robot.commands.SysIdCommand.Mode mode, SysIdRoutine.Direction direction) {
        return agitatorMotor.createSysIdCommand(this, mode, direction);
    }

    /** SysId characterization for the feed motor. Hold the button for the duration of the test. */
    public Command sysIdFeedCommand(frc.robot.commands.SysIdCommand.Mode mode, SysIdRoutine.Direction direction) {
        return feedMotor.createSysIdCommand(this, mode, direction);
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

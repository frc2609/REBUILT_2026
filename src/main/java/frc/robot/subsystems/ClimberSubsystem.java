package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;
import frc.robot.subsystems.io.motor.PositionMotorIO;

/** Intake Subsystem using velocity control (rotations per second). */
public class ClimberSubsystem extends SubsystemBase {
    private PositionMotorIO climberMotor;
    private AbsEncoderIO climberEncoder;

    public ClimberSubsystem(AbsEncoderIO climberEncoder, PositionMotorIO climberMotor) {
        this.climberEncoder = climberEncoder;
        this.climberMotor = climberMotor;
    }

    public void setToPosition(double degrees) {
        climberMotor.setTargetPositionDegrees(degrees);
    }

    public void resetPositionToAbsolute() {
        climberMotor.resetToAbsolute(climberEncoder.getRotations());
    }

    public boolean isAtPosition(double tolerance) {
        return climberMotor.isAtPosition(tolerance);
    }

    public void stop() {
        climberMotor.stop();
    }

    @Override
    public void periodic()
    {
        climberMotor.logMotorPID();
        climberMotor.updateFromTunables();
    }
}

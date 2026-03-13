package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

    /** SysId characterization for the climber. Hold the button for the duration of the test. */
    public Command sysIdCommand(frc.robot.commands.SysIdCommand.Mode mode, SysIdRoutine.Direction direction) {
        return climberMotor.createSysIdCommand(this, mode, direction);
    }

    @Override
    public void periodic()
    {
        climberMotor.logMotorPID(climberEncoder.getRotations());
        climberMotor.updateFromTunables();
    }
}

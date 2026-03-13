package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class TurretSubsystem extends SubsystemBase {
    private final PositionMotorIO aimMotor;
    private final PositionMotorIO hoodMotor;
    private final AbsEncoderIO aimEncoder;

    public TurretSubsystem(
        PositionMotorIO aimMotor, PositionMotorIO hoodMotor, 
        AbsEncoderIO aimEncoder
    ) {
        this.aimMotor = aimMotor;
        this.hoodMotor = hoodMotor;
        this.aimEncoder = aimEncoder;
    }

    public void setSetpoints(double aimDeg, double hoodDeg) {
        aimMotor.setSetpoint(aimDeg);
        hoodMotor.setSetpoint(hoodDeg);
    }

    public void setAimPosition() {
        aimMotor.setTargetPositionDegrees(aimMotor.getSetpoint());
    }
    public void setAimPosition(double degrees) {
        aimMotor.setTargetPositionDegrees(degrees);
    }

    public void resetAimPositionToAbsolute(double offsetRotations) {
        aimMotor.resetToAbsolute(aimEncoder.getRotations()-offsetRotations);
    }
    public boolean aimIsAtPosition(double toleranceDegrees) {
        return aimMotor.isAtPosition(toleranceDegrees);
    }

    public void setHoodPosition() {
        hoodMotor.setTargetPositionDegrees(hoodMotor.getSetpoint());
    }
    public void setHoodPosition(double degrees) {
        hoodMotor.setTargetPositionDegrees(degrees);
    }

    public boolean hoodIsAtPosition(double toleranceDegrees) {
        return hoodMotor.isAtPosition(toleranceDegrees);
    }

    public void stop() {
        aimMotor.stop();
        hoodMotor.stop();
    }

    /** SysId for turret aim motor. Hold the button for the duration of the test. */
    public Command sysIdAimCommand(frc.robot.commands.SysIdCommand.Mode mode, SysIdRoutine.Direction direction) {
        return aimMotor.createSysIdCommand(this, mode, direction);
    }

    /** SysId for hood motor. Hold the button for the duration of the test. */
    public Command sysIdHoodCommand(frc.robot.commands.SysIdCommand.Mode mode, SysIdRoutine.Direction direction) {
        return hoodMotor.createSysIdCommand(this, mode, direction);
    }

    @Override
    public void periodic()
    {
        aimMotor.logMotorPID(aimEncoder.getRotations());
        hoodMotor.logMotorPID();
        aimMotor.updateFromTunables();
        hoodMotor.updateFromTunables();
    }
}

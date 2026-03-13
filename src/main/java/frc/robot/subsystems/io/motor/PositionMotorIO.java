package frc.robot.subsystems.io.motor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SysIdCommand;

/** Hardware-agnostic position control (degrees). */
public interface PositionMotorIO {
    void setTargetPositionDegrees(double degrees);

    double getPositionDegrees();

    boolean isAtPosition(double toleranceDegrees);

    void logMotorPID();
    void logMotorPID(double absEncoderRotations);

    void updateFromTunables();

    double getSetpoint();
    void setSetpoint(double value);

    void resetToAbsolute(double absolutePositionRotations);

    void stop();

    /**
     * Returns a SysIdRoutine for this motor, or null if this IO does not support SysId
     * (e.g. sim implementations).
     */
    default SysIdRoutine getSysIdRoutine(SubsystemBase subsystem) {
        return null;
    }

    /**
     * Returns a command that runs a SysId test (quasistatic or dynamic) in the given direction.
     * Default uses getSysIdRoutine() and returns a no-op if SysId is unsupported.
     */
    default Command createSysIdCommand(
            SubsystemBase subsystem, SysIdCommand.Mode mode, SysIdRoutine.Direction direction) {
        SysIdRoutine routine = getSysIdRoutine(subsystem);
        if (routine == null) {
            return Commands.none();
        }
        return new SysIdCommand(routine, subsystem, mode, direction);
    }

    /**
     * Returns a SysIdRoutine logging position in radians from horizontal, for arm characterization.
     * Override in implementations that support arm SysId.
     */
    default SysIdRoutine getSysIdRoutineArm(SubsystemBase subsystem, double horizontalOffsetRad) {
        return null;
    }

    /**
     * Returns a command that runs an arm SysId test logging position in radians from horizontal.
     * Returns a no-op if the implementation does not support arm SysId.
     */
    default Command createSysIdArmCommand(
            SubsystemBase subsystem, SysIdCommand.Mode mode, SysIdRoutine.Direction direction,
            double horizontalOffsetRad) {
        SysIdRoutine routine = getSysIdRoutineArm(subsystem, horizontalOffsetRad);
        if (routine == null) {
            return Commands.none();
        }
        return new SysIdCommand(routine, subsystem, mode, direction);
    }
}

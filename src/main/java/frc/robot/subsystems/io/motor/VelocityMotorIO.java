package frc.robot.subsystems.io.motor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SysIdCommand;

/** Hardware-agnostic velocity control (rotations per second). */
public interface VelocityMotorIO {
    void setVelocityRps(double rotationsPerSecond);

    double getVelocityRps();

    void logMotorPID();

    void updateFromTunables();

    double getSetpointRPM();
    void setSetpoint(double value);

    void set(double percent);

    boolean isAtSpeed(double toleranceRps);

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
}

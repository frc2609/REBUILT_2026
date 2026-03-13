package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Reusable command that runs a SysId test (quasistatic or dynamic) in a given direction.
 * Hold the bound button for the duration of the test.
 */
public class SysIdCommand extends Command {
    public enum Mode {
        QUASISTATIC,
        DYNAMIC
    }

    private final SysIdRoutine routine;
    private final SubsystemBase subsystem;
    private final Mode mode;
    private final SysIdRoutine.Direction direction;
    private Command inner;

    public SysIdCommand(
            SysIdRoutine routine,
            SubsystemBase subsystem,
            Mode mode,
            SysIdRoutine.Direction direction) {
        this.routine = routine;
        this.subsystem = subsystem;
        this.mode = mode;
        this.direction = direction;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        inner =
                (mode == Mode.QUASISTATIC)
                        ? routine.quasistatic(direction)
                        : routine.dynamic(direction);
        inner.initialize();
    }

    @Override
    public void execute() {
        inner.execute();
    }

    @Override
    public void end(boolean interrupted) {
        inner.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return inner.isFinished();
    }
}


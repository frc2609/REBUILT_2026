package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeedSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;

/**
 * Lob flywheel speed every tick and feed when the wheel tracks; avoids SOTM clearing auto speed
 * while the robot moves fast.
 */
public class MovingAutoShoot extends Command {
    private final FlywheelSubsystem flywheel;
    private final FeedSubsystem agitator;

    public MovingAutoShoot(
            FlywheelSubsystem flywheel, FeedSubsystem agitator) {
        this.flywheel = flywheel;
        this.agitator = agitator;
        addRequirements(flywheel, agitator);
    }

    @Override
    public void execute() {
        double lobRps = Constants.Controls.FLYWHEEL_LOB_RPM / 60.0;
        flywheel.setSpeed(lobRps);

        double tol = Constants.Controls.FLYWHEEL_TOLERANCE_RPM / 60.0;
        if (flywheel.isAtSpeed(lobRps, tol)) {
            agitator.setAgitatorSpeed();
            agitator.setFeedSpeed();
        } else {
            agitator.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        agitator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

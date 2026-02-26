package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.VelocityMotorIO;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class ShooterSubsystem extends SubsystemBase {
    private final VelocityMotorIO flywheelMotor;
    private final VelocityMotorIO feedMotor;
    private final PositionMotorIO aimMotor;
    private final PositionMotorIO hoodMotor;
    private final AbsEncoderIO aimEncoder;


    public ShooterSubsystem(
        VelocityMotorIO flywheelMotor, VelocityMotorIO feedMotor, 
        PositionMotorIO aimMotor, PositionMotorIO hoodMotor, 
        AbsEncoderIO aimEncoder
    ) {
        this.flywheelMotor = flywheelMotor;
        this.feedMotor = feedMotor;
        this.aimMotor = aimMotor;
        this.hoodMotor = hoodMotor;
        this.aimEncoder = aimEncoder;
    }

    public void setFlywheelSpeed(double rotationsPerSecond) {
        flywheelMotor.setVelocityRps(rotationsPerSecond);
    }

    public boolean flywheelIsAtSpeed(double tolerance) {
        return flywheelMotor.isAtSpeed(tolerance);
    }

    public void setFeedSpeed(double rotationsPerSecond) {
        feedMotor.setVelocityRps(rotationsPerSecond);
    }

    public boolean feedIsAtSpeed(double tolerance) {
        return feedMotor.isAtSpeed(tolerance);
    }

    public boolean isReady() {
        return flywheelIsAtSpeed(2.0);
    }

    public void setAimPosition(double degrees) {
        aimMotor.setTargetPositionDegrees(degrees);
    }

    public void resetAimPositionToAbsolute() {
        aimMotor.resetToAbsolute(aimEncoder.getRotations());
    }

    public boolean aimIsAtPosition(double toleranceDegrees) {
        return aimMotor.isAtPosition(toleranceDegrees);
    }

    public void stop() {
        aimMotor.stop();
        hoodMotor.stop();
        flywheelMotor.stop();
    }
}

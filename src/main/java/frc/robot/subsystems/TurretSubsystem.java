package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public void setAimPosition(double degrees) {
        aimMotor.setTargetPositionDegrees(degrees);
    }

    public void resetAimPositionToAbsolute() {
        aimMotor.resetToAbsolute(aimEncoder.getRotations());
    }

    public boolean aimIsAtPosition(double toleranceDegrees) {
        return aimMotor.isAtPosition(toleranceDegrees);
    }

    public void setHoodPosition(double degrees) {
        hoodMotor.setTargetPositionDegrees(degrees);
    }

    public void resetHoodPositionToAbsolute() {
        hoodMotor.resetToAbsolute(aimEncoder.getRotations());
    }

    public boolean hoodIsAtPosition(double toleranceDegrees) {
        return hoodMotor.isAtPosition(toleranceDegrees);
    }

    public void stop() {
        aimMotor.stop();
        hoodMotor.stop();
    }
}

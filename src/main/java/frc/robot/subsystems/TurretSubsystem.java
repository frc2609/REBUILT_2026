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

        System.out.println("RESET HOOD TO ZERO");
        hoodMotor.resetToZero();
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
    public void setAimPositionFF(double degrees, double ff) {
        aimMotor.setTargetPositionDegrees(degrees, ff);
    }

    public void setEncoderInvert(boolean invert){
        this.aimEncoder.setInverted(invert);
    }
    public void resetAimPositionToAbsolute(double offsetRotations) {
        if (aimEncoder.getRotations() == 0) {
            System.out.println("RETRYING ZERO ON AIM");
            resetAimPositionToAbsolute(offsetRotations);
        }
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
    public double getHoodPosition() {
        return hoodMotor.getPositionDegrees();
    }

    public boolean hoodIsAtPosition(double toleranceDegrees) {
        return hoodMotor.isAtPosition(toleranceDegrees);
    }

    public void stop() {
        aimMotor.stop();
        hoodMotor.stop();
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

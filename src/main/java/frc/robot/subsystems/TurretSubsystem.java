package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.util.Conversions;

/** Shooter Subsystem using velocity control (rotations per second). */
public class TurretSubsystem extends SubsystemBase {
    private final PositionMotorIO aimMotor;
    private final PositionMotorIO hoodMotor;
    // private final AbsEncoderIO aimEncoder;

    public TurretSubsystem(
        PositionMotorIO aimMotor, PositionMotorIO hoodMotor
        // AbsEncoderIO aimEncoder
    ) {
        this.aimMotor = aimMotor;
        this.hoodMotor = hoodMotor;
        // this.aimEncoder = aimEncoder;

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
    public double getAimPosition(){
        return aimMotor.getPositionDegrees();
    }

    // public void setEncoderInvert(boolean invert){
    //     this.aimEncoder.setInverted(invert);
    // }
    // public void resetAimPositionToAbsolute(double offsetRotations) {
    //     if (aimEncoder.getRotations() == 0.0 && 
    //         Constants.currentMode != Constants.Mode.SIM
    //     ){
    //         System.out.println("RETRYING ZERO ON AIM");
    //         resetAimPositionToAbsolute(offsetRotations);
    //         return;
    //     }
    //     aimMotor.resetToAbsolute(aimEncoder.getRotations()-offsetRotations);
    // }

    public void zeroCurrentAimPosition() {
        aimMotor.resetToZero();
    }

    public void setAimCoastMode(boolean coast) {
        aimMotor.setCoastMode(coast);
    }

    public void setCoastMode(boolean coast) {
        aimMotor.setCoastMode(coast);
        hoodMotor.setCoastMode(coast);
    }

    public void restoreConfiguredNeutralMode() {
        aimMotor.restoreConfiguredNeutralMode();
        hoodMotor.restoreConfiguredNeutralMode();
    }

    public double getHoodCurrentAmps() {
        return hoodMotor.getStatorCurrentAmps();
    }

    public void zeroHoodPosition() {
        hoodMotor.resetToZero();
    }
    public boolean aimIsAtPosition() {
        return aimMotor.isAtPosition(Constants.Controls.TURRET_READY_TOLERANCE);
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
        //aimMotor.logMotorPID(aimEncoder.getRotations());
        hoodMotor.logMotorPID();
        aimMotor.logMotorPID();
        aimMotor.updateFromTunables();
        hoodMotor.updateFromTunables();

        Logger.recordOutput(
            "MechanismOutput/Turret azimuth (degs per s)",
            Conversions.motorRpsToOutputDegPerSec(
                aimMotor.getRotorVelocityRps(), Constants.Turret.Aim.GEAR_RATIO));
        Logger.recordOutput(
            "MechanismOutput/Turret hood (degs per s)",
            Conversions.motorRpsToOutputDegPerSec(
                hoodMotor.getRotorVelocityRps(), Constants.Turret.Hood.GEAR_RATIO));
    }
}

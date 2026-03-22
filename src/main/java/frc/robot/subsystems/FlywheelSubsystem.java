package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class FlywheelSubsystem extends SubsystemBase {
    private final VelocityMotorIO flywheelMotor;
    private double autoSpeedRPS = 0.0;
    public Translation3d launchPosSim;
    public Translation3d launchSpeedSim;

    public FlywheelSubsystem(
        VelocityMotorIO flywheelMotor
    ) {
        this.flywheelMotor = flywheelMotor;
    }

    public boolean validShotDetected() {
        return this.autoSpeedRPS != 0.0;
    }
    public void setAutoSpeed(double rps) {
        this.autoSpeedRPS = rps;
        Logger.recordOutput("OnTheFly Setpoint", this.autoSpeedRPS*60.0);
    }
    public void useAutoSpeed(){
        setSpeed(this.autoSpeedRPS);
    }

    public void setSetpoint(double rps) {
        flywheelMotor.setSetpoint(rps);
    }
    public double getSetpointRPS() {
        return flywheelMotor.getSetpointRPM()/60.0;
    }

    public void setSpeed() {
        flywheelMotor.setVelocityRps(getSetpointRPS());
    }
    public void setSpeed(double rotationsPerSecond) {
        flywheelMotor.setVelocityRps(rotationsPerSecond);
    }
    
    public void bangBang(double speedRPS, double kF){
        if (flywheelMotor.getVelocityRps() < speedRPS) {
            flywheelMotor.set(1.0);
        } else {
            flywheelMotor.set(kF);
        }
    }

    public boolean isAtSpeed() {
        return flywheelMotor.isAtSpeed(Constants.Controls.FLYWHEEL_TOLERANCE_RPM);
    }

    public void stop() {
        flywheelMotor.stop();
    }

    @Override
    public void periodic()
    {
        flywheelMotor.logMotorPID();
        flywheelMotor.updateFromTunables();
    }
}

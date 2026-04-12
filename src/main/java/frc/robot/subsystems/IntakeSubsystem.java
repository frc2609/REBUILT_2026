package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

/** Intake Subsystem using velocity control (rotations per second). */
public class IntakeSubsystem extends SubsystemBase {
    private final PositionMotorIO deployMotor;
    private final VelocityMotorIO driveMotor;

    public IntakeSubsystem(
        PositionMotorIO deployMotor, VelocityMotorIO driveMotor) {
        this.deployMotor = deployMotor;
        this.driveMotor = driveMotor;
    }

    public void setDeploySetpoint(double deg) {
        deployMotor.setSetpoint(deg);
    }

    public void setRollerSpeed(double speedRPS) {
        driveMotor.setVelocityRps(speedRPS);
    }

    public boolean isRollerRunning() {
        return driveMotor.getVelocityRps() > 0.1;
    }

    public boolean rollerIsAtSpeed(double tolerance) {
        return driveMotor.isAtSpeed(tolerance);
    }

    public void setDeployPosition() {
        deployMotor.setTargetPositionDegrees(deployMotor.getSetpoint());
    }
    public void setDeployPosition(double degrees) {
        deployMotor.setTargetPositionDegrees(degrees);
    }

    // public void resetDeployPositionToAbsolute(double offsetRotations) {
    //     double motorRotations = deployEncoder.getRotations()-offsetRotations;
    //     if (motorRotations <= -0.1) { motorRotations += 1.0; }

    //     System.out.println("DEPLOY ZEROED, rotor offset: "+motorRotations);
    //     deployMotor.resetToAbsolute(motorRotations);
    // }

    public void zeroDeployToRotations(double offsetRotations) {
        deployMotor.resetToRotations(offsetRotations);
    }

    public double getDeployCurrentAmps() {
        return deployMotor.getStatorCurrentAmps();
    }

    public void zeroDeployPosition() {
        deployMotor.resetToZero();
    }

    public boolean deployIsAtPosition(double toleranceDegrees) {
        return deployMotor.isAtPosition(toleranceDegrees);
    }
    public void stopDeploy() {
        deployMotor.stop();
    }

    public void stop() {
        driveMotor.stop();
    }

    @Override
    public void periodic()
    {
        driveMotor.logMotorPID();
        deployMotor.updateFromTunables();
        driveMotor.updateFromTunables();
    }
}

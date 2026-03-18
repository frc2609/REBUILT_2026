package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.subsystems.io.motor.VelocityMotorIO;

/** Intake Subsystem using velocity control (rotations per second). */
public class IntakeSubsystem extends SubsystemBase {
    private final PositionMotorIO deployMotor;
    private final VelocityMotorIO driveMotor;
    private final AbsEncoderIO deployEncoder;

    public IntakeSubsystem(
        AbsEncoderIO deployEncoder, PositionMotorIO deployMotor, VelocityMotorIO driveMotor) {
        this.deployMotor = deployMotor;
        this.driveMotor = driveMotor;
        this.deployEncoder = deployEncoder;
    }

    public void setDeploySetpoint(double deg) {
        deployMotor.setSetpoint(deg);
    }

    public void setRollerSpeed(double speedRPS) {
        driveMotor.setVelocityRps(speedRPS);
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

    public void resetDeployPositionToAbsolute(double offsetRotations) {
        System.out.println("DEPLOY ZEROED, ENCODER: "+deployEncoder.getRotations()+" - OFFSET: "+offsetRotations);
        deployMotor.resetToAbsolute(deployEncoder.getRotations()-offsetRotations);
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
        deployMotor.logMotorPID(deployEncoder.getRotations());
        driveMotor.logMotorPID();
        deployMotor.updateFromTunables();
        driveMotor.updateFromTunables();
    }

    public void setEncoderInvert(boolean inverted) {
        this.deployEncoder.setInverted(inverted);
    }
}

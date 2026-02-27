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

    public void setRollerSpeed(double speed) {
        driveMotor.setVelocityRps(speed);
    }

    public boolean rollerIsAtSpeed(double tolerance)
    {
        return driveMotor.isAtSpeed(tolerance);
    }

    public void setDeployPosition(double degrees) {
        deployMotor.setTargetPositionDegrees(degrees);
    }

    public void resetDeployPositionToAbsolute() {
        deployMotor.resetToAbsolute(deployEncoder.getRotations());
    }

    public boolean deployIsAtPosition(double toleranceDegrees) {
        return deployMotor.isAtPosition(toleranceDegrees);
    }

    public void stop() {
        driveMotor.stop();
    }

    @Override
    public void periodic()
    {
        deployMotor.logMotorPID();
        driveMotor.logMotorPID();

        deployMotor.updateFromTunables();
        driveMotor.updateFromTunables();
    }
}

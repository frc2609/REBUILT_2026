package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
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

    /** SysId for intake deploy motor. Hold the bound button for the duration of the test. */
    public Command sysIdDeployCommand(frc.robot.commands.SysIdCommand.Mode mode, SysIdRoutine.Direction direction) {
        return deployMotor.createSysIdArmCommand(
            this, mode, direction, Constants.Intake.Deploy.HORIZONTAL_OFFSET_RAD);
    }

    /** SysId for intake roller (drive) motor. Hold the bound button for the duration of the test. */
    public Command sysIdRollerCommand(frc.robot.commands.SysIdCommand.Mode mode, SysIdRoutine.Direction direction) {
        return driveMotor.createSysIdCommand(this, mode, direction);
    }

    @Override
    public void periodic()
    {
        deployMotor.logMotorPID(deployEncoder.getRotations());
        driveMotor.logMotorPID();
        deployMotor.updateFromTunables();
        driveMotor.updateFromTunables();

        // Verify SysId arm angle offset: position the arm at horizontal and confirm this reads 0.
        // Adjust Constants.Intake.Deploy.HORIZONTAL_OFFSET_RAD until it does.
        Logger.recordOutput("Intake/Deploy/SysIdArmAngleRad",
            deployMotor.getPositionDegrees() * Math.PI / 180.0
            + Constants.Intake.Deploy.HORIZONTAL_OFFSET_RAD);
    }
}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.subsystems.io.motor.PercentMotorIO;

/** Intake Subsystem using velocity control (rotations per second). */
public class IntakeSubsystem extends SubsystemBase {
    private final PositionMotorIO deployMotor;
    private final PercentMotorIO rollerMotor;

    public IntakeSubsystem(
        PositionMotorIO deployMotor, PercentMotorIO rollerMotor) {
        this.deployMotor = deployMotor;
        this.rollerMotor = rollerMotor;
    }

    public void setSetpoints(double rollerPercent, double deployDeg) {
        rollerMotor.setSetpoint(rollerPercent);
        System.out.println("~~~~~~~~~SET ROLLER~~~~~~~: "+rollerPercent);
        deployMotor.setSetpoint(deployDeg);
    }

    public boolean isRollerRunning() {
        return false; // TODO
    }

    public void setRollerPercent(double percent) {
        System.out.println("~~~~~~ROLLER POWER~~~~~~"+rollerMotor.getSetpoint());
        rollerMotor.setPercent(percent);
    }

    public double getDeploySetpoint() {
        return deployMotor.getSetpoint();
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
        rollerMotor.stop();
    }

    @Override
    public void periodic()
    {
        rollerMotor.logMotorPID();
        deployMotor.logMotorPID();

        deployMotor.updateFromTunables();
    }
}

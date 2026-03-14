package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.util.FuelPhysicsSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class TurretSubsystem extends SubsystemBase {
    private final PositionMotorIO aimMotor;
    private final PositionMotorIO hoodMotor;
    private final AbsEncoderIO aimEncoder;
    public final FuelPhysicsSim ballSim;

    public TurretSubsystem(
        PositionMotorIO aimMotor, PositionMotorIO hoodMotor, 
        AbsEncoderIO aimEncoder, DriveSubsystem swerve
    ) {
        this.aimMotor = aimMotor;
        this.hoodMotor = hoodMotor;
        this.aimEncoder = aimEncoder;

        this.ballSim = new FuelPhysicsSim("Sim/Fuel");
        ballSim.enable();
        ballSim.placeFieldBalls(); 

        ballSim.configureRobot(0.5, 0.5, 0.01,
            () -> swerve.getPose(), () -> swerve.getChassisSpeeds());
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
    
    @Override
    public void periodic()
    {
        aimMotor.logMotorPID();
        hoodMotor.logMotorPID();

        aimMotor.updateFromTunables();
        hoodMotor.updateFromTunables();

        ballSim.tick();
    }
}

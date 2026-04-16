package frc.robot.subsystems.io.motor;

/** Hardware-agnostic position control (degrees). */
public interface PositionMotorIO {
    void setTargetPositionDegrees(double degrees);
    void setTargetPositionDegrees(double degrees, double FF);

    double getPositionDegrees();

    boolean isAtPosition(double toleranceDegrees);

    void logMotorPID();
    void logMotorPID(double absEncoderRotations);

    void updateFromTunables();

    double getSetpoint();
    void setSetpoint(double value);

    void resetToAbsolute(double absolutePositionRotations);
    void resetToRotations(double rotations);
    void resetToZero();

    double getStatorCurrentAmps();
    double getRotorVelocityRps();

    void setCoastMode(boolean coast);

    default void restoreConfiguredNeutralMode() {}

    void stop();
}

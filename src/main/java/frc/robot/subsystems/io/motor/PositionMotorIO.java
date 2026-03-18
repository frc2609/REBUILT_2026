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
    void resetToZero();

    void stop();

    /** Sends a 0% DutyCycleOut — an active (non-neutral) request with zero output.
     *  Use when a live control request is needed (e.g. to trigger soft-limit faults)
     *  without actually driving the motor. No-op by default for non-CTRE implementations. */
    default void setOutputZero() {}


}

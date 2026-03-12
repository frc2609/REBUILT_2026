package frc.robot.subsystems.io.motor;

/** Hardware-agnostic velocity control (rotations per second). */
public interface VelocityMotorIO {
    void setVelocityRps(double rotationsPerSecond);

    double getVelocityRps();

    void logMotorPID();

    void updateFromTunables();

    double getSetpointRPM();
    void setSetpoint(double value);

    void set(double percent);

    boolean isAtSpeed(double toleranceRps);

    void stop();
}

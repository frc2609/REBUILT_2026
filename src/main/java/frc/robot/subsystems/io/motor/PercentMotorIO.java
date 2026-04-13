package frc.robot.subsystems.io.motor;

/** Hardware-agnostic percent motor control. Percent is expected in the range [-1.0, 1.0]. */
public interface PercentMotorIO {
    void setPercent(double percent);

    void logMotorPID();

    void setSetpoint(double setpoint);
    double getSetpoint();

    double getRotorVelocityRps();
    
    void stop();
}

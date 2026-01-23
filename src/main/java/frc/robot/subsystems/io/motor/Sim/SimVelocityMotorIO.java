package frc.robot.subsystems.io.motor.Sim;

import frc.robot.subsystems.io.motor.VelocityMotorIO;

public class SimVelocityMotorIO implements VelocityMotorIO {
    private double targetRotationsPerSecond = 0.0;

    @Override
    public void setVelocityRps(double rotationsPerSecond) {
        this.targetRotationsPerSecond = rotationsPerSecond;
    }

    @Override
    public double getVelocityRps() {
        return targetRotationsPerSecond;
    }

    @Override
    public boolean isAtSpeed(double toleranceRps) {
        return Math.abs(targetRotationsPerSecond - getVelocityRps()) <= toleranceRps;
    }

    @Override
    public void stop() {
        targetRotationsPerSecond = 0.0;
    }
}

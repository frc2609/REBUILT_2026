package frc.robot.subsystems.io.encoder.impl;

import frc.robot.subsystems.io.encoder.AbsEncoderIO;

public class SimAbsEncoderIO implements AbsEncoderIO {
    private final double absolutePositionRotations;

    public SimAbsEncoderIO(double absolutePositionRotations) {
        this.absolutePositionRotations = absolutePositionRotations;
    }

    @Override
    public double getAbsolutePositionRotations() {
        return absolutePositionRotations;
    }
}

package frc.robot.subsystems.io.encoder;

/**
 * Hardware-agnostic absolute encoder. Returns absolute position in rotations in the range [0, 1).
 */
public interface AbsEncoderIO {
  double getAbsolutePositionRotations();
}

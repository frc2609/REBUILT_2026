package frc.robot.util;

public final class Conversions {
  public static double degreesToRotations(double degrees, double gearRatio) {
    return degrees / (360.0 / gearRatio);
  }

  public static double rotationsToDegrees(double rotations, double gearRatio) {
    return rotations * (360.0 / gearRatio);
  }
}

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public final class Constants {

  public static class Arm {
    // Wrist
    public static final double speed = 1;
    public static final double WRIST_ENCODER_DISTANCE_PULSE = 2.0 * Math.PI / 4096;
    public static final int WRIST_MOTOR_ID = 12;
    public static final int WRIST_ENCODER_ID = 25;
    public static final int kP = 0;
    public static final int kI = 0;
    public static final int kD = 0;
    public static final Angle WRIST_MIN_ANGLE = Degrees.of(-90);
    public static final Angle WRIST_MAX_ANGLE = Degrees.of(270);
    public static final Distance WRIST_LENGTH = Inches.of(11.877934);
    public static final MomentOfInertia WRIST_MOI = KilogramSquareMeters.of(0.3175242664);
    public static final double WRIST_GEAR_RATIO = 35;
    public static final Angle WRIST_ZERO_OFFSET = Revolutions.of(0.8164931);
  }
}

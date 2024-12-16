package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.*;

public final class Constants {
  public static final boolean isReplayMode = false;

  public static class ArmConstants {
    // Wrist
    public static final int WRIST_MOTOR_ID = 12;
    public static final int WRIST_ENCODER_ID = 25;

    public static final Angle WRIST_MIN_ANGLE = Degrees.of(-180);
    public static final Angle WRIST_MAX_ANGLE = Degrees.of(360 * 2);
    public static final Distance WRIST_LENGTH = Inches.of(11.877934);
    public static final MomentOfInertia WRIST_MOI = KilogramSquareMeters.of(0.3175242664);
    public static final double WRIST_GEAR_RATIO = 35;
    public static final Angle WRIST_ZERO_OFFSET = Revolutions.of(0.8164931);

    public static final ArmFeedforward FF = new ArmFeedforward(0, 0.54, 4.28, 0.05);
    public static final ClosedLoopConfig loopCfg =
        new ClosedLoopConfig()
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(Rotations.of(0.03).in(Degrees), 0, 0);
    public static final SparkBaseConfig sparkCfg = new SparkMaxConfig().apply(loopCfg);
  }
}

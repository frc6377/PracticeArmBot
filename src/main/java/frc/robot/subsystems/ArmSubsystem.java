package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmSubsystem {

  SparkMax wristMotor = new SparkMax(Constants.Arm.WRIST_MOTOR_ID, MotorType.kBrushless);
  SparkAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();
  SparkClosedLoopController closedLoop = wristMotor.getClosedLoopController();
  SparkMaxConfig motorConfig = new SparkMaxConfig();

  SingleJointedArmSim m_armSubsytemSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          Constants.Arm.WRIST_GEAR_RATIO,
          Constants.Arm.WRIST_MOI.in(KilogramSquareMeters),
          Constants.Arm.WRIST_LENGTH.in(Meters),
          Constants.Arm.WRIST_MIN_ANGLE.in(Radians),
          Constants.Arm.WRIST_MAX_ANGLE.in(Radians),
          true,
          0.0);

  public ArmSubsystem() {
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
    // Set PID values for velocity control in slot

  }

  public void spin(int speed) {
    wristMotor.set(speed);
  }

  public void stop() {
    wristMotor.set(0);
  }

  public void setAngle(int position) {
    closedLoop.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
  }
}

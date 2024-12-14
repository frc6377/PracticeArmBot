package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
// import static edu.wpi.first.units.Units.*;
// import edu.wpi.first.units.measure.*;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class ArmSubsystem {
  SparkMax wristMotor =
      new SparkMax(
          Constants.Arm.WRIST_MOTOR_ID,
          SparkLowLevel.MotorType.fromId(Constants.Arm.WRIST_MOTOR_ID));
  SparkAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();
  SparkClosedLoopController closedLoop = wristMotor.getClosedLoopController();

  public ArmSubsystem() {}

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

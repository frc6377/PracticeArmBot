package frc.robot.subsytems;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.leftBumper;
import edu.wpi.first.wpilibj2.command.rightBumper;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private final SparkMax motor;
  private final SparkAbsoluteEncoder motorEncoder;

  public Arm(int id) {
    motor = new SparkMax(id, null);
    motorEncoder = motor.getAbsoluteEncoder();
    motor.configure(
        Constants.Arm.sparkCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Goes to angle
  public Command gotoAngle(Angle a) {
    return startEnd(
        () -> {
          motor.getClosedLoopController().setReference(a.in(Rotations), ControlType.kPosition);
        },
        motor::stopMotor);
  }

  // Stops motor
  public Command stop(){
    return run(motor::stopMotor);
  }

  // Motor goes to max angle
  public Trigger leftBumper(){
      return startEnd(
        () -> {
          motor.gotoAngle(Degrees.of(Constants.Arm.WRIST_MAX_ANGLE))  // The Degrees.of here is redundant but I felt like it
        }
      )
  }

  // Motor goes to min angle
  public Trigger rightBumper(){
      return startEnd(
        motor.gotoAngle(Degrees.of(Constants.Arm.WRIST_MIN_ANGLE))  // Ditto
      )
  }
}

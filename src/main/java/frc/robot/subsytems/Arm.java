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

  public Command gotoAngle(Angle a) {
    return startEnd(
        () -> {
          motor.getClosedLoopController().setReference(a.in(Rotations), ControlType.kPosition);
        },
        motor::stopMotor);
  }

  public Command stop(){
    return run(motor::stopMotor);
  }

  public Command telopPeriodic(){
    
  }
}

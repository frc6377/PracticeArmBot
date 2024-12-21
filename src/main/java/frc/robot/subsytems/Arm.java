package frc.robot.subsytems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.Constants;
import frc.robot.Robot;

public class Arm extends SubsystemBase {

  private final SparkMax motor;
  private final SparkMaxSim simMotor;
  private final SingleJointedArmSim simJoint;
  private final Mechanism2d mech = new Mechanism2d(10, 10);
  private final MechanismLigament2d wristMech;
  private static final DCMotor gearbox = DCMotor.getNEO(1);
  private static final Angle mechOffset = Degrees.of(90);

  public Arm(int id) {
    motor = new SparkMax(id, null);
    motor.getAbsoluteEncoder();
    motor.configure(
        Constants.Arm.sparkCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (Robot.isSimulation()){
      simMotor = new SparkMaxSim(motor, null);
      simJoint = new SingleJointedArmSim(gearbox,
                                         Constants.Arm.WRIST_GEAR_RATIO,
                                         Constants.Arm.WRIST_MOI.in(KilogramSquareMeters),
                                         Constants.Arm.WRIST_LENGTH.in(Meters),
                                         Constants.Arm.WRIST_MIN_ANGLE.minus(Degrees.of(90)).in(Radians),
                                         Constants.Arm.WRIST_MAX_ANGLE.minus(Degrees.of(90)).in(Radians),
                                         true,
                                         mechOffset.in(Radians));
      wristMech = mech
                      .getRoot("root", 5, 5);
    }

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
}


/*
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
*/

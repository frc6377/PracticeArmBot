// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubSys extends SubsystemBase {

  private final SparkMax wristMotor;
  private final SparkAbsoluteEncoder wristEncoder;

  /** Creates a new ArmSubSys. */
  public ArmSubSys(int id) {
    wristMotor = new SparkMax(id, MotorType.kBrushless);
    wristEncoder = wristMotor.getAbsoluteEncoder();
    wristMotor.configure(
        Constants.Arm.sparkCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public ArmSubSys() {
    this(Arm.WRIST_MOTOR_ID);
  }

  public Command gotoAngle(Angle a) {
    return run(
        () -> {
          Voltage newFF =
              Arm.FF.calculate(
                  Rotations.of(wristEncoder.getPosition()),
                  RPM.of(wristEncoder.getVelocity()),
                  RPM.zero());
          wristMotor
              .getClosedLoopController()
              .setReference(a.in(Rotations), ControlType.kPosition, 0, newFF.in(Volts));
        });
  }

  public Command stop() {
    return runOnce(wristMotor::stopMotor);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}

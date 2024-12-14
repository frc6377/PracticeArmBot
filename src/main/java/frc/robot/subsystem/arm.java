// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

public class arm extends SubsystemBase {
  private final SparkMax PivotMotor;
  private final SparkAbsoluteEncoder PivotEncoder;
  private final PIDController PivotPid;
  private double TargetAngle;
  private final DCMotor pivotGearbox = DCMotor.getNEO(1);
  private final SingleJointedArmSim armSim;
  /** Creates a new arm. */
  public arm() {
    PivotMotor = new SparkMax(Arm.WRIST_MOTOR_ID, MotorType.kBrushless);
    PivotEncoder = PivotMotor.getAbsoluteEncoder();
    PivotPid = new PIDController(Arm.kP, Arm.kI, Arm.kD);
    armSim =
        new SingleJointedArmSim(
            pivotGearbox,
            Arm.WRIST_GEAR_RATIO,
            Arm.WRIST_MOI.in(KilogramSquareMeters),
            Arm.WRIST_LENGTH.in(Meters),
            Arm.WRIST_MIN_ANGLE.in(Radians),
            Arm.WRIST_MAX_ANGLE.in(Radians),
            true,
            0,
            Arm.WRIST_ENCODER_DISTANCE_PULSE,
            1);
  }

  public Command GoToAngle(Angle angle) {
    return runOnce(
        () -> {
          double TargetAngle = angle.in(Radians);
        });
  }

  public Command Stop() {
    return runOnce(
        () -> {
          PivotMotor.set(0);
        });
  }

  private void Update() {
    PivotMotor.set(PivotPid.calculate(PivotEncoder.getPosition(), TargetAngle));
  }

  @Override
  public void periodic() {
    Update();
  }
}

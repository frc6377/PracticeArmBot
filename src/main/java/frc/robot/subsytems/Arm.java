// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private SparkMax motor;
  private SparkAbsoluteEncoder encoder;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoop;

  public Arm() {
    motor = new SparkMax(Constants.Arm.WRIST_MOTOR_ID, SparkLowLevel.MotorType.fromId(Constants.Arm.WRIST_MOTOR_ID));
    encoder = motor.getAbsoluteEncoder();
    closedLoop = motor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  void gotoAngle(double speed, int angle) {
    closedLoop.setReference(speed, SparkBase.ControlType.kMAXMotionPositionControl, angle);
  }

  void stop() {
    motor.stopMotor();
  }
  
}

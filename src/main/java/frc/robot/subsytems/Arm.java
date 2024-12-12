// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private SparkMax motor;
  private SparkAbsoluteEncoder encoder;

  public Arm() {
    motor = new SparkMax(12, null);
    encoder = motor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  void gotoAngle(double angle) {
    motor.
  }

  
}

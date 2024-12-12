// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private SparkAbsoluteEncoder encoder;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    motor = new SparkMax(1, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();

    motorConfig = new SparkMaxConfig();

    encoder = motor.getAbsoluteEncoder();

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(0.0)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Pose (Deg)", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  private void gotoAngle(double angleDeg) {
    closedLoopController.setReference(angleDeg, ControlType.kPosition);
    SmartDashboard.putNumber("Target Pose (Deg)", angleDeg);
  }

  private void stop() {
    motor.stopMotor();
  }

  public Command setAngleCommand(Double angleDeg) {
    return Commands.run(
            () -> {
              gotoAngle(angleDeg);
            },
            this)
        .andThen(
            () -> {
              gotoAngle(0);
            });
  }

  public Command stopCommand() {
    return Commands.run(
        () -> {
          stop();
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Angle", encoder.getPosition());
    SmartDashboard.putNumber("Motor 1 output", motor.get());
  }
}

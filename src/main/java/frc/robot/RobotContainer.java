// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystem.ArmSubsystem;
import frc.robot.Subsystem.DriveSubsystem;

public class RobotContainer {
  private ArmSubsystem m_ArmSubsystem;
  private DriveSubsystem m_DriveSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    m_ArmSubsystem = new ArmSubsystem();
    m_DriveSubsystem = new DriveSubsystem();

    configureBindings();
  }

  private void configureBindings() {
    if (Robot.isSimulation()) {

      // Arm Controlls
      m_driverController
          .button(1)
          .whileTrue(m_ArmSubsystem.setPercentCommand(1.0 * RobotController.getBatteryVoltage()));
      m_driverController
          .button(2)
          .whileTrue(m_ArmSubsystem.setPercentCommand(-1.0 * RobotController.getBatteryVoltage()));
      m_driverController.button(3).whileTrue(m_ArmSubsystem.setAngleCommand(Degrees.of(-90)));
      m_driverController.button(4).whileTrue(m_ArmSubsystem.setAngleCommand(Degrees.of(90.0)));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

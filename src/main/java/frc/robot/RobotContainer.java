// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystem.ArmSubsystem;

public class RobotContainer {
  private ArmSubsystem m_ArmSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    m_ArmSubsystem = new ArmSubsystem(1);

    configureBindings();
  }

  private void configureBindings() {
    if (Robot.isSimulation()) {
      m_driverController.button(1).whileTrue(m_ArmSubsystem.setPercentCommand(1.0));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.testCommand;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().onTrue(new testCommand(m_armSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

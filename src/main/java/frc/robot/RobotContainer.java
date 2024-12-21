// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsytems.Arm;

public class RobotContainer {
  private final Arm arm;
  private final CommandXboxController xboxController;

  public RobotContainer() {
    arm = new Arm(0);
    xboxController = new CommandXboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    xboxController.a().onTrue(arm.gotoAngle(Degrees.of(0)));
    xboxController.b().onTrue(arm.gotoAngle(Degrees.of(-90)));
    xboxController.x().onTrue(arm.gotoAngle(Degrees.of(90)));
    xboxController.y().onTrue(arm.gotoAngle(Degrees.of(180)));
    xboxController.start().onTrue(arm.stop());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

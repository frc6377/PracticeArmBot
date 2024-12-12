// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Arm;

public class RobotContainer {
  private final ArmSubSys arm, arm2;
  private final CommandXboxController xboxCtrl;

  public RobotContainer() {
    xboxCtrl = new CommandXboxController(0);
    arm = new ArmSubSys();
    arm2 = new ArmSubSys(Arm.WRIST_MOTOR_ID + 1);
    configureBindings();
  }

  private void configureBindings() {
    xboxCtrl.a().onTrue(arm.gotoAngle(Degrees.of(0)));
    xboxCtrl.b().onTrue(arm.gotoAngle(Degrees.of(90)));
    xboxCtrl.x().onTrue(arm.gotoAngle(Degrees.of(-90)));
    xboxCtrl.y().onTrue(arm.gotoAngle(Degrees.of(180)));
    xboxCtrl.start().onTrue(arm.stop());

    xboxCtrl.a().onTrue(arm2.gotoAngle(Degrees.of(0)));
    xboxCtrl.b().onTrue(arm2.gotoAngle(Degrees.of(90)));
    xboxCtrl.x().onTrue(arm2.gotoAngle(Degrees.of(-90)));
    xboxCtrl.y().onTrue(arm2.gotoAngle(Degrees.of(180)));
    xboxCtrl.start().onTrue(arm2.stop());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.subsystem.RobotArm;

public class RobotContainer {
  private final RobotArm robotArm = new RobotArm();
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Move arm to 45 degrees when A button is pressed
    controller.a().whileTrue(new SetArmPositionCommand(robotArm, 45.0));

    // Move arm to -45 degrees when B button is pressed
    controller.b().whileTrue(new SetArmPositionCommand(robotArm, -45.0));

    // Return to 0 degrees when X button is pressed
    controller.x().whileTrue(new SetArmPositionCommand(robotArm, 0.0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

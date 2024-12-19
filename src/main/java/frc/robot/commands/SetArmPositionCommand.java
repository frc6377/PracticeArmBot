// SetArmPositionCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.RobotArm;

public class SetArmPositionCommand extends Command {
  private final RobotArm arm;
  private final double targetAngle;

  public SetArmPositionCommand(RobotArm arm, double targetAngleDegrees) {
    this.arm = arm;
    this.targetAngle = targetAngleDegrees;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    arm.setPosition(targetAngle);
  }

  @Override
  public boolean isFinished() {
    // Command runs continuously until interrupted
    return false;
  }
}

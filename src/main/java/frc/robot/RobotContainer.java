
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm;
import frc.robot.commands.SetArmPositionCommand;

public class RobotContainer {
    private final Subsystem robotArm = new arm();
    private final CommandXboxController controller = new CommandXboxController(0);
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {
        // Move arm to 45 degrees when A button is pressed
        controller.a().onTrue(new SetArmPositionCommand(robotArm, 45.0));
        
        // Move arm to -45 degrees when B button is pressed
        controller.b().onTrue(new SetArmPositionCommand(robotArm, -45.0));
        
        // Return to 0 degrees when X button is pressed
        controller.x().onTrue(new SetArmPositionCommand(robotArm, 0.0));
    }
}
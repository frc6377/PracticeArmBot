// RobotArm.java
package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class RobotArm extends SubsystemBase {
    // Create a simulated arm
    private final SingleJointedArmSim armSim;
    
    // PID controller for position control
    private final PIDController pidController;
    
    // Feedforward controller for gravity compensation
    private final ArmFeedforward feedforward;
    
    // Visualization
    private final Mechanism2d mechanism;
    private final MechanismRoot2d root;
    private final MechanismLigament2d arm;
    
    // Constants
    private static final double ARM_LENGTH = 0.5; // meters
    private static final double ARM_MASS = 2.0; // kg
    private static final double MIN_ANGLE = -90.0;
    private static final double MAX_ANGLE = 90.0;
    
    public RobotArm() {
        // Initialize the arm simulation
        armSim = new SingleJointedArmSim(
            DCMotor.getNEO(1),        // 1 NEO motor
            100.0,                    // Gear ratio
            SingleJointedArmSim.EstimateMOI(ARM_LENGTH, ARM_MASS),  // Moment of inertia
            ARM_LENGTH,               // Arm length
            Units.degreesToRadians(MIN_ANGLE),  // Min angle
            Units.degreesToRadians(MAX_ANGLE),  // Max angle
            true                      // Simulate gravity
        );
        
        // Initialize PID controller
        pidController = new PIDController(1.0, 0.0, 0.0);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Initialize feedforward
        feedforward = new ArmFeedforward(0.0, 0.5, 1.0);
        
        // Initialize visualization
        mechanism = new Mechanism2d(60, 60);
        root = mechanism.getRoot("ArmPivot", 30, 30);
        arm = root.append(new MechanismLigament2d(
            "Arm",
            ARM_LENGTH * 30, // Scale up for visibility
            Units.radiansToDegrees(armSim.getAngleRads())
        ));
        
        // Put visualization on SmartDashboard
        SmartDashboard.putData("Arm Sim", mechanism);
    }
    
    public void setPosition(double targetAngleDegrees) {
        double currentAngle = armSim.getAngleRads();
        double targetAngle = Units.degreesToRadians(targetAngleDegrees);
        
        // Calculate PID output
        double pidOutput = pidController.calculate(currentAngle, targetAngle);
        
        // Calculate feedforward
        double ffOutput = feedforward.calculate(targetAngle, 0);
        
        // Apply voltage to the simulated arm
        armSim.setInputVoltage(pidOutput + ffOutput);
    }
    
    @Override
    public void periodic() {
        // Update simulation
        armSim.update(0.02);  // 20ms update rate
        
        // Update visualization
        arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
        
        // Update SmartDashboard
        SmartDashboard.putNumber("Arm Angle (deg)", Units.radiansToDegrees(armSim.getAngleRads()));
        SmartDashboard.putNumber("Arm Velocity (deg/s)", Units.radiansToDegrees(armSim.getVelocityRadPerSec()));
    }
    
    @Override
    public void simulationPeriodic() {
        // This method is called periodically during simulation
    }
}
// RobotArm.java
package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

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

  public RobotArm() {
    // Initialize the arm simulation
    armSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1), // 1 NEO motor
            Arm.WRIST_GEAR_RATIO, // Gear ratio
            Arm.WRIST_MOI.in(KilogramSquareMeters), // Moment of inertia
            Arm.WRIST_LENGTH.in(Meters), // Arm length
            Arm.WRIST_MIN_ANGLE.in(Radians), // Min angle
            Arm.WRIST_MAX_ANGLE.in(Radians), // Max angle
            true, // Simulate gravity
            0,
            Arm.WRIST_ENCODER_DISTANCE_PULSE,
            1);

    // Initialize PID controller
    pidController = new PIDController(1.0, 0.0, 0.0);
    pidController.enableContinuousInput(-Math.PI, Math.PI);

    // Initialize feedforward
    feedforward = new ArmFeedforward(0.0, 0.18, 0.69);

    // Initialize visualization
    mechanism = new Mechanism2d(60, 60);
    root = mechanism.getRoot("ArmPivot", 30, 30);
    arm =
        root.append(
            new MechanismLigament2d(
                "Arm",
                30, // Scale up for visibility
                Units.radiansToDegrees(armSim.getAngleRads())));

    // Put visualization on SmartDashboard
    SmartDashboard.putData("Arm Sim", mechanism);
  }

  public void setPosition(double targetAngleDegrees) {
    Angle currentAngle = Angle.ofBaseUnits(armSim.getAngleRads(), Radians);
    Angle targetAngle = Angle.ofBaseUnits(targetAngleDegrees, Degrees);

    // Calculate PID output
    double pidOutput = pidController.calculate(currentAngle.in(Radians), targetAngle.in(Radians));

    // Calculate feedforward
    Voltage ffOutput =
        feedforward.calculate(
            currentAngle,
            AngularVelocity.ofBaseUnits(armSim.getVelocityRadPerSec(), RadiansPerSecond));

    // Apply voltage to the simulated arm
    armSim.setInputVoltage(pidOutput + ffOutput.magnitude() + 1);
  }

  @Override
  public void periodic() {
    // Update simulation
    armSim.update(0.02); // 20ms update rate

    // Update visualization
    arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    // Update SmartDashboard
    SmartDashboard.putNumber("Arm Angle (deg)", Units.radiansToDegrees(armSim.getAngleRads()));
    SmartDashboard.putNumber(
        "Arm Velocity (deg/s)", Units.radiansToDegrees(armSim.getVelocityRadPerSec()));
  }

  @Override
  public void simulationPeriodic() {
    // This method is called periodically during simulation
  }
}

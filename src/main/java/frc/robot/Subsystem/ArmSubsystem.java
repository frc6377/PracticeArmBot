// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final SparkClosedLoopController closedLoopController;
  private final SparkAbsoluteEncoder encoder;

  // Simulation Things
  private SparkMaxSim motorSim;
  private SingleJointedArmSim armSim;
  private Mechanism2d armMech;
  private MechanismLigament2d armMechLig;

  private final Time sparkPeriod;

  private ShuffleboardTab armTab = Shuffleboard.getTab("ArmSubsystem");

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(int id) {
    sparkPeriod = id == ArmConstants.WRIST_MOTOR_ID ? Milliseconds.one() : Robot.period;

    motor = new SparkMax(ArmConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();

    motorConfig = new SparkMaxConfig();

    encoder = motor.getAbsoluteEncoder();

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(1.0)
        .i(0.0)
        .d(0.0)
        .outputRange(-1, 1);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    if (Robot.isSimulation()) {
      motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));

      armSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1),
              ArmConstants.WRIST_GEAR_RATIO,
              ArmConstants.WRIST_MOI.in(KilogramSquareMeters),
              ArmConstants.WRIST_LENGTH.in(Meters),
              ArmConstants.WRIST_MIN_ANGLE.in(Radians),
              ArmConstants.WRIST_MAX_ANGLE.in(Radians),
              true,
              ArmConstants.WRIST_ZERO_OFFSET.in(Radians));

      armMech = new Mechanism2d(10, 10);
      armMechLig =
          armMech
              .getRoot("root", 5, 5)
              .append(
                  new MechanismLigament2d(
                      "Wrist Mech [" + id + "]",
                      ArmConstants.WRIST_LENGTH.times(10).in(Meters),
                      0,
                      10,
                      new Color8Bit(
                          id == ArmConstants.WRIST_MOTOR_ID ? Color.kRed : Color.kGreen)));

      armTab.add("Arm Mech", armMech);
    }
  }

  public ArmSubsystem() {
    this(ArmConstants.WRIST_MOTOR_ID);
  }

  private Angle getArmPose() {
    if (Robot.isReal()) {
      return Rotations.of(encoder.getPosition());
    }
    return Radians.of(armSim.getAngleRads());
  }

  private void gotoAngle(Double angleDeg) {
    closedLoopController.setReference(angleDeg, ControlType.kPosition);
    SmartDashboard.putNumber("Target Pose (Deg)", angleDeg);
  }

  public Command setAngleCommand(Double angleDeg) {
    return Commands.run(() -> gotoAngle(angleDeg));
  }

  public Command setPercentCommand(Double percentPower) {
    return Commands.run(
        () -> {
          motor.set(percentPower);
        });
  }

  public Command stopCommand() {
    return runOnce(motor::stopMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Angle (Deg)", getArmPose().in(Degrees));
    SmartDashboard.putNumber("Motor 1 output", motor.get());
  }

  @Override
  public void simulationPeriodic() {
    AngularVelocity simVel = RPM.zero();
    Angle simPos = Degrees.zero();

    for (Time i = Seconds.zero(); i.lt(Robot.period); i = i.plus(sparkPeriod)) {
      armSim.setInputVoltage(motorSim.getBusVoltage() * motorSim.getAppliedOutput());
      armSim.update(sparkPeriod.in(Seconds));

      simVel = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
      simPos = Radians.of(armSim.getAngleRads());

      motorSim.iterate(
          simVel.in(RPM), RobotController.getBatteryVoltage(), sparkPeriod.in(Seconds));
    }
    armMechLig.setAngle(simPos.in(Degrees));

    SmartDashboard.putNumber("Sim Arm Pose (Rads)", armSim.getAngleRads());
    SmartDashboard.putNumber("Mech Sim Pose (Deg)", armMechLig.getAngle());
    SmartDashboard.putNumber("simVel", simVel.in(RadiansPerSecond));
    SmartDashboard.putNumber("simPos", simPos.in(Degrees));
  }
}

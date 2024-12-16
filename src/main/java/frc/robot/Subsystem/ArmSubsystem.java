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
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
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
  private final SparkAbsoluteEncoder encoder;

  // Simulation Things
  private SparkMaxSim motorSim;
  private SingleJointedArmSim armSim;
  private Mechanism2d armMech;
  private MechanismLigament2d armBaseMech;
  private MechanismLigament2d armMechLig;
  private static ComplexWidget widg;

  private final Time sparkPeriod;
  private static final DCMotor gearbox = DCMotor.getNEO(1);

  private Pose3d compPose1 = new Pose3d(0.0, 0.0, 0.15, new Rotation3d());
  private Pose3d compPose2 = new Pose3d();
  private StructPublisher<Pose3d> publisher1 =
      NetworkTableInstance.getDefault().getStructTopic("compPose1", Pose3d.struct).publish();
  private StructPublisher<Pose3d> publisher2 =
      NetworkTableInstance.getDefault().getStructTopic("compPose2", Pose3d.struct).publish();

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(int id) {
    sparkPeriod = id == ArmConstants.WRIST_MOTOR_ID ? Milliseconds.one() : Robot.period;

    motor = new SparkMax(ArmConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    motor.configure(
        ArmConstants.sparkCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    if (Robot.isSimulation()) {
      motorSim = new SparkMaxSim(motor, gearbox);

      armSim =
          new SingleJointedArmSim(
              gearbox,
              ArmConstants.WRIST_GEAR_RATIO,
              ArmConstants.WRIST_MOI.in(KilogramSquareMeters),
              ArmConstants.WRIST_LENGTH.in(Meters),
              ArmConstants.WRIST_MIN_ANGLE.in(Radians),
              ArmConstants.WRIST_MAX_ANGLE.in(Radians),
              true,
              ArmConstants.WRIST_ZERO_OFFSET.in(Radians));

      armMech = new Mechanism2d(1, 1);
      armBaseMech =
          armMech
              .getRoot("root", 0, 0)
              .append(
                  new MechanismLigament2d(
                      "Base",
                      0.5,
                      90,
                      15,
                      new Color8Bit(
                          id == ArmConstants.WRIST_MOTOR_ID ? Color.kPurple : Color.kDarkRed)));
      armMechLig =
          armBaseMech.append(
              new MechanismLigament2d(
                  "Wrist Mech [" + id + "]",
                  ArmConstants.WRIST_LENGTH.in(Meters),
                  0,
                  10,
                  new Color8Bit(id == ArmConstants.WRIST_MOTOR_ID ? Color.kBlue : Color.kRed)));

      if (widg == null) {
        widg = Shuffleboard.getTab(getName()).add("Arm", armMech);
      }

      // TODO Hack used because SparkSim iterate()
      // doesn't ever set the "Control Mode" SimInteger
      // Remove once this is fixed in REVLib
      SimDeviceSim simDev = new SimDeviceSim("SPARK MAX [" + id + "]");
      simDev.getInt("Control Mode").set(ControlType.kPosition.value);
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

  public Command setAngleCommand(Angle angleDeg) {
    return Commands.run(
        () -> {
          Voltage newFF =
              ArmConstants.FF.calculate(
                  Rotations.of(encoder.getPosition()), RPM.of(encoder.getVelocity()), RPM.zero());
          motor
              .getClosedLoopController()
              .setReference(angleDeg.in(Rotations), ControlType.kPosition, 0, newFF.in(Volts));
        });
  }

  public Command setPercentCommand(Double percentPower) {
    return Commands.runEnd(
        () -> {
          motor.getClosedLoopController().setReference(percentPower, ControlType.kVoltage);
        },
        motor::stopMotor);
  }

  public Command stopCommand() {
    return runOnce(motor::stopMotor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Pose", getArmPose().in(Degrees));
    publisher1.set(compPose1);
    publisher2.set(compPose2);
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
    armMechLig.setAngle(simPos.minus(Degrees.of(90)).in(Degrees));
    compPose2 =
        new Pose3d(
            0.0,
            0.0,
            1.075,
            new Rotation3d(0.0, simPos.in(Radians), Degrees.of(180.0).in(Radians)));

    SmartDashboard.putNumber("Applied Output", motorSim.getAppliedOutput());

    SmartDashboard.putNumber("Comp1 Rotation", simPos.minus(Degrees.of(90)).in(Degrees));
    SmartDashboard.putNumber("simVel", simVel.in(RPM));
    SmartDashboard.putNumber("simPos", simPos.in(Degrees));
  }
}

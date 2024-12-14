// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.utilities.DebugEntry;
import frc.robot.Robot;

public class arm extends SubsystemBase {
  private final SparkMax PivotMotor;
  private final SparkAbsoluteEncoder PivotEncoder;
  private final PIDController PivotPid;
  private double TargetAngle;
  private final DCMotor pivotGearbox = DCMotor.getNEO(1);
  private final SingleJointedArmSim armSim;
  private final SparkMaxSim PivotMotorSim;
  private final SparkAbsoluteEncoderSim PivotEncoderSim;
  private final Mechanism2d Mech;
  private final MechanismRoot2d ArmPivotRootMech;
  private final MechanismLigament2d ArmTowerMech;
  private final MechanismLigament2d ArmMech;
  private DebugEntry<Double> TargetAngleSim;
  private final PIDController pivotPidSim;
  /** Creates a new arm. */
  public arm() {
    PivotMotor = new SparkMax(Arm.WRIST_MOTOR_ID, MotorType.kBrushless);
    PivotEncoder = PivotMotor.getAbsoluteEncoder();
    PivotPid = new PIDController(Arm.kP, Arm.kI, Arm.kD);
    TargetAngleSim=new DebugEntry<Double>(Arm.WRIST_MIN_ANGLE.in(Degree),"Target angle",this);
    // sim stuff
    PivotMotorSim = new SparkMaxSim(PivotMotor, pivotGearbox);
    PivotEncoderSim = PivotMotorSim.getAbsoluteEncoderSim();
    pivotPidSim = new PIDController(Arm.kP,Arm.kI,Arm.kD);
    armSim =
        new SingleJointedArmSim(
            pivotGearbox,
            Arm.WRIST_GEAR_RATIO,
            Arm.WRIST_MOI.in(KilogramSquareMeters),
            Arm.WRIST_LENGTH.in(Meters),
            Arm.WRIST_MIN_ANGLE.in(Radians),
            Arm.WRIST_MAX_ANGLE.in(Radians),
            true,
            0,
            Arm.WRIST_ENCODER_DISTANCE_PULSE,
            1);
    Mech = new Mechanism2d(60, 60);
    ArmPivotRootMech = Mech.getRoot("ArmPivot", 30, 30);
    ArmTowerMech = ArmPivotRootMech.append(new MechanismLigament2d("ArmTower", 30, -90));
    ArmMech =
        ArmPivotRootMech.append(
            new MechanismLigament2d("Arm", 30, Units.radiansToDegrees(armSim.getAngleRads())));

    SmartDashboard.putData("Arm sim", Mech);
  }

  public Command GoToAngle(Angle angle) {
    return runOnce(
        () -> {
          double TargetAngle = angle.in(Radians);
        });
  }

  public Command Stop() {
    return runOnce(
        () -> {
          PivotMotor.set(0);
        });
  }

  private void Update() {
    PivotMotor.set(PivotPid.calculate(PivotEncoder.getPosition(), TargetAngle));
  }

  private void UpdateSim() {
    armSim.setInput(PivotMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    armSim.update(0.020);
    PivotEncoderSim.setPosition(armSim.getAngleRads());
    PivotMotorSim.setVelocity(pivotPidSim.calculate(PivotEncoderSim.getPosition(),Units.degreesToRadians(TargetAngleSim.get())));
    ArmMech.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    SmartDashboard.putData("Arm sim", Mech);
    SmartDashboard.putNumber("Angle", Units.radiansToDegrees(armSim.getAngleRads()));

  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      Update();
    } else {
      UpdateSim();
    }
  }
}

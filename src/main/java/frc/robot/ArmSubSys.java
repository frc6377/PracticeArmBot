// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubSys extends SubsystemBase {

  private final SparkMax wristMotor;
  private final SparkAbsoluteEncoder wristEncoder;

  // Simulation
  private SparkMaxSim simWristMotor;
  private SingleJointedArmSim simWrist;
  private Mechanism2d mech;
  private MechanismLigament2d wristMech;

  private final Time sparkPeriod = Millisecond.one();
  private final DCMotor gearbox = DCMotor.getNEO(1);
  private final Angle mechOffset = Degrees.of(-90);

  /** Creates a new ArmSubSys. */
  public ArmSubSys() {
    wristMotor = new SparkMax(Constants.Arm.WRIST_MOTOR_ID, MotorType.kBrushless);
    wristEncoder = wristMotor.getAbsoluteEncoder();
    wristMotor.configure(
        Constants.Arm.sparkCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (Robot.isSimulation()) {
      simWristMotor = new SparkMaxSim(wristMotor, gearbox);

      simWrist =
          new SingleJointedArmSim(
              gearbox,
              Constants.Arm.WRIST_GEAR_RATIO,
              Constants.Arm.WRIST_MOI.in(KilogramSquareMeters),
              Constants.Arm.WRIST_LENGTH.in(Meters),
              Constants.Arm.WRIST_MIN_ANGLE.minus(Degrees.of(90)).in(Radians),
              Constants.Arm.WRIST_MAX_ANGLE.plus(Degrees.of(90)).in(Radians),
              true,
              mechOffset.in(Radians));

      mech = new Mechanism2d(10, 10);
      wristMech =
          mech.getRoot("root", 5, 5)
              .append(
                  new MechanismLigament2d(
                      "Wrist Mech",
                      Constants.Arm.WRIST_LENGTH.times(10).in(Meters),
                      0,
                      10,
                      new Color8Bit(Color.kRed)));

      Shuffleboard.getTab(getName()).add("Arm", mech);

      // TODO Hack used because SparkSim iterate()
      // doesn't ever set the "Control Mode" SimInteger
      // Remove once this is fixed in REVLib
      SimDeviceSim simDev = new SimDeviceSim("SPARK MAX [" + Arm.WRIST_MOTOR_ID + "]");
      simDev.getInt("Control Mode").set(ControlType.kPosition.value);
    }
  }

  public Command gotoAngle(Angle a) {
    return run(
        () -> {
          Voltage newFF =
              Arm.FF.calculate(
                  Rotations.of(wristEncoder.getPosition()),
                  RPM.of(wristEncoder.getVelocity()),
                  RPM.zero());
          wristMotor
              .getClosedLoopController()
              .setReference(a.in(Rotations), ControlType.kPosition, 0, newFF.in(Volts));
        });
  }

  public Command stop() {
    return runOnce(wristMotor::stopMotor);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    AngularVelocity simVel = RPM.zero();
    Angle simPos = Degrees.zero();
    for (Time i = Seconds.zero(); i.lt(Robot.period); i = i.plus(sparkPeriod)) {
      simWrist.setInputVoltage(simWristMotor.getBusVoltage() * simWristMotor.getAppliedOutput());
      simWrist.update(sparkPeriod.in(Seconds));
      simVel = RadiansPerSecond.of(simWrist.getVelocityRadPerSec());
      simPos = Radians.of(simWrist.getAngleRads());

      simWristMotor.iterate(
          simVel.in(RPM), RobotController.getBatteryVoltage(), sparkPeriod.in(Seconds));
    }
    wristMech.setAngle(simPos.in(Degrees));
  }
}

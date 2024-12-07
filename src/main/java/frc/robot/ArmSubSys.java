// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubSys extends SubsystemBase {

  private final Time sparkPeriod = Millisecond.one();

  private final SparkMax wristMotor;

  // Simulation
  private final SparkMaxSim simWristMotor;
  private final SingleJointedArmSim simWrist;
  private final DCMotor gearbox = DCMotor.getNEO(1);
  private final Mechanism2d mech;
  private final MechanismLigament2d wristMech;
  private final Angle mechOffset = Degrees.of(-90);

  /** Creates a new ArmSubSys. */
  public ArmSubSys() {
    wristMotor = new SparkMax(Constants.Arm.WRIST_MOTOR_ID, MotorType.kBrushless);
    simWristMotor = new SparkMaxSim(wristMotor, gearbox);

    simWrist =
        new SingleJointedArmSim(
            gearbox,
            Constants.Arm.WRIST_GEAR_RATIO,
            Constants.Arm.WRIST_MOI.in(KilogramSquareMeters),
            Constants.Arm.WRIST_LENGTH.in(Meters),
            Constants.Arm.WRIST_MIN_ANGLE.in(Radians),
            Constants.Arm.WRIST_MAX_ANGLE.in(Radians),
            true,
            0);

    mech = new Mechanism2d(2, 2);
    wristMech =
        mech.getRoot("root", 1, 1)
            .append(
                new MechanismLigament2d(
                    "Wrist Mech",
                    Constants.Arm.WRIST_LENGTH.in(Meters),
                    mechOffset.in(Degrees),
                    10,
                    new Color8Bit(Color.kRed)));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run

    for (Time i = Seconds.zero(); i.lt(Robot.period); i = i.plus(sparkPeriod)) {
      simWrist.setInputVoltage(simWristMotor.getBusVoltage() * simWristMotor.getAppliedOutput());
      simWrist.update(sparkPeriod.in(Seconds));
      AngularVelocity simVel = RadiansPerSecond.of(simWrist.getVelocityRadPerSec());

      simWristMotor.iterate(
          simVel.times(Constants.Arm.WRIST_GEAR_RATIO).in(RPM),
          RobotController.getBatteryVoltage(),
          sparkPeriod.in(Seconds));
    }
    wristMech.setAngle(Radians.of(simWrist.getAngleRads()).plus(mechOffset).in(Degrees));
  }
}

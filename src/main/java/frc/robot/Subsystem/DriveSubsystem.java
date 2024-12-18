// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {
  private Pose3d drivePose;

  private SimulatedArena arenaSim;

  private final DriveTrainSimulationConfig driveTrainSimulationConfig;
  private final SelfControlledSwerveDriveSimulation swerveDriveSim;
  private final Field2d field2d;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    drivePose = new Pose3d();

    driveTrainSimulationConfig = DriveTrainSimulationConfig.Default();

    swerveDriveSim =
        new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(
                driveTrainSimulationConfig, new Pose2d(3, 3, new Rotation2d())));
    swerveDriveSim.withCurrentLimits(Amps.of(60), Amps.of(20));

    // Obtains the default instance of the simulation world, which is a Crescendo Arena.
    arenaSim = SimulatedArena.getInstance();
    arenaSim.addGamePiece(new CrescendoNoteOnField(new Translation2d(3, 3)));
    arenaSim.addDriveTrainSimulation(swerveDriveSim.getDriveTrainSimulation());

    field2d = new Field2d();
    SmartDashboard.putData("Field Sim/field2d", field2d);
  }

  public Command DriveCommand(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
    return Commands.run(
        () -> {
          drivePose =
              new Pose3d(
                  drivePose.getX() + (leftY.getAsDouble() * DriveConstants.DRIVE_SPEED),
                  drivePose.getY() + (leftX.getAsDouble() * DriveConstants.DRIVE_SPEED),
                  0.0,
                  new Rotation3d());
          swerveDriveSim.setSimulationWorldPose(drivePose.toPose2d());
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Field Sim/Bot Sim/Drive Pose3d", drivePose);
  }

  @Override
  public void simulationPeriodic() {
    // Update Arena Sim
    arenaSim.simulationPeriodic();

    // Logs notes on the field
    List<Pose3d> noteList = arenaSim.getGamePiecesByType("Note");
    for (int i = 0; i < noteList.size(); i++) {
      Logger.recordOutput("Field Sim/Game Pieces " + (i + 1), noteList.get(i));
    }

    field2d.setRobotPose(drivePose.toPose2d());
    field2d.getObject("odometry").setPose(drivePose.toPose2d());
  }
}

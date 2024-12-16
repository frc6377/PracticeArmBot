// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
  private Pose3d drivePose = new Pose3d();

  private StructPublisher<Pose3d> publisher1 =
      NetworkTableInstance.getDefault().getStructTopic("fakeBotPose", Pose3d.struct).publish();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  public Command DriveCommand(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
    return Commands.run(
        () -> {
          drivePose =
              new Pose3d(
                  drivePose.getX() + (leftY.getAsDouble() * DriveConstants.DRIVE_SPEED),
                  drivePose.getY() + (leftX.getAsDouble() * DriveConstants.DRIVE_SPEED),
                  0.0,
                  new Rotation3d());
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    publisher1.set(drivePose);
  }
}

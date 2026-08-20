// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotContainer {
  // Hardcoded 3m 3m coords fn
  private static final Pose2d PATHING_TARGET_POSE = new Pose2d(3.0, 3.0, Rotation2d.kZero);

  public final Superstructure superstructure;
  public final Drive drive;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    superstructure = new Superstructure(this);
    drive = new Drive(drivetrain);

    Controllers.DRIVER
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.setPathingTargetPose(PATHING_TARGET_POSE);
                  drive.setDriveMode(Drive.Mode.PATHING);
                  System.out.println("Pathing enabled");
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  drive.setDriveMode(Drive.Mode.MANUAL);
                  System.out.println("Pathing disabled");
                }));

    Controllers.DRIVER
        .b()
        .onTrue(
            Commands.runOnce(() -> drivetrain.resetPose(new Pose2d(1.0, 1.0, Rotation2d.kZero))));

    DriverStation.silenceJoystickConnectionWarning(true);

    if (Robot.isSimulation())
      drivetrain.resetPose(new Pose2d(3, 3, Rotation2d.kZero));

    FollowPath.registerEventTrigger("shoot", superstructure.shootAuto());
    FollowPath.registerEventTrigger("shootNoJuice", superstructure.shootAutoNoJuice());
    FollowPath.registerEventTrigger("neutral", superstructure.neutral());
    FollowPath.registerEventTrigger("intakeDown", superstructure.intakePivotDownAuto());
    FollowPath.registerEventTrigger("autoAlign", drive.setDriveModeCommand(Drive.Mode.AUTO_ALIGN));
    FollowPath.registerEventTrigger("thetaLock", drive.setDriveModeCommand(Drive.Mode.THETA_LOCK));
    FollowPath.registerEventTrigger("pathControl", drive.setDriveModeCommand(Drive.Mode.NONE));
    FollowPath.registerEventTrigger("driveBrake", drive.setDriveModeCommand(Drive.Mode.BRAKE));
  }

  public void periodic() {
    superstructure.periodic();
    drive.periodic();
  }

  public void resetSuperstructure() {
    superstructure.resetPositions();
  }
}

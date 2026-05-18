// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotContainer {
  public final Superstructure superstructure;
  public final Drive drivetrainController;

  private final Telemetry logger = new Telemetry(3.5);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    superstructure = new Superstructure(this);
    drivetrainController = new Drive(drivetrain, superstructure);

    configureDrivetrain();

    DriverStation.silenceJoystickConnectionWarning(true);

    drivetrain.registerTelemetry(logger::telemeterize);

    if (Robot.isSimulation()) drivetrain.resetPose(new Pose2d(3, 3, Rotation2d.kZero));

    FollowPath.registerEventTrigger("shoot", superstructure.shootAuto());
    FollowPath.registerEventTrigger("shootNoJuice", superstructure.shootAutoNoJuice());
    FollowPath.registerEventTrigger("neutral", superstructure.neutral());
    FollowPath.registerEventTrigger("intakeDown", superstructure.intakePivotDownAuto());
  }

  private void configureDrivetrain() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void periodic() {
    drivetrain.periodic();
    superstructure.periodic();
    drivetrainController.periodic();
  }

  public void resetSuperstructure() {
    superstructure.resetPositions();
  }

  public Telemetry getTelemetry() {
    return logger;
  }
}

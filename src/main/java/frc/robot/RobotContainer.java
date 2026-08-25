// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.*;
import frc.robot.lib.simulation.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotContainer {
  public final Superstructure superstructure;
  public final Drive drive;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final FuelSimHelper fuelSimHelper;

  public RobotContainer() {
    superstructure = new Superstructure(this);
    drive = new Drive(drivetrain);

    DriverStation.silenceJoystickConnectionWarning(true);

    if (Robot.isSimulation()) {
      this.fuelSimHelper = new FuelSimHelper(this);
      drivetrain.resetPose(new Pose2d(3, 3, Rotation2d.kZero));
      fuelSimHelper.configureFuelSim();
    } else {
      fuelSimHelper = null;
    }

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
    if (RobotBase.isSimulation()) {
      fuelSimHelper.periodic();
    }
    drive.periodic();
  }

  public void resetSuperstructure() {
    superstructure.resetPositions();
  }

  public void simAutoInit() {
    if (fuelSimHelper != null) fuelSimHelper.resetFuelSim();
  }
}

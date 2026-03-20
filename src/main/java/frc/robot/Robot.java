// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.HubShiftUtil;
import frc.robot.subsystems.vision.BOS;
import frc.robot.subsystems.vision.TagHelper;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.subsystems.limelight.Limelight;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  private final BOS bos;
  private final Limelight limelight;

  private final boolean USE_LIMELIGHT = false;

  public Robot() {
    Logger.recordMetadata("ProjectName", "971 First Bot 2026");
    Logger.recordMetadata("TeamNumber", "971");
    Logger.recordMetadata("RobotName", "971 Robot");

    switch (Constants.MODE) {
      case REAL -> {
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> {
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
      }
      case REPLAY -> {
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    Logger.start();

    // IMPORTANT: DO NOT DELETE OR EXPENSIVE FILE READING WILL FREEZE WHEN FIRST TAG HELPER FUNCTION
    // IS CALLED
    TagHelper.init();

    robotContainer = new RobotContainer();
    bos = new BOS(robotContainer.drivetrain);
    limelight = new Limelight(robotContainer.drivetrain);
  }

  @Override
  public void robotInit() {
    DataLogManager.start();

    robotContainer.resetSuperstructure();
  }

  @Override
  public void robotPeriodic() {
    robotContainer.periodic();

    HubShiftUtil.ShiftInfo info = HubShiftUtil.getOfficialShiftInfo();

    Logger.recordOutput("HubShift/Active", info.active());
    Logger.recordOutput("HubShift/RemainingTime", info.remainingTime());
    Logger.recordOutput("HubShift/CurrentShift", info.currentShift().toString());

    if (USE_LIMELIGHT) {
      limelight.updatePose();
    } else {
      bos.updatePose();
    }

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    HubShiftUtil.initialize();
  }

  @Override
  public void disabledPeriodic() {
    // Log the autonomous starting pose
    Pose2d autoInitPose = null;
    Command selectedAuto = robotContainer.getAutonomousCommand();

    if (selectedAuto instanceof PathPlannerAuto auto) {
      autoInitPose = auto.getStartingPose();

      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        autoInitPose = FlippingUtil.flipFieldPose(autoInitPose);
      }
    }

    // Update the field visualization with the auto start pose
    robotContainer.getTelemetry().setAutoStartPose(autoInitPose);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(robotContainer.superstructure.neutral());
      CommandScheduler.getInstance().schedule(autonomousCommand);
      robotContainer.resetPositionForAuto();
    }
    HubShiftUtil.initialize();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    HubShiftUtil.initialize();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

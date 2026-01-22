// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotContainer {
  public final Superstructure superstructure;

  private double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private final Telemetry logger = new Telemetry(MAX_SPEED);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Drive drive;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    superstructure = new Superstructure(this);
    drive = new Drive(drivetrain, superstructure);

    autoChooser = AutoBuilder.buildAutoChooser("Mobility");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // NOTE: registering drivetrain is necessary for CommandSwerveDrivetrain#periodic to run
    CommandScheduler.getInstance().registerSubsystem(drivetrain);

    DriverStation.silenceJoystickConnectionWarning(true);

    drivetrain.registerTelemetry(logger::telemeterize);

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    if (Robot.isSimulation()) drivetrain.resetPose(new Pose2d(3, 3, Rotation2d.kZero));
  }

  public void periodic() {
    drive.periodic();

    superstructure.periodic();
  }

  public void resetSuperstructure() {
    superstructure.resetPositions();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void resetPositionForAuto() {
    if (autoChooser.getSelected() instanceof PathPlannerAuto auto) {
      Pose2d startingPose = auto.getStartingPose();

      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        startingPose = FlippingUtil.flipFieldPose(startingPose);
      }

      drivetrain.resetPose(startingPose);
    }
  }

  public Telemetry getTelemetry() {
    return logger;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.JoystickValues;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.superstructure.Superstructure;

public class RobotContainer {
  public final Superstructure superstructure;

  private static final double MAX_SPEED = 3.4;
  private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.8).in(RadiansPerSecond);

  private static final double TRANSLATION_DEADBAND = 0.05;
  private static final double ROTATION_DEADBAND = 0.1;

  /* Setting up bindings for necessary control of the swerve drive platform */

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED * TRANSLATION_DEADBAND)
          .withRotationalDeadband(MAX_ANGULAR_RATE * ROTATION_DEADBAND)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Current values are placeholders and should be tuned for optimal robot control
  // Slew rate limit for translation (m/s^2)
  private static final double SLEW_TRANSLATE_LIMIT = 1000.0;
  // Slew rate limit for rotation (rad/s^2)
  private static final double SLEW_ROTATION_LIMIT = 1000.0;

  // Exponential curve for translation joystick
  private static final double TRANSLATION_EXP_CURVE = 2;
  // Exponential curve for rotation joystick
  private static final double ROTATION_EXP_CURVE = 2;

  // Rate limiters for smooth control
  private static final SlewRateLimiter X_LIMITER = new SlewRateLimiter(SLEW_TRANSLATE_LIMIT);
  private static final SlewRateLimiter Y_LIMITER = new SlewRateLimiter(SLEW_TRANSLATE_LIMIT);
  private static final SlewRateLimiter ROT_LIMITER = new SlewRateLimiter(SLEW_ROTATION_LIMIT);

  private static final JoystickValues JOYSTICK_VALUES = new JoystickValues();

  private final Telemetry logger = new Telemetry(MAX_SPEED);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    superstructure = new Superstructure(this);

    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser("Outpost Side");
    AutoBuilder.getAllAutoNames().stream()
        .sorted()
        .forEach(
            autoName ->
                autoChooser.addOption(
                    autoName + " (Mirrored)", new PathPlannerAuto(autoName, true)));

    SmartDashboard.putData("Auto Mode", autoChooser);

    configureDrivetrain();

    DriverStation.silenceJoystickConnectionWarning(true);

    drivetrain.registerTelemetry(logger::telemeterize);

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    if (Robot.isSimulation()) drivetrain.resetPose(new Pose2d(3, 3, Rotation2d.kZero));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", superstructure.intakeAuto());
    NamedCommands.registerCommand("IntakePivotDown", superstructure.intakePivotDownAuto());
    NamedCommands.registerCommand("Deployed", superstructure.deployedAuto());
    NamedCommands.registerCommand("Shoot", superstructure.shootAuto());
    NamedCommands.registerCommand("ReverseShooters", superstructure.reverseShooters());
    NamedCommands.registerCommand("ShootSequence", superstructure.shootSequenceAuto());
    NamedCommands.registerCommand("Neutral", superstructure.neutral());
    NamedCommands.registerCommand("ShootOnce", superstructure.shootOnceAuto());
  }

  private void configureDrivetrain() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              JOYSTICK_VALUES
                  .setValues(
                      Controllers.TROY.getLeftY(),
                      Controllers.TROY.getLeftX(),
                      Controllers.TROY.getRightX())
                  .exponentialCurve(TRANSLATION_EXP_CURVE, ROTATION_EXP_CURVE)
                  .scale(
                      -MAX_SPEED, // Negative max speed and angular rate since
                      -MAX_ANGULAR_RATE) // controller inputs are reversed
                  .slewRateLimit(X_LIMITER, Y_LIMITER, ROT_LIMITER);

              return drive
                  .withVelocityX(JOYSTICK_VALUES.getX())
                  .withVelocityY(JOYSTICK_VALUES.getY())
                  .withRotationalRate(JOYSTICK_VALUES.getRot());
            }));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void periodic() {
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
      if (startingPose == null) return;

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

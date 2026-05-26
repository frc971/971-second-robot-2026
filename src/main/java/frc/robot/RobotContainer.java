// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.*;
import frc.robot.lib.JoystickValues;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.lib.simulation.*;

public class RobotContainer {
  public final Superstructure superstructure;

  private static final double MAX_SPEED = 3.5;
  private static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.8).in(RadiansPerSecond);

  private static final double TRANSLATION_DEADBAND = 0.05;
  private static final double ROTATION_DEADBAND = 0.1;

  private static final double SHOOTING_SPEED = 0.3 * MAX_SPEED;
  private static final double SHOOTING_ANGULAR_RATE = 0.3 * MAX_ANGULAR_RATE;
  private static final double SHUTTLING_SPEED = 0.5 * MAX_SPEED;
  private static final double SHUTTLING_ANGULAR_RATE = 0.5 * MAX_ANGULAR_RATE;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric shootingDrive =
      new SwerveRequest.FieldCentric()
          .withDeadband(SHOOTING_SPEED * TRANSLATION_DEADBAND)
          .withRotationalDeadband(SHOOTING_ANGULAR_RATE * ROTATION_DEADBAND)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentric shuttlingDrive =
      new SwerveRequest.FieldCentric()
          .withDeadband(SHUTTLING_SPEED * TRANSLATION_DEADBAND)
          .withRotationalDeadband(SHUTTLING_ANGULAR_RATE * ROTATION_DEADBAND)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED * TRANSLATION_DEADBAND)
          .withRotationalDeadband(MAX_ANGULAR_RATE * ROTATION_DEADBAND)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // Slew rate limit for translation (m/s^2)
  private static final double SHOOTING_SLEW_TRANSLATE_LIMIT = 10.0;
  // Slew rate limit for rotation (rad/s^2)
  private static final double SHOOTING_SLEW_ROTATION_LIMIT = 100.0;
  // Slew rate limit for translation (m/s^2)
  private static final double SHUTTLING_SLEW_TRANSLATE_LIMIT = 20.0;
  // Slew rate limit for rotation (rad/s^2)
  private static final double SHUTTLING_SLEW_ROTATION_LIMIT = 100.0;

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

  private static final SlewRateLimiter SHOOTING_X_LIMITER =
      new SlewRateLimiter(SHOOTING_SLEW_TRANSLATE_LIMIT);
  private static final SlewRateLimiter SHOOTING_Y_LIMITER =
      new SlewRateLimiter(SHOOTING_SLEW_TRANSLATE_LIMIT);
  private static final SlewRateLimiter SHOOTING_ROT_LIMITER =
      new SlewRateLimiter(SHOOTING_SLEW_ROTATION_LIMIT);
  private static final SlewRateLimiter SHUTTLING_X_LIMITER =
      new SlewRateLimiter(SHUTTLING_SLEW_TRANSLATE_LIMIT);
  private static final SlewRateLimiter SHUTTLING_Y_LIMITER =
      new SlewRateLimiter(SHUTTLING_SLEW_TRANSLATE_LIMIT);
  private static final SlewRateLimiter SHUTTLING_ROT_LIMITER =
      new SlewRateLimiter(SHUTTLING_SLEW_ROTATION_LIMIT);

  private static final JoystickValues JOYSTICK_VALUES = new JoystickValues();

  private final Telemetry logger = new Telemetry(MAX_SPEED);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    superstructure = new Superstructure(this);

    configureDrivetrain();

    DriverStation.silenceJoystickConnectionWarning(true);

    drivetrain.registerTelemetry(logger::telemeterize);

    if (Robot.isSimulation()){
      drivetrain.resetPose(new Pose2d(3, 3, Rotation2d.kZero));
      configureFuelSim();
    }

    FollowPath.registerEventTrigger("shoot", superstructure.shootAuto());
    FollowPath.registerEventTrigger("shootNoJuice", superstructure.shootAutoNoJuice());
    FollowPath.registerEventTrigger("neutral", superstructure.neutral());
    FollowPath.registerEventTrigger("intakeDown", superstructure.intakePivotDownAuto());
  }

  private void configureDrivetrain() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              boolean wantsDrive =
                  Math.abs(Controllers.TROY.getLeftY()) > TRANSLATION_DEADBAND
                      || Math.abs(Controllers.TROY.getLeftX()) > TRANSLATION_DEADBAND
                      || Math.abs(Controllers.TROY.getRightX()) > ROTATION_DEADBAND;

              if (Controllers.SHOOTING.getAsBoolean() && !wantsDrive) {
                return brake;
              }

              if (Controllers.SHUTTLE_EDGE.falling()) {
                X_LIMITER.reset(SHUTTLING_X_LIMITER.lastValue());
                Y_LIMITER.reset(SHUTTLING_Y_LIMITER.lastValue());
                ROT_LIMITER.reset(SHUTTLING_ROT_LIMITER.lastValue());
              }

              if (Controllers.SHOOT_EDGE.falling()) {
                X_LIMITER.reset(SHOOTING_X_LIMITER.lastValue());
                Y_LIMITER.reset(SHOOTING_Y_LIMITER.lastValue());
                ROT_LIMITER.reset(SHOOTING_ROT_LIMITER.lastValue());
              }

              if (Controllers.SHUTTLE_EDGE.rising()) {
                SHUTTLING_X_LIMITER.reset(X_LIMITER.lastValue());
                SHUTTLING_Y_LIMITER.reset(Y_LIMITER.lastValue());
                SHUTTLING_ROT_LIMITER.reset(ROT_LIMITER.lastValue());
              }

              if (Controllers.SHOOT_EDGE.rising()) {
                SHOOTING_X_LIMITER.reset(X_LIMITER.lastValue());
                SHOOTING_Y_LIMITER.reset(Y_LIMITER.lastValue());
                SHOOTING_ROT_LIMITER.reset(ROT_LIMITER.lastValue());
              }

              if (Controllers.SHUTTLING.getAsBoolean()) {
                JOYSTICK_VALUES
                    .setValues(
                        Controllers.TROY.getLeftY(),
                        Controllers.TROY.getLeftX(),
                        Controllers.TROY.getRightX())
                    .exponentialCurve(TRANSLATION_EXP_CURVE, ROTATION_EXP_CURVE)
                    .scale(
                        -SHUTTLING_SPEED, // Negative max speed and angular rate since
                        -SHUTTLING_ANGULAR_RATE) // controller inputs are reversed
                    .slewRateLimit(X_LIMITER, Y_LIMITER, ROT_LIMITER)
                    .slewRateLimit(SHUTTLING_X_LIMITER, SHUTTLING_Y_LIMITER, SHUTTLING_ROT_LIMITER);

                return shuttlingDrive
                    .withVelocityX(JOYSTICK_VALUES.getX())
                    .withVelocityY(JOYSTICK_VALUES.getY())
                    .withRotationalRate(JOYSTICK_VALUES.getRot());

              } else if (Controllers.SHOOTING.getAsBoolean()) {
                JOYSTICK_VALUES
                    .setValues(
                        Controllers.TROY.getLeftY(),
                        Controllers.TROY.getLeftX(),
                        Controllers.TROY.getRightX())
                    .exponentialCurve(TRANSLATION_EXP_CURVE, ROTATION_EXP_CURVE)
                    .scale(
                        -SHOOTING_SPEED, // Negative max speed and angular rate since
                        -SHOOTING_ANGULAR_RATE) // controller inputs are reversed
                    .slewRateLimit(X_LIMITER, Y_LIMITER, ROT_LIMITER)
                    .slewRateLimit(SHOOTING_X_LIMITER, SHOOTING_Y_LIMITER, SHOOTING_ROT_LIMITER);
                return shootingDrive
                    .withVelocityX(JOYSTICK_VALUES.getX())
                    .withVelocityY(JOYSTICK_VALUES.getY())
                    .withRotationalRate(JOYSTICK_VALUES.getRot());

              } else {
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
              }
            }));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void periodic() {
    superstructure.periodic();
  }

  public void resetSuperstructure() {
    superstructure.resetPositions();
  }

  public Telemetry getTelemetry() {
    return logger;
  }

  // MARK: Fuel Simulation
  private void configureFuelSim() {
        FuelSim instance = FuelSim.getInstance();
        instance.clearFuel();
        instance.registerRobot(
            Dimensions.FULL_WIDTH,
            Dimensions.FULL_LENGTH,
            Dimensions.BUMPER_HEIGHT,
            () -> drivetrain.getState().Pose,
            this::getFieldRelativeChassisSpeedsForSim
        );
        instance.registerIntake(
            -Dimensions.FULL_LENGTH / 2.0,
            Dimensions.FULL_LENGTH / 2.0,
            -Dimensions.FULL_WIDTH / 2.0,
            Dimensions.FULL_WIDTH / 2.0,
            //intake::isIntaking, maybe will add this back later
            () -> Logger.recordOutput("FuelSim/LastEvent", "Intake")
        );

        instance.spawnStartingFuel();
        instance.start();

        Command spawnFuelCommand = Commands.runOnce(this::spawnFuelInFrontOfRobot)
            .ignoringDisable(true)
            .withName("FuelSim/Spawn Fuel");
        Command resetFuelCommand = Commands.runOnce(() -> {
                instance.clearFuel();
                instance.spawnStartingFuel();
                Logger.recordOutput("FuelSim/LastEvent", "Reset");
            })
            .ignoringDisable(true)
            .withName("FuelSim/Reset Fuel");
        Command launchFuelCommand = Commands.runOnce(() -> launchFuelInSim(MetersPerSecond.of(8), Degrees.of(45)))
            .ignoringDisable(true)
            .withName("FuelSim/Launch Fuel");

        SmartDashboard.putData(spawnFuelCommand);
        SmartDashboard.putData(resetFuelCommand);
        SmartDashboard.putData(launchFuelCommand);
    }

    public void resetFuelSim() {
        if (!RobotBase.isSimulation()) {
            return;
        }
        FuelSim instance = FuelSim.getInstance();
        instance.clearFuel();
        instance.spawnStartingFuel();
        Logger.recordOutput("FuelSim/LastEvent", "Auto Reset");
    }

    private void spawnFuelInFrontOfRobot() {
        Pose2d pose = drivetrain.getState().Pose;
        Translation2d offset = new Translation2d(Dimensions.FULL_LENGTH / 2.0 + 0.1, 0)
            .rotateBy(pose.getRotation());
        Translation3d location = new Translation3d(
            pose.getX() + offset.getX(),
            pose.getY() + offset.getY(),
            Dimensions.BUMPER_HEIGHT / 2.0
        );
        FuelSim.getInstance().spawnFuel(location, new Translation3d());
        Logger.recordOutput("FuelSim/LastEvent", "Manual Spawn");
    }

    private void launchFuelInSim(LinearVelocity velocity, Angle elevation) {
        Pose2d pose = drivetrain.getState().Pose;
        Translation2d muzzleOffset = new Translation2d(Dimensions.FULL_LENGTH / 2.0, 0)
            .rotateBy(pose.getRotation());
        Translation3d initialPosition = new Translation3d(
            pose.getX() + muzzleOffset.getX(),
            pose.getY() + muzzleOffset.getY(),
            Dimensions.BUMPER_HEIGHT + 0.25
        );
        Translation3d launchVelocity = createLaunchVelocity(velocity, elevation, pose.getRotation());
        FuelSim.getInstance().spawnFuel(initialPosition, launchVelocity);
        Logger.recordOutput("FuelSim/LastEvent", "Launch");
    }

    private Translation3d createLaunchVelocity(LinearVelocity velocity, Angle elevation, Rotation2d heading) {
        double speed = velocity.in(MetersPerSecond);
        double elevationRadians = elevation.in(Radians);
        double planarSpeed = speed * Math.cos(elevationRadians);
        double verticalSpeed = speed * Math.sin(elevationRadians);
        Translation2d planar = new Translation2d(planarSpeed, 0).rotateBy(heading);
        return new Translation3d(planar.getX(), planar.getY(), verticalSpeed);
    }

    private ChassisSpeeds getFieldRelativeChassisSpeedsForSim() {
        ChassisSpeeds speeds = drivetrain.getState().Speeds;
        if (speeds == null) {
            return new ChassisSpeeds();
        }
        return new ChassisSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond
        );
    }
}

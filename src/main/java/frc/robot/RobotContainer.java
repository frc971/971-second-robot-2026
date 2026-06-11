// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.*;
import frc.robot.lib.JoystickValues;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.superstructure.Superstructure;

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
    private final SwerveRequest.FieldCentric shootingDrive = new SwerveRequest.FieldCentric()
            .withDeadband(SHOOTING_SPEED * TRANSLATION_DEADBAND)
            .withRotationalDeadband(SHOOTING_ANGULAR_RATE * ROTATION_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric shuttlingDrive = new SwerveRequest.FieldCentric()
            .withDeadband(SHUTTLING_SPEED * TRANSLATION_DEADBAND)
            .withRotationalDeadband(SHUTTLING_ANGULAR_RATE * ROTATION_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
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

    private static final SlewRateLimiter SHOOTING_X_LIMITER = new SlewRateLimiter(SHOOTING_SLEW_TRANSLATE_LIMIT);
    private static final SlewRateLimiter SHOOTING_Y_LIMITER = new SlewRateLimiter(SHOOTING_SLEW_TRANSLATE_LIMIT);
    private static final SlewRateLimiter SHOOTING_ROT_LIMITER = new SlewRateLimiter(SHOOTING_SLEW_ROTATION_LIMIT);
    private static final SlewRateLimiter SHUTTLING_X_LIMITER = new SlewRateLimiter(SHUTTLING_SLEW_TRANSLATE_LIMIT);
    private static final SlewRateLimiter SHUTTLING_Y_LIMITER = new SlewRateLimiter(SHUTTLING_SLEW_TRANSLATE_LIMIT);
    private static final SlewRateLimiter SHUTTLING_ROT_LIMITER = new SlewRateLimiter(SHUTTLING_SLEW_ROTATION_LIMIT);

    private static final JoystickValues JOYSTICK_VALUES = new JoystickValues();

    private final Telemetry logger = new Telemetry(MAX_SPEED);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    // test stuff for pathing
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final DoubleSubscriber vxSub = inst.getDoubleTopic("/pathing/vx").subscribe(0.0);
    private final DoubleSubscriber vySub = inst.getDoubleTopic("/pathing/vy").subscribe(0.0);
    private final BooleanSubscriber pathingEnabled = inst.getBooleanTopic("/pathing/enabled").subscribe(false);
    private final BooleanPublisher pathingEnabledPub = inst.getBooleanTopic("/pathing/enabled").publish();
    private final StructPublisher<Pose2d> targetPub = inst.getStructTopic("/pathing/target", Pose2d.struct).publish();

    public RobotContainer() {
        superstructure = new Superstructure(this);

        configureDrivetrain();

        targetPub.set(new Pose2d(8.0, 4.0, Rotation2d.kZero));

        DriverStation.silenceJoystickConnectionWarning(true);

        drivetrain.registerTelemetry(logger::telemeterize);

        if (Robot.isSimulation())
            drivetrain.resetPose(new Pose2d(3, 3, Rotation2d.kZero));

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
                            if (pathingEnabled.get()) {
                                return drive
                                        .withVelocityX(vxSub.get())
                                        .withVelocityY(vySub.get())
                                        .withRotationalRate(0);
                            } else {
                                return drive
                                        .withVelocityX(JOYSTICK_VALUES.getX())
                                        .withVelocityY(JOYSTICK_VALUES.getY())
                                        .withRotationalRate(JOYSTICK_VALUES.getRot());
                            }
                        }));

        Controllers.TROY
                .a()
                .onTrue(Commands.runOnce(() -> pathingEnabledPub.set(true)))
                .onFalse(Commands.runOnce(() -> pathingEnabledPub.set(false)));

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
}

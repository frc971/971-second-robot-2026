package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class AutoAlign {

  public enum Goal {
    NONE,
    ALIGN
  }

  private final Pose2d EXAMPLE_TARGET_POSE = new Pose2d(1, 4, new Rotation2d(Math.PI));

  @Getter @Setter private Goal goal = Goal.NONE;
  private Goal previousGoal = Goal.NONE;

  private final CommandSwerveDrivetrain drivetrain;
  private final PIDController distanceController;

  private static final double DISTANCE_KP = 5.0;
  private static final double DISTANCE_KI = 0.0;
  private static final double DISTANCE_KD = 0.2;

  private static final double DISTANCE_FF = 0.0;
  private static final Distance DISTANCE_PID_TOLERANCE = Meters.of(0.02);

  public static final double ROTATION_KP = 3.5;
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 0.0;

  // Max velocity used to clamp output
  private static final LinearVelocity MAX_TRANSLATION_VELOCITY = MetersPerSecond.of(4.5);

  // Tolerable error before stopping auto align
  private static final Distance NORMAL_TOLERANCE = Meters.of(0.1);
  private static final Distance TANGENT_TOLERANCE = Meters.of(0.1);
  private static final Angle ROTATION_TOLERANCE = Degrees.of(3);

  private final SwerveRequest.FieldCentricFacingAngle driveAtAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
          .withHeadingPID(ROTATION_KP, ROTATION_KI, ROTATION_KD);

  private boolean isAligning = false;

  private Pose2d currentPose;
  private Pose2d targetPose;
  private Distance normalError;
  private Distance tangentError;
  private Angle rotationError;
  private Rotation2d angleToTarget;
  private Translation2d translationOutput;

  public AutoAlign(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Create PID controller for distance control
    distanceController = new PIDController(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);

    distanceController.setTolerance(DISTANCE_PID_TOLERANCE.in(Meters));
  }

  public void periodic() {
    updateDashboard();
    isAligning = goal != Goal.NONE;

    if (previousGoal == Goal.NONE && goal != Goal.NONE) {
      distanceController.reset();
    }
    previousGoal = goal;

    if (goal == Goal.NONE) {
      return;
    }

    Optional<Pose2d> targetPose =
        switch (goal) {
          case ALIGN -> DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
              ? Optional.of(FlippingUtil.flipFieldPose(EXAMPLE_TARGET_POSE))
              : Optional.of(EXAMPLE_TARGET_POSE);
          default -> Optional.empty();
        };

    if (targetPose.isEmpty()) {
      return;
    }

    Pose2d currentPose = drivetrain.getState().Pose;
    SwerveRequest.FieldCentricFacingAngle alignRequest =
        computeAlignment(currentPose, targetPose.get());

    drivetrain.setRequest(alignRequest);
  }

  /**
   * Computes a single alignment step toward the target pose.
   *
   * @param currentPose the robot's current field-relative pose
   * @param targetPose the desired pose to align to
   * @return a SwerveRequest containing translation and rotation velocities toward the target
   */
  public FieldCentricFacingAngle computeAlignment(Pose2d currentPose, Pose2d targetPose) {
    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d targetTranslation = targetPose.getTranslation();

    // control x and y translation with one distanceController using length of translation vector
    // NOTE: the distance controller's goal is 0 (distance from the target pos) (i.e. relative)
    // while the rotation controller's goal is the target rotation (i.e. absolute).
    Translation2d translationError = targetTranslation.minus(currentTranslation);
    double currentDistanceToTarget = translationError.getNorm();
    // negate because controller moves toward 0 and we want positive movement
    double distanceOutput = -distanceController.calculate(currentDistanceToTarget, 0) + DISTANCE_FF;

    LinearVelocity clampedDistanceOutput =
        MetersPerSecond.of(
            MathUtil.clamp(
                distanceOutput,
                -MAX_TRANSLATION_VELOCITY.in(MetersPerSecond),
                MAX_TRANSLATION_VELOCITY.in(MetersPerSecond)));

    Rotation2d angleToTarget =
        targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    this.angleToTarget = angleToTarget;

    // Convert distanceOutput into a 2D velocity vector pointing toward the target
    Translation2d translationOutput =
        new Translation2d(clampedDistanceOutput.in(MetersPerSecond), angleToTarget);

    // Assign to instance variables for dashboard updates
    this.currentPose = currentPose;
    this.targetPose = targetPose;

    this.rotationError =
        Degrees.of(targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());
    this.translationOutput = translationOutput;

    return driveAtAngle
        .withVelocityX(translationOutput.getX())
        .withVelocityY(translationOutput.getY())
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withTargetDirection(targetPose.getRotation());
  }

  /** Checks if the robot is aligned to the current target within tolerance. */
  public boolean isAligned() {
    if (currentPose == null || targetPose == null) {
      return false;
    }

    Translation2d error = targetPose.getTranslation().minus(currentPose.getTranslation());
    Translation2d tagRelativeError = error.rotateBy(targetPose.getRotation().unaryMinus());

    this.normalError = Meters.of(tagRelativeError.getX());
    this.tangentError = Meters.of(tagRelativeError.getY());

    return tagRelativeError != null
        && normalError.abs(Meters) < NORMAL_TOLERANCE.in(Meters)
        && tangentError.abs(Meters) < TANGENT_TOLERANCE.in(Meters)
        && rotationError != null
        && rotationError.abs(Degree) < ROTATION_TOLERANCE.in(Degrees);
  }

  /** Updates dashboard with current alignment status. */
  public void updateDashboard() {
    Logger.recordOutput("AutoAlign/Aligned", isAligned());
    Logger.recordOutput("Drive/AutoAlign/Goal", goal);
    Logger.recordOutput("Drive/AutoAlign/IsAligning", isAligning);

    if (rotationError != null) {
      Logger.recordOutput("Drive/AutoAlign/Rotation Error", rotationError.in(Degrees));
    }
    if (tangentError != null) {
      Logger.recordOutput("Drive/AutoAlign/Tangent Error", tangentError.in(Meters));
    }
    if (normalError != null) {
      Logger.recordOutput("Drive/AutoAlign/Normal Error", normalError.in(Meters));
    }
    if (translationOutput != null) {
      Logger.recordOutput("Drive/AutoAlign/Outputs/Linear Velocity", translationOutput.getNorm());
      Logger.recordOutput("Drive/AutoAlign/Outputs/Translation Velocity", translationOutput);
    }
    if (angleToTarget != null) {
      Logger.recordOutput("Drive/AutoAlign/AngleToTarget", angleToTarget);
    }

    if (currentPose != null) {
      Logger.recordOutput("Drive/AutoAlign/Current Pose", currentPose);
    }

    if (targetPose != null) {
      Logger.recordOutput("Drive/AutoAlign/Target Pose", targetPose);
    }
  }
}

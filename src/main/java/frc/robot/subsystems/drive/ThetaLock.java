package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class ThetaLock {
  private final CommandSwerveDrivetrain drivetrain;
  private final Manual manual;

  public enum Goal {
    NONE,
    ACTIVE
  };

  @Getter @Setter private Goal goal = Goal.NONE;

  private static final double SNAP_ROTATION_KP = 5.0;
  private static final double SNAP_ROTATION_KI = 0.0;
  private static final double SNAP_ROTATION_KD = 0.0;

  private final SwerveRequest.FieldCentricFacingAngle driveAtAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
          .withHeadingPID(SNAP_ROTATION_KP, SNAP_ROTATION_KI, SNAP_ROTATION_KD);

  public ThetaLock(CommandSwerveDrivetrain drivetrain, Manual manual) {
    this.drivetrain = drivetrain;
    this.manual = manual;
  }

  public void periodic() {
    Logger.recordOutput("Drive/ThetaLock/Goal", goal);

    if (goal == Goal.NONE) {
      return;
    }

    Optional<Rotation2d> desiredRotation =
        switch (goal) {
          case ACTIVE -> Optional.of(new Rotation2d());
          default -> Optional.empty();
        };

    if (desiredRotation.isEmpty()) {
      return;
    }

    Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
    double rotationError = Math.abs(desiredRotation.get().minus(currentRotation).getDegrees());

    Logger.recordOutput("Drive/ThetaLock/TargetAngle", desiredRotation.get().getDegrees());
    Logger.recordOutput("Drive/ThetaLock/CurrentAngle", currentRotation.getDegrees());
    Logger.recordOutput("Drive/ThetaLock/RotationError", rotationError);

    drivetrain.setControl(
        driveAtAngle
            .withVelocityX(manual.getValues().getX())
            .withVelocityY(manual.getValues().getY())
            .withTargetDirection(desiredRotation.get())
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance));
  }
}

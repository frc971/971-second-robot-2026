package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;
import org.littletonrobotics.junction.Logger;

public class Limelight {
  public static String[] LIMELIGHT_NAMES = {"limelight-front"};
  public static Transform3d[] LIMELIGHT_EXTRINSICS = {
    new Transform3d(0.307057, 0.020606, 0.467946, Rotation3d.kZero)
  };
  public static int LIMELIGHT_LOCALIZATION_PIPELINE = 0;
  public static AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(2.0);
  public static Distance MAX_DISPLACEMENT = Meters.of(3.0);
  public static int MINIMUM_APRIL_TAG_COUNT = 1;

  private final CommandSwerveDrivetrain drivetrain;
  private Rotation2d headingOffset = Rotation2d.kZero;

  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    for (int i = 0; i < LIMELIGHT_NAMES.length; i++) {
      String name = LIMELIGHT_NAMES[i];
      Transform3d extrinsics = LIMELIGHT_EXTRINSICS[i];

      LimelightHelpers.setPipelineIndex(name, LIMELIGHT_LOCALIZATION_PIPELINE);
      LimelightHelpers.setCameraPose_RobotSpace(
          name,
          extrinsics.getX(),
          extrinsics.getY(),
          extrinsics.getZ(),
          Math.toDegrees(extrinsics.getRotation().getX()),
          Math.toDegrees(extrinsics.getRotation().getY()),
          Math.toDegrees(extrinsics.getRotation().getZ()));
    }
  }

  public void resetHeading(Rotation2d heading) {
    headingOffset = heading.minus(drivetrain.getPigeon2().getRotation2d());
    Logger.recordOutput("Limelight/HeadingOffsetDegrees", headingOffset.getDegrees());
  }

  public void updatePose() {
    SwerveDriveState driveState = drivetrain.getState();

    Rotation2d heading = drivetrain.getPigeon2().getRotation2d().plus(headingOffset);
    Logger.recordOutput("Gyro", drivetrain.getPigeon2().getRotation2d());
    Logger.recordOutput("Limelight/HeadingDegrees", heading.getDegrees());
    AngularVelocity omega = RadiansPerSecond.of(Math.abs(driveState.Speeds.omegaRadiansPerSecond));

    for (String name : LIMELIGHT_NAMES) {
      LimelightHelpers.SetRobotOrientation(name, heading.getDegrees(), 0, 0, 0, 0, 0);

      PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

      boolean accept =
          estimate != null
              && estimate.tagCount >= MINIMUM_APRIL_TAG_COUNT
              && omega.lte(MAX_ANGULAR_SPEED)
              && driveState.Pose.minus(estimate.pose).getTranslation().getNorm()
                  < MAX_DISPLACEMENT.in(Meters);

      if (accept) {
        double averageAmbiguity = getAverageAmbiguity(estimate);
        double linearStdDev = 0.3 * estimate.avgTagDist;
        drivetrain.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            VecBuilder.fill(linearStdDev, linearStdDev, 999999));

        Logger.recordOutput("Limelight/" + name + "/Accepted Estimated Pose", estimate.pose);
        Logger.recordOutput("Limelight/" + name + "/Average Tag Ambiguity", averageAmbiguity);
        Logger.recordOutput("Limelight/" + name + "/Linear StdDev", linearStdDev);
      }

      Logger.recordOutput("Limelight/" + name + "/Accepted", accept);
      if (estimate != null && estimate.tagCount > 0) {
        Logger.recordOutput("Limelight/" + name + "/Estimated Pose", estimate.pose);
        Logger.recordOutput("Limelight/" + name + "/Latency", estimate.latency);
        Logger.recordOutput("Limelight/" + name + "/Average Tag Distance", estimate.avgTagDist);
        Logger.recordOutput("Limelight/" + name + "/Tag Count", estimate.tagCount);
      }
    }
  }

  private static double getAverageAmbiguity(PoseEstimate estimate) {
    if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
      return 0.0;
    }

    double totalAmbiguity = 0.0;
    for (RawFiducial fiducial : estimate.rawFiducials) {
      totalAmbiguity += fiducial.ambiguity;
    }

    return totalAmbiguity / estimate.rawFiducials.length;
  }
}
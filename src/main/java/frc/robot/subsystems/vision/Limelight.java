package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

public class Limelight {
  public static String[] LIMELIGHT_NAMES = {"limelight"};
  public static int LIMELIGHT_LOCALIZATION_PIPELINE = 0;
  public static AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(2.0);
  public static int MINIMUM_APRIL_TAG_COUNT = 1;

  private final CommandSwerveDrivetrain drivetrain;

  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public void updatePose() {
    SwerveDriveState driveState = drivetrain.getState();

    Rotation2d heading = driveState.Pose.getRotation();
    AngularVelocity omega = RadiansPerSecond.of(Math.abs(driveState.Speeds.omegaRadiansPerSecond));

    for (String limelightName : LIMELIGHT_NAMES) {
      LimelightHelpers.setPipelineIndex(limelightName, LIMELIGHT_LOCALIZATION_PIPELINE);
      LimelightHelpers.SetRobotOrientation(limelightName, heading.getDegrees(), 0, 0, 0, 0, 0);

      PoseEstimate llMeasurement =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

      boolean accept =
          llMeasurement != null
              && llMeasurement.tagCount >= MINIMUM_APRIL_TAG_COUNT
              && omega.lte(MAX_ANGULAR_SPEED);

      if (accept) {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }

      Logger.recordOutput(limelightName + "/Accepted", accept);
      if (llMeasurement != null) {
        Logger.recordOutput(limelightName + "/Estimated Pose", llMeasurement.pose);
        Logger.recordOutput(limelightName + "/Latency", llMeasurement.latency);
        Logger.recordOutput(limelightName + "/Average Tag Distance", llMeasurement.avgTagDist);
        Logger.recordOutput(limelightName + "/Tag Count", llMeasurement.tagCount);
      }
    }
  }
}

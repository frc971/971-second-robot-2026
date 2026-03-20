package frc.robot.subsystems.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class Limelight {

  public static String[] LIMELIGHT_NAMES = {"limelight-left", "limelight-right"};
  public static int LIMELIGHT_LOCALIZATION_PIPELINE = 0;
  public static double MAX_OMEGA_RPS_THRESHOLD = 2.0;
  public static int MINIMUM_APRIL_TAG_COUNT = 0;

  private final CommandSwerveDrivetrain drivetrain;

  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  /*
   * This example of adding Limelight is very simple and may not be sufficient for
   * on-field use.
   * Users typically need to provide a standard deviation that scales with the
   * distance to target
   * and changes with number of tags available.
   *
   * This example is sufficient to show that vision integration is possible,
   * though exact implementation
   * of how to use vision should be tuned per-robot and to the team's
   * specification.
   */
  public void updatePose() {
    var driveState = drivetrain.getState();
    double headingDeg = driveState.Pose.getRotation().getDegrees();
    double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    for (String limelightName : LIMELIGHT_NAMES) {
      LimelightHelpers.setPipelineIndex(limelightName, LIMELIGHT_LOCALIZATION_PIPELINE);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
      if (llMeasurement != null
          && llMeasurement.tagCount > MINIMUM_APRIL_TAG_COUNT
          && Math.abs(omegaRps) < MAX_OMEGA_RPS_THRESHOLD) {
        // TODO: Possibly find the correct values for the std dev in the future since these were
        // guessed
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, .7));
        drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        Logger.recordOutput(limelightName + "/Estimated Pose", llMeasurement.pose);
        Logger.recordOutput(limelightName + "/Latency", llMeasurement.latency);
        Logger.recordOutput(limelightName + "/Average Tag Distance", llMeasurement.avgTagDist);
        Logger.recordOutput(limelightName + "/Num Tags Detected", llMeasurement.tagCount);
      }
    }
  }
}

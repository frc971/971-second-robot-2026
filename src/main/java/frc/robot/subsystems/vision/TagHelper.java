package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;

public class TagHelper {
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private TagHelper() {}

  // IMPORTANT: DO NOT DELETE OR EXPENSIVE FILE READING WILL FREEZE WHEN FIRST TAG HELPER FUNCTION
  // IS CALLED
  public static final void init() {}

  public static final Optional<Pose2d> getClosestTagPose(
      CommandSwerveDrivetrain drivetrain, int[] tagIds) {
    Translation2d currentTranslation = drivetrain.getState().Pose.getTranslation();
    int closestTagId = -1;
    double minimumDistance = Double.MAX_VALUE;

    for (int id : tagIds) {
      Optional<Pose3d> optionalPose = TAG_LAYOUT.getTagPose(id);
      if (optionalPose.isPresent()) {
        double distance =
            optionalPose.get().toPose2d().getTranslation().getDistance(currentTranslation);
        if (distance < minimumDistance) {
          minimumDistance = distance;
          closestTagId = id;
        }
      }
    }

    if (closestTagId == -1) {
      System.out.println("Warning: could not find april tag ids. This shouldn't happen");
      return Optional.empty();
    }

    return Optional.of(TAG_LAYOUT.getTagPose(closestTagId).get().toPose2d());
  }
}

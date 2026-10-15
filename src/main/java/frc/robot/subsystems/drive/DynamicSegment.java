package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.util.Set;

public abstract class DynamicSegment implements Segment {
  abstract Path getDynamicPath();

  public Command getCommand(FollowPath.Builder builder) {
    return Commands.defer(() -> builder.build(getDynamicPath()), Set.of());
  }

  // The first segment is expected to have a valid start position. To simplify
  // things, we currently don't support DynamicSegments as the first segment, so
  // we don't need to handle star positions
  public Pose2d getStartPose(boolean mirrored) {
    return null;
  }
}

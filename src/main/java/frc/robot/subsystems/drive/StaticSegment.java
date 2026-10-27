package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import lombok.*;

public class StaticSegment implements Segment {

  @Getter private final Path path;

  public StaticSegment(Path path) {
    this.path = path;
  }

  public Command getCommand(FollowPath.Builder builder) {
    return builder.build(path);
  }

  public Pose2d getStartPose(boolean mirrored) {
    Path startPath = path.copy();
    if (mirrored) startPath.mirror();
    return startPath.getStartPose();
  }

  public Path copy() {
    return path.copy();
  }
}

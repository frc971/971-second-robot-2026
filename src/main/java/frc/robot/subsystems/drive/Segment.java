package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

interface Segment {
  Command getCommand(FollowPath.Builder builder);

  Pose2d getStartPose(boolean mirrored);

  Path copy();
}

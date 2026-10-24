package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.pathing.Pair;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public abstract class DynamicSegment implements Segment {
  private final double MAX_SPEED = 4.5;

  abstract Path getDynamicPath();

  public Command getCommand(FollowPath.Builder builder) {
    return Commands.defer(() -> builder.build(getDynamicPath()), Set.of());
  }

  private double area(Pose2d a, Pose2d b, Pose2d c) {
    return (0.5
        * Math.abs(
            (a.getX() * (b.getY() - c.getY()))
                + (b.getX() * (c.getY() - a.getY()))
                + (c.getX() * (a.getY() - b.getY()))));
  }

  protected Pair<List<Path.PathElement>, List<Path.RangedConstraint>> removeRedundantPoints(
      List<Pose2d> path, Rotation2d rotation) {
    // uses menger curvature to figure out which points are actually important, so
    // pid doesn't have to navigate redundant points
    List<Path.PathElement> needed = new ArrayList<>();
    Pose2d first = path.get(0);
    Pose2d second = path.get(1);
    // Makes the intake point in the direction where the robot is going.
    needed.add(
        new Path.Waypoint(
            new Pose2d(
                first.getX(),
                first.getY(),
                rotation != null
                    ? rotation
                    : new Rotation2d(
                        Math.atan2(second.getY() - first.getY(), second.getX() - first.getX())
                            + Math.PI))));
    List<Path.RangedConstraint> speeds = new ArrayList<>();
    for (int i = 1; i < path.size() - 1; i++) {
      Pose2d a = path.get(i - 1);
      Pose2d b = path.get(i);
      Pose2d c = path.get(i + 1);

      double sideAB = Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
      double sideBC = Math.hypot(b.getX() - c.getX(), b.getY() - c.getY());
      double sideCA = Math.hypot(c.getX() - a.getX(), c.getY() - a.getY());
      double curvature;

      if (sideAB == 0 || sideBC == 0 || sideCA == 0) {
        curvature = 0.0;
        continue;
      }
      curvature = (4 * area(a, b, c)) / (sideAB * sideBC * sideCA);
      if (curvature >= 0.0000001) {
        speeds.add(new Path.RangedConstraint(MAX_SPEED * 0.3, needed.size(), needed.size()));
      } else {
        speeds.add(new Path.RangedConstraint(MAX_SPEED, needed.size(), needed.size()));
      }
      needed.add(
          new Path.Waypoint(
              new Pose2d(
                  b.getX(),
                  b.getY(),
                  rotation != null
                      ? rotation
                      : new Rotation2d(
                          Math.atan2(c.getY() - b.getY(), c.getX() - b.getX()) + Math.PI))));
    }

    Pose2d secondLast = path.get(path.size() - 2);
    Pose2d last = path.get(path.size() - 1);
    needed.add(
        new Path.Waypoint(
            new Pose2d(
                last.getX(),
                last.getY(),
                rotation != null
                    ? rotation
                    : new Rotation2d(
                        Math.atan2(last.getY() - secondLast.getY(), last.getX() - secondLast.getX())
                            + Math.PI))));

    return new Pair<>(needed, speeds);
  }

  // The first segment is expected to have a valid start position. To simplify
  // things, we currently don't support DynamicSegments as the first segment, so
  // we don't need to handle star positions
  public Pose2d getStartPose(boolean mirrored) {
    return null;
  }
}

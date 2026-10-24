package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.pathing.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class DynamicSegmentPOC extends DynamicSegment {

  private final double MAX_SPEED = 4.5;

  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable table = instance.getTable("DriveState");
  private double blank = -1;
  private DoubleSubscriber clusterSubscriberX =
      table.getDoubleTopic("ClusterPositionsX").subscribe(blank);
  private DoubleSubscriber clusterSubscriberY =
      table.getDoubleTopic("ClusterPositionsY").subscribe(blank);

  private final CommandSwerveDrivetrain drivetrain;

  public DynamicSegmentPOC(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  private double area(Pose2d a, Pose2d b, Pose2d c) {
    return (0.5
        * Math.abs(
            (a.getX() * (b.getY() - c.getY()))
                + (b.getX() * (c.getY() - a.getY()))
                + (c.getX() * (a.getY() - b.getY()))));
  }

  private Pair<List<Path.PathElement>, List<Path.RangedConstraint>> removeRedundantPoints(
      List<Pose2d> path) {
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
                new Rotation2d(
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
        speeds.add(new Path.RangedConstraint(MAX_SPEED * 0.7, needed.size(), needed.size()));
      } else {
        speeds.add(new Path.RangedConstraint(MAX_SPEED, needed.size(), needed.size()));
      }
      needed.add(
          new Path.Waypoint(
              new Pose2d(
                  b.getX(),
                  b.getY(),
                  new Rotation2d(Math.atan2(c.getY() - b.getY(), c.getX() - b.getX()) + Math.PI))));
    }

    Pose2d secondLast = path.get(path.size() - 2);
    Pose2d last = path.get(path.size() - 1);
    needed.add(
        new Path.Waypoint(
            new Pose2d(
                last.getX(),
                last.getY(),
                new Rotation2d(
                    Math.atan2(last.getY() - secondLast.getY(), last.getX() - secondLast.getX())
                        + Math.PI))));

    return new Pair<>(needed, speeds);
  }

  public Path getDynamicPath() {

    // TODO: flipping flips the navgrid s the coorinates become weird for pathing
    NavGrid navGrid =
        Pathfinding.GetGrid(
            new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json").getPath());
    Pose2d start = drivetrain.getState().Pose;
    Translation2d destination =
        new Translation2d(clusterSubscriberX.get(), clusterSubscriberY.get());
    double nodeSize = navGrid.nodeSizeMeters;
    List<Node> path =
        Pathfinding.BFS(
            navGrid.grid,
            new Point(
                (int) Math.round(start.getX() / nodeSize),
                (int) Math.round(start.getY() / nodeSize)),
            new Point(
                (int) Math.round(destination.getX() / nodeSize),
                (int) Math.round(destination.getY() / nodeSize)));
    List<Pose2d> points = new ArrayList<>();
    for (Node node : path) {
      points.add(new Pose2d(node.x * nodeSize, node.y * nodeSize, Rotation2d.kZero));
    }

    Pair<List<Path.PathElement>, List<Path.RangedConstraint>> returned =
        removeRedundantPoints(points);
    List<Path.PathElement> elements = returned.first;
    List<Path.RangedConstraint> speeds = returned.second;

    List<Node> forPrinting = new ArrayList<>();
    for (Path.PathElement element : elements) {
      Node node = new Node();
      Path.Waypoint translationTarget = (Path.Waypoint) (element);
      node.x =
          (int) Math.round(translationTarget.translationTarget().translation().getX() / nodeSize);
      node.y =
          (int) Math.round(translationTarget.translationTarget().translation().getY() / nodeSize);
      forPrinting.add(node);
    }

    Pathfinding.printField(navGrid.grid, forPrinting);

    return new Path(
        elements,
        new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(speeds.toArray(new Path.RangedConstraint[0])));
  }

  public Path copy() {
    return new Path();
  }
}

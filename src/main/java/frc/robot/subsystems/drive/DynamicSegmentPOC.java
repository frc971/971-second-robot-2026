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
        removeRedundantPoints(points, null);
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

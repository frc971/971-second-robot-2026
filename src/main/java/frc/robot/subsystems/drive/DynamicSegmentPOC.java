package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.pathing.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.Rotation;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class DynamicSegmentPOC extends DynamicSegment {
  private static int counter = 0;

  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable table = instance.getTable("DriveState");
  private double blank = -1;
  private DoubleSubscriber clusterSubscriberX = table.getDoubleTopic("ClusterPositionsX").subscribe(blank);
  private DoubleSubscriber clusterSubscriberY = table.getDoubleTopic("ClusterPositionsY").subscribe(blank);

  private final CommandSwerveDrivetrain drivetrain;

  public DynamicSegmentPOC(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public Path getDynamicPath() {
    counter++;

    // TODO: flipping flips the navgrid s the coorinates become weird for pathing
    NavGrid navGrid = Pathfinding.GetGrid(
        new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json").getPath());
    Pose2d start = drivetrain.getState().Pose;
    Translation2d destination = new Translation2d(clusterSubscriberX.get(), clusterSubscriberY.get());
    double nodeSize = navGrid.nodeSizeMeters;
    List<Node> path = Pathfinding.BFS(navGrid.grid,
        new Point((int) Math.round(start.getX() / nodeSize), (int) Math.round(start.getY() / nodeSize)),
        new Point((int) Math.round(destination.getX() / nodeSize),
            (int) Math.round(destination.getY() / nodeSize)));
    List<Pose2d> points = new ArrayList<>();
    int count = 0;
    for (Node node : path) {
      if (count % 2 == 0) {
        points.add(new Pose2d(node.x * nodeSize, node.y * nodeSize, Rotation2d.kZero));

      }
      count++;
    }

    System.out.println("Computing path to destination (" + counter + ", " + counter + ")");

    List<Path.PathElement> elements = new ArrayList<>();
    for (Pose2d point : points) {
      Translation2d p = new Translation2d(point.getX(), point.getY());
      elements.add(new Path.Waypoint(p, Rotation2d.k180deg));
    }
    return new Path(elements, new Path.PathConstraints().setMaxVelocityMetersPerSec(3));
  }

  public Path copy() {
    return new Path();
  }
}

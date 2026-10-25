package frc.robot.subsystems.drive;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.pathing.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class DynamicSegmentHub extends DynamicSegment {

  private final CommandSwerveDrivetrain drivetrain;

  private final boolean mirrored;

  public DynamicSegmentHub(CommandSwerveDrivetrain drivetrain, boolean mirrored) {
    this.drivetrain = drivetrain;
    this.mirrored = mirrored;
  }

  public Path getDynamicPath() {

    // TODO: flipping flips the navgrid s the coorinates become weird for pathing
    NavGrid navGrid =
        Pathfinding.GetGrid(
            new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json").getPath());
    Pose2d start = drivetrain.getState().Pose;
    // hardcoded start end position, the start position of all the S_ autos (blue side coordinates)
    Translation2d destination = new Translation2d(4.149, 7.492);
    Rotation2d endRotation = Rotation2d.kCW_90deg;
    // mirror across the long middle line of the field (y -> fieldSizeY - y), done in blue
    // coordinates before the alliance flip
    if (mirrored) {
      destination =
          new Translation2d(destination.getX(), FlippingUtil.fieldSizeY - destination.getY());
      endRotation = endRotation.unaryMinus();
    }
    // dynamic paths are built in real field coordinates and not flipped by the builder, so flip
    // the blue side target ourselves on red
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      destination = FlippingUtil.flipFieldPosition(destination);
      endRotation = FlippingUtil.flipFieldRotation(endRotation);
    }
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
        removeRedundantPoints(points, endRotation);
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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.pathing.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class DynamicSegmentHub extends DynamicSegment {

  private final Supplier<Pose2d> robotPose;

  private final boolean mirrored;
  private final boolean trenchReturn;

  public DynamicSegmentHub(CommandSwerveDrivetrain drivetrain, boolean mirrored) {
    this(drivetrain, mirrored, false);
  }

  public DynamicSegmentHub(
      CommandSwerveDrivetrain drivetrain, boolean mirrored, boolean trenchReturn) {
    this(() -> drivetrain.getState().Pose, mirrored, trenchReturn);
  }

  DynamicSegmentHub(Supplier<Pose2d> robotPose, boolean mirrored, boolean trenchReturn) {
    this.robotPose = robotPose;
    this.mirrored = mirrored;
    this.trenchReturn = trenchReturn;
  }

  public Path getDynamicPath() {

    // TODO: flipping flips the navgrid s the coorinates become weird for pathing
    NavGrid navGrid = Pathfinding.getNavGrid();
    Pose2d start = robotPose.get();
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

    return new Path(
        elements,
        new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(speeds.toArray(new Path.RangedConstraint[0])));
  }

  private Pose2d fieldPose(double x, double y, Rotation2d rotation) {
    var position = new Translation2d(x, mirrored ? FlippingUtil.fieldSizeY - y : y);
    if (mirrored) rotation = rotation.unaryMinus();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      position = FlippingUtil.flipFieldPosition(position);
      rotation = FlippingUtil.flipFieldRotation(rotation);
    }
    return new Pose2d(position, rotation);
  }

  @Override
  public Command getCommand(FollowPath.Builder builder) {
    if (!trenchReturn) return super.getCommand(builder);
    return Commands.defer(
        () -> {
          // Either longitudinal heading fits the trench; choose the shorter turn.
          var heading =
              Math.cos(robotPose.get().getRotation().getRadians()) >= 0
                  ? Rotation2d.kZero
                  : Rotation2d.k180deg;
          var turn = new Pose2d(fieldPose(6.6, 6.9, Rotation2d.kZero).getTranslation(), heading);
          var lane = new Pose2d(fieldPose(6.6, 7.35, Rotation2d.kZero).getTranslation(), heading);
          var shooting =
              new Pose2d(fieldPose(3.1, 7.35, Rotation2d.kZero).getTranslation(), heading);
          var start = robotPose.get();
          var aligned = new Pose2d(start.getTranslation(), turn.getRotation());
          // Each leg settles before the next, keeping rotation out of the trench lane.
          return Commands.sequence(
              builder.build(returnLeg(start, aligned, 0.7)),
              builder.build(returnLeg(aligned, turn, 2.8)),
              Commands.waitSeconds(0.2),
              builder.build(returnLeg(turn, lane, 0.7)),
              Commands.waitSeconds(0.2),
              builder.build(returnLeg(lane, shooting, 2.8)));
        },
        Set.of());
  }

  private Path returnLeg(Pose2d start, Pose2d end, double maxSpeed) {
    return new Path(
        List.of(new Path.Waypoint(start), new Path.Waypoint(end)),
        new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(maxSpeed)
            .setMaxAccelerationMetersPerSec2(3.0)
            .setMaxVelocityDegPerSec(180)
            .setMaxAccelerationDegPerSec2(360));
  }

  public Path copy() {
    return new Path();
  }
}

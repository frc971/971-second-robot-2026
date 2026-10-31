package frc.robot.lib.simulation;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Synthetic COS detections for desktop simulation; no camera or inference model is involved. */
public class CosBallPathingSim implements AutoCloseable {
  private final Supplier<Pose2d> pose;
  private final NetworkTable controls;
  private final DoubleArrayPublisher publisher;
  private final Field2d field = new Field2d();
  private final Supplier<java.util.List<Translation2d>> balls;
  private Translation2d selected;
  private boolean mirrored;
  private int targetIndex;

  public CosBallPathingSim(Supplier<Pose2d> pose, NetworkTableInstance instance) {
    this(pose, instance, null);
  }

  public CosBallPathingSim(
      Supplier<Pose2d> pose,
      NetworkTableInstance instance,
      Supplier<java.util.List<Translation2d>> balls) {
    this.pose = pose;
    this.balls = balls;
    controls = instance.getTable("Sim/COS");
    controls.getEntry("Layout").setDefaultInteger(-1);
    controls.getEntry("Enabled").setDefaultBoolean(true);
    controls.getEntry("Detected").setDefaultBoolean(true);
    controls.getEntry("ClusterX").setDefaultDouble(FlippingUtil.fieldSizeX / 2);
    controls.getEntry("ClusterY").setDefaultDouble(FlippingUtil.fieldSizeY / 2);
    publisher =
        instance
            .getTable("COS")
            .getDoubleArrayTopic("BallVelocityGoal")
            .publish(PubSubOption.keepDuplicates(true), PubSubOption.periodic(0.02));
    SmartDashboard.putData("COS Simulation", field);
  }

  public void reset(boolean mirrored) {
    this.mirrored = mirrored;
    targetIndex = 0;
    selected = null;
  }

  private Translation2d target(int index) {
    Translation2d point =
        new Translation2d(
            controls.getEntry("ClusterX").getDouble(FlippingUtil.fieldSizeX / 2)
                + (index - 1) * 0.4,
            controls.getEntry("ClusterY").getDouble(FlippingUtil.fieldSizeY / 2));
    if (mirrored) point = new Translation2d(point.getX(), FlippingUtil.fieldSizeY - point.getY());
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red
        ? FlippingUtil.flipFieldPosition(point)
        : point;
  }

  public void periodic() {
    Pose2d robot = pose.get();
    if (balls != null) {
      publishBalls(robot);
      return;
    }
    var cluster = new Pose2d[9];
    for (int i = 0; i < cluster.length; i++) {
      Translation2d center = target(i / 3);
      cluster[i] =
          new Pose2d(center.plus(new Translation2d(0, (i % 3 - 1) * 0.16)), Rotation2d.kZero);
    }
    if (targetIndex < 2 && robot.getTranslation().getDistance(target(targetIndex)) < 0.3)
      targetIndex++;
    Translation2d destination = target(targetIndex);
    field.setRobotPose(robot);
    field.getObject("YellowCluster").setPoses(cluster);
    field.getObject("Target").setPose(new Pose2d(destination, Rotation2d.kZero));
    Logger.recordOutput("Sim/COS/YellowCluster", cluster);
    Logger.recordOutput("Sim/COS/Target", new Pose2d(destination, Rotation2d.kZero));
    Logger.recordOutput("Sim/COS/TargetIndex", targetIndex);
    if (controls.getEntry("Enabled").getBoolean(true)) {
      publisher.set(goal(robot, destination, controls.getEntry("Detected").getBoolean(true)));
    }
  }

  private void publishBalls(Pose2d robot) {
    boolean red =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;
    var visible =
        balls.get().stream()
            .filter(
                p -> {
                  double x = red ? FlippingUtil.fieldSizeX - p.getX() : p.getX();
                  return x >= 5.95
                      && x <= FlippingUtil.fieldSizeX / 2 - 0.30
                      && p.getY() >= 0.55
                      && p.getY() <= FlippingUtil.fieldSizeY - 0.55;
                })
            .toList();
    // Keep the selected ball until it disappears; switching on every frame causes oscillation.
    if (selected == null || visible.stream().noneMatch(p -> p.getDistance(selected) < 0.18)) {
      selected =
          visible.stream()
              .min(
                  java.util.Comparator.comparingDouble(
                      p ->
                          robot.getTranslation().getDistance(p)
                              / (1
                                  + visible.stream().filter(q -> q.getDistance(p) < 0.6).count()
                                      * 0.15)))
              .orElse(null);
    } else {
      selected =
          visible.stream()
              .min(java.util.Comparator.comparingDouble(p -> p.getDistance(selected)))
              .orElse(null);
    }
    var markers = visible.stream().map(p -> new Pose2d(p, Rotation2d.kZero)).toArray(Pose2d[]::new);
    field.setRobotPose(robot);
    field.getObject("YellowCluster").setPoses(markers);
    field
        .getObject("Target")
        .setPoses(
            selected == null
                ? new Pose2d[0]
                : new Pose2d[] {new Pose2d(selected, Rotation2d.kZero)});
    Logger.recordOutput("Sim/COS/YellowCluster", markers);
    Logger.recordOutput(
        "Sim/COS/Target",
        selected == null ? new Pose2d[0] : new Pose2d[] {new Pose2d(selected, Rotation2d.kZero)});
    if (controls.getEntry("Enabled").getBoolean(true)) {
      // Stop the robot with the ball inside the rear intake, rather than under its bumper.
      var target = selected == null ? robot.getTranslation() : selected;
      var values =
          goal(robot, target, selected != null && controls.getEntry("Detected").getBoolean(true));
      double distance = target.getDistance(robot.getTranslation());
      double speed =
          Math.min(
              frc.robot.subsystems.drive.Autos.MAX_SPEED_METERS_PER_SECOND,
              Math.max(0, distance - 0.50) * 3);
      if (distance > 0) {
        values[0] = values[2] / distance * speed;
        values[1] = values[3] / distance * speed;
      }
      publisher.set(values);
    }
  }

  static double[] goal(Pose2d robot, Translation2d target, boolean detected) {
    Translation2d relative =
        target.minus(robot.getTranslation()).rotateBy(robot.getRotation().unaryMinus());
    double distance = relative.getNorm();
    if (!detected || !Double.isFinite(distance)) return new double[10];
    boolean moving = distance > 0.15;
    // Intake is on the rear: translate toward the target while pointing -X toward it.
    double speed = moving ? Math.min(1.5, distance * 2) : 0;
    double omega =
        moving
            ? MathUtil.clamp(
                2 * MathUtil.angleModulus(relative.getAngle().getRadians() + Math.PI), -2, 2)
            : 0;
    return new double[] {
      moving ? relative.getX() / distance * speed : 0,
      moving ? relative.getY() / distance * speed : 0,
      relative.getX(),
      relative.getY(),
      distance,
      1,
      0,
      0,
      0,
      omega
    };
  }

  public void close() {
    publisher.close();
  }
}

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
  private boolean mirrored;
  private int targetIndex;

  public CosBallPathingSim(Supplier<Pose2d> pose, NetworkTableInstance instance) {
    this.pose = pose;
    controls = instance.getTable("Sim/COS");
    controls.getEntry("Enabled").setDefaultBoolean(true);
    controls.getEntry("Detected").setDefaultBoolean(true);
    controls.getEntry("ClusterX").setDefaultDouble(6.2);
    controls.getEntry("ClusterY").setDefaultDouble(6.8);
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
  }

  private Translation2d target(int index) {
    Translation2d point =
        new Translation2d(
            controls.getEntry("ClusterX").getDouble(6.2) + (index - 1) * 0.4,
            controls.getEntry("ClusterY").getDouble(6.8));
    if (mirrored) point = new Translation2d(point.getX(), FlippingUtil.fieldSizeY - point.getY());
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red
        ? FlippingUtil.flipFieldPosition(point)
        : point;
  }

  public void periodic() {
    Pose2d robot = pose.get();
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

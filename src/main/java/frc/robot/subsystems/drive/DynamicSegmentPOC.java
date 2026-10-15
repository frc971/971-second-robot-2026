package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.lib.BLine.Path;
import java.util.ArrayList;
import java.util.List;

public class DynamicSegmentPOC extends DynamicSegment {
  private static int counter = 0;

  NetworkTableInstance instance = NetworkTableInstance.getDefault();
  NetworkTable table = instance.getTable("DriveState");
  double blank = -1;
  DoubleSubscriber clusterSubscriberX = table.getDoubleTopic("ClusterPositionsX").subscribe(blank);
  DoubleSubscriber clusterSubscriberY = table.getDoubleTopic("ClusterPositionsY").subscribe(blank);

  public Path getDynamicPath() {
    counter++;
    Translation2d destination =
        new Translation2d(clusterSubscriberX.get(), clusterSubscriberY.get());

    System.out.println("Computing path to target (" + counter + ", " + counter + ")");

    List<Path.PathElement> elements = new ArrayList<>();
    elements.add(new Path.Waypoint(destination, Rotation2d.kZero));
    return new Path(elements, new Path.PathConstraints().setMaxVelocityMetersPerSec(4.5));
  }

  public Path copy() {
    return new Path();
  }
}

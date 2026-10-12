package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.Path;
import java.util.ArrayList;
import java.util.List;

public class DynamicSegmentPOC extends DynamicSegment {
  private static int counter = 0;

  Path getDynamicPath() {
    counter++;
    Translation2d destination = new Translation2d(1.0 + counter, 1.0 + counter);

    List<Path.PathElement> elements = new ArrayList<>();
    elements.add(new Path.Waypoint(destination, Rotation2d.kZero));
    return new Path(elements, new Path.PathConstraints().setMaxVelocityMetersPerSec(4.5));
  }

  public Path copy() {
    return new Path();
  }
}

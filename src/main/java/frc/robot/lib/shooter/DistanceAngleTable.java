package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.TreeSet;

public class DistanceAngleTable {
  private final InterpolatingDoubleTreeMap angleTable = new InterpolatingDoubleTreeMap();
  private final TreeSet<Distance> distances = new TreeSet<>();

  public void put(Distance distance, Angle angle) {
    distances.add(distance);
    angleTable.put(distance.in(Meters), angle.in(Degrees));
  }

  public Angle get(Distance distance) {
    return Degrees.of(angleTable.get(distance.in(Meters)));
  }

  public boolean hasEntries() {
    return !distances.isEmpty();
  }
}

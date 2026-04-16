package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import java.util.TreeSet;

public class DistanceDistanceTable {
  private final InterpolatingDoubleTreeMap distanceTable = new InterpolatingDoubleTreeMap();
  private final TreeSet<Distance> distances = new TreeSet<>();

  public void put(Distance distance, Distance value) {
    distances.add(distance);
    distanceTable.put(distance.in(Meters), value.in(Meters));
  }

  public Distance get(Distance distance) {
    return Meters.of(distanceTable.get(distance.in(Meters)));
  }

  public boolean hasEntries() {
    return !distances.isEmpty();
  }
}

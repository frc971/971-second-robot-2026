package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import java.util.Set;
import java.util.TreeSet;

/*
 * ShotTable class to manage shooter data (hood angle and flywheel speed) based on distance to target
 * Uses separate interpolation tables for angle and speed
 */
public class ShotTable {
  public record ShooterData(Angle hoodAngle, AngularVelocity flywheelSpeed) {}

  /*
   * Separate interpolation tables for hood angle and flywheel speed
   * Key: distance to target (in meters)
   * Value: hood angle (degrees) or flywheel speed (rot/s)
   */
  private final InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flywheelSpeedTable = new InterpolatingDoubleTreeMap();

  private final Set<Double> distances = new TreeSet<>();

  public ShotTable() {}

  public void put(Distance distance, Angle hoodAngle) {
    double distanceMeters = distance.in(Meters);
    distances.add(distanceMeters);
    hoodAngleTable.put(distanceMeters, hoodAngle.in(Degrees));
  }

  public void put(Distance distance, AngularVelocity flywheelSpeed) {
    double distanceMeters = distance.in(Meters);
    distances.add(distanceMeters);
    flywheelSpeedTable.put(distanceMeters, flywheelSpeed.in(RotationsPerSecond));
  }

  public void put(Distance distance, Angle hoodAngle, AngularVelocity flywheelSpeed) {
    double distanceMeters = distance.in(Meters);
    distances.add(distanceMeters);
    hoodAngleTable.put(distanceMeters, hoodAngle.in(Degrees));
    flywheelSpeedTable.put(distanceMeters, flywheelSpeed.in(RotationsPerSecond));
  }

  public ShooterData getShooterData(Distance distance) {
    double distanceMeters = distance.in(Meters);
    double hoodAngleDegrees = hoodAngleTable.get(distanceMeters);
    double flywheelSpeed = flywheelSpeedTable.get(distanceMeters);
    return new ShooterData(Degrees.of(hoodAngleDegrees), RotationsPerSecond.of(flywheelSpeed));
  }

  public void clear() {
    distances.clear();
    hoodAngleTable.clear();
    flywheelSpeedTable.clear();
  }

  public String printSingleLine() {
    StringBuilder sb = new StringBuilder();
    for (Double distance : distances) {
      if (sb.length() > 0) {
        sb.append(" ");
      }
      sb.append(
          String.format(
              "table.put(Meters.of(%.5f), Degrees.of(%.5f), RotationsPerSecond.of(%.5f));",
              distance, hoodAngleTable.get(distance), flywheelSpeedTable.get(distance)));
    }
    return sb.toString();
  }
}

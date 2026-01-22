package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/*
 * ShotTable class to manage shooter data (hood angle and flywheel speed) based on distance to target
 * Uses separate interpolation tables for angle and speed
 */
public class ShotTable {
  // Immutable Data class to hold hood angle and flywheel speed
  public record ShooterData(Angle hoodAngle, LinearVelocity flywheelSpeed) {}

  /*
   * Separate interpolation tables for hood angle and flywheel speed
   * Key: distance to target (in meters)
   * Value: hood angle (degrees) or flywheel speed (RPM)
   */
  private final InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flywheelSpeedTable = new InterpolatingDoubleTreeMap();

  public ShotTable() {}

  public void put(Distance distance, Angle hoodAngle) {
    double distanceMeters = distance.in(Meters);
    hoodAngleTable.put(distanceMeters, hoodAngle.in(Degrees));
  }

  public void put(Distance distance, LinearVelocity flywheelSpeed) {
    double distanceMeters = distance.in(Meters);
    flywheelSpeedTable.put(distanceMeters, flywheelSpeed.in(MetersPerSecond));
  }

  public void put(Distance distance, Angle hoodAngle, LinearVelocity flywheelSpeed) {
    double distanceMeters = distance.in(Meters);
    hoodAngleTable.put(distanceMeters, hoodAngle.in(Degrees));
    flywheelSpeedTable.put(distanceMeters, flywheelSpeed.in(MetersPerSecond));
  }

  public ShooterData getShooterData(Distance distance) {
    double distanceMeters = distance.in(Meters);
    double hoodAngleDegrees = hoodAngleTable.get(distanceMeters);
    double flywheelSpeedRPM = flywheelSpeedTable.get(distanceMeters);
    return new ShooterData(Degrees.of(hoodAngleDegrees), MetersPerSecond.of(flywheelSpeedRPM));
  }
}

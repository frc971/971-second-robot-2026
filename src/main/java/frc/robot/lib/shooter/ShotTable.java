package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

/*
 * ShotTable class to manage shooter data (hood angle and flywheel speed) based on distance to target
 * Uses separate interpolation tables for angle and speed
 */
public class ShotTable {
  // Immutable Data class to hold hood angle and flywheel speed and time of flight
  public record ShooterData(Angle hoodAngle, LinearVelocity flywheelSpeed, Time timeOfFlight) {}

  /*
   * Separate interpolation tables for hood angle and flywheel speed
   * Key: distance to target (in meters)
   * Value: hood angle (degrees) or flywheel speed (RPM) or time of flight (seconds)
   */
  private final InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flywheelSpeedTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap timeTable = new InterpolatingDoubleTreeMap();

  public ShotTable() {}

  public void put(Distance distance, Angle hoodAngle) {
    double distanceMeters = distance.in(Meters);
    hoodAngleTable.put(distanceMeters, hoodAngle.in(Degrees));
  }

  public void put(Distance distance, LinearVelocity flywheelSpeed) {
    double distanceMeters = distance.in(Meters);
    flywheelSpeedTable.put(distanceMeters, flywheelSpeed.in(MetersPerSecond));
  }

  public void put(Distance distance, Time timeOfFlight) {
    double distanceMeters = distance.in(Meters);
    timeTable.put(distanceMeters, timeOfFlight.in(Seconds));
  }

  public void put(Distance distance, Angle hoodAngle, LinearVelocity flywheelSpeed) {
    double distanceMeters = distance.in(Meters);
    hoodAngleTable.put(distanceMeters, hoodAngle.in(Degrees));
    flywheelSpeedTable.put(distanceMeters, flywheelSpeed.in(MetersPerSecond));
  }

  public void put(
      Distance distance, Angle hoodAngle, LinearVelocity flywheelSpeed, Time timeOfFlight) {
    double distanceMeters = distance.in(Meters);
    hoodAngleTable.put(distanceMeters, hoodAngle.in(Degrees));
    flywheelSpeedTable.put(distanceMeters, flywheelSpeed.in(MetersPerSecond));
    timeTable.put(distanceMeters, timeOfFlight.in(Seconds));
  }

  public ShooterData getShooterData(Distance distance) {
    double distanceMeters = distance.in(Meters);
    double hoodAngleDegrees = hoodAngleTable.get(distanceMeters);
    double flywheelSpeedRPM = flywheelSpeedTable.get(distanceMeters);
    double timeOfFlight = timeTable.get(distanceMeters);
    return new ShooterData(
        Degrees.of(hoodAngleDegrees),
        MetersPerSecond.of(flywheelSpeedRPM),
        Seconds.of(timeOfFlight));
  }

  public Time getTime(Distance distance) {
    return Seconds.of(timeTable.get(distance.in(Meters)));
  }
}

package frc.robot.lib;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.lib.shooter.ShotTable;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShotTableTest {
  private static final double DELTA = 1e-6;
  private ShotTable shotTable;

  @BeforeEach
  public void setup() {
    shotTable = new ShotTable();

    // Populate with test data
    // Distance: 1m -> Angle: 30°, Speed: 10 m/s
    shotTable.put(Meters.of(1.0), Degrees.of(30.0), MetersPerSecond.of(10.0));

    // Distance: 2m -> Angle: 40°, Speed: 15 m/s
    shotTable.put(Meters.of(2.0), Degrees.of(40.0), MetersPerSecond.of(15.0));

    // Distance: 4m -> Angle: 50°, Speed: 25 m/s
    shotTable.put(Meters.of(4.0), Degrees.of(50.0), MetersPerSecond.of(25.0));
  }

  @Test
  public void testExactMatch() {
    // Test exact match at 2m
    ShotTable.ShooterData data = shotTable.getShooterData(Meters.of(2.0));

    assertEquals(40.0, data.hoodAngle().in(Degrees), DELTA);
    assertEquals(15.0, data.flywheelSpeed().in(MetersPerSecond), DELTA);
  }

  @Test
  public void testInterpolation() {
    // Test interpolation between 2m and 4m
    // At 3m (midpoint), should interpolate to midpoint values
    // Angle: (40 + 50) / 2 = 45°
    // Speed: (15 + 25) / 2 = 20 m/s
    ShotTable.ShooterData data = shotTable.getShooterData(Meters.of(3.0));

    assertEquals(45.0, data.hoodAngle().in(Degrees), DELTA);
    assertEquals(20.0, data.flywheelSpeed().in(MetersPerSecond), DELTA);
  }

  @Test
  public void testInterpolationQuarterPoint() {
    // Test interpolation at 1.5m (25% between 1m and 2m)
    // Angle: 30 + (40 - 30) * 0.5 = 35°
    // Speed: 10 + (15 - 10) * 0.5 = 12.5 m/s
    ShotTable.ShooterData data = shotTable.getShooterData(Meters.of(1.5));

    assertEquals(35.0, data.hoodAngle().in(Degrees), DELTA);
    assertEquals(12.5, data.flywheelSpeed().in(MetersPerSecond), DELTA);
  }

  @Test
  public void testBelowMinimumReturnsLowest() {
    // Test value below minimum (0.5m, below 1m)
    // Should return the lowest values in the table
    ShotTable.ShooterData data = shotTable.getShooterData(Meters.of(0.5));

    assertEquals(30.0, data.hoodAngle().in(Degrees), DELTA);
    assertEquals(10.0, data.flywheelSpeed().in(MetersPerSecond), DELTA);
  }

  @Test
  public void testAboveMaximumReturnsHighest() {
    // Test value above maximum (5m, above 4m)
    // Should return the highest values in the table
    ShotTable.ShooterData data = shotTable.getShooterData(Meters.of(5.0));

    assertEquals(50.0, data.hoodAngle().in(Degrees), DELTA);
    assertEquals(25.0, data.flywheelSpeed().in(MetersPerSecond), DELTA);
  }

  @Test
  public void testFarBelowMinimum() {
    // Test value far below minimum (0m)
    ShotTable.ShooterData data = shotTable.getShooterData(Meters.of(0.0));

    assertEquals(30.0, data.hoodAngle().in(Degrees), DELTA);
    assertEquals(10.0, data.flywheelSpeed().in(MetersPerSecond), DELTA);
  }

  @Test
  public void testFarAboveMaximum() {
    // Test value far above maximum (10m)
    ShotTable.ShooterData data = shotTable.getShooterData(Meters.of(10.0));

    assertEquals(50.0, data.hoodAngle().in(Degrees), DELTA);
    assertEquals(25.0, data.flywheelSpeed().in(MetersPerSecond), DELTA);
  }

  @Test
  public void testSeparatePutMethods() {
    // Test that we can use separate put methods
    ShotTable table = new ShotTable();
    table.put(Meters.of(1.0), Degrees.of(20.0));
    table.put(Meters.of(1.0), MetersPerSecond.of(5.0));

    ShotTable.ShooterData data = table.getShooterData(Meters.of(1.0));

    assertEquals(20.0, data.hoodAngle().in(Degrees), DELTA);
    assertEquals(5.0, data.flywheelSpeed().in(MetersPerSecond), DELTA);
  }
}

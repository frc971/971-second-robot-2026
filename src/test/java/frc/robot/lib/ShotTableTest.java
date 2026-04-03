package frc.robot.lib;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.lib.shooter.ShotTable;
import org.junit.jupiter.api.Test;

public class ShotTableTest {
  private static final double DELTA = 1e-6;

  @Test
  public void returnsInsertedShooterData() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(2.0), Degrees.of(40.0), RotationsPerSecond.of(15.0));

    ShotTable.ShooterData data = table.getShooterData(Meters.of(2.0));

    assertEquals(40.0, data.hoodAngle().in(Degrees), DELTA);
    assertEquals(15.0, data.flywheelSpeed().in(RotationsPerSecond), DELTA);
  }

  @Test
  public void interpolatesBetweenPoints() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(1.0), Degrees.of(30.0), RotationsPerSecond.of(10.0));
    table.put(Meters.of(3.0), Degrees.of(50.0), RotationsPerSecond.of(20.0));

    ShotTable.ShooterData data = table.getShooterData(Meters.of(2.0));

    assertEquals(40.0, data.hoodAngle().in(Degrees), DELTA);
    assertEquals(15.0, data.flywheelSpeed().in(RotationsPerSecond), DELTA);
  }
}

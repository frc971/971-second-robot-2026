package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;

public class ShooterConfigs {
  public static final ShooterConfig ALL_DEFAULTS = ShooterConfig.builder().build();
  public static final double RIGHT_SIDE_FUDGE = 1.01;

  public static final ShooterConfig TEST_CONFIG =
      ShooterConfig.builder()
          .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(testShotTable()).build())
          .build();
  public static final ShooterConfig RIGHT =
      ShooterConfig.builder()
          .name("Shooter Right")
          .PHYSICAL_CONVERSION(
              ShooterConfig.PhysicalConversion.builder()
                  .TURRET_XY_OFFSET(
                      new Translation3d(0.1624076 - 0.0144525, -0.195097 - 0.0144525, 0.272987))
                  .build())
          .PHYSICS(
              ShooterConfig.Physics.builder()
                  .SHOT_TABLE(shotTable())
                  .FUDGE_FACTOR(RIGHT_SIDE_FUDGE)
                  .build())
          .build();
  public static final ShooterConfig LEFT =
      ShooterConfig.builder()
          .name("Shooter Left")
          .PHYSICAL_CONVERSION(
              ShooterConfig.PhysicalConversion.builder()
                  .TURRET_XY_OFFSET(
                      new Translation3d(0.1624076 - 0.0144525, 0.224003 - 0.0144525, 0.2761615))
                  .build())
          .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(shotTable()).build())
          .build();

  private static ShotTable testShotTable() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(0.5), Degrees.of(77.8668349861), RotationsPerSecond.of(9.23134170253));
    table.put(Meters.of(1.0), Degrees.of(65.9925402176), RotationsPerSecond.of(10.9919493835));
    table.put(Meters.of(1.5), Degrees.of(55.8687578859), RotationsPerSecond.of(12.8321560354));
    table.put(Meters.of(2.0), Degrees.of(47.7289563383), RotationsPerSecond.of(14.7174046972));
    table.put(Meters.of(2.5), Degrees.of(41.6492543046), RotationsPerSecond.of(15.6753316472));
    table.put(Meters.of(3.0), Degrees.of(36.8586755743), RotationsPerSecond.of(16.6385620582));

    table.put(Meters.of(1.0), Seconds.of(0)); // do not delete, would cause errors
    return table;
  }

  private static ShotTable shotTable() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(1.39940 + .18), Degrees.of(9.37500), RotationsPerSecond.of(46.87500));
    table.put(Meters.of(2.47702 + .18), Degrees.of(18.75000), RotationsPerSecond.of(49.97559));
    table.put(Meters.of(3.91026 + .18), Degrees.of(37.50000), RotationsPerSecond.of(50.00000));
    table.put(Meters.of(5.04561), Degrees.of(37.50000), RotationsPerSecond.of(62.50000));

    table.put(Meters.of(1.47), Seconds.of(1.28)); // do not delete, would cause errors
    table.put(Meters.of(2.06), Seconds.of(1.2 - .15));
    table.put(Meters.of(3.01), Seconds.of(1.21 - .15));
    table.put(Meters.of(4.1), Seconds.of(1.06 - .2));
    table.put(Meters.of(5.01), Seconds.of(1.34 - .2));
    return table;
  }
}

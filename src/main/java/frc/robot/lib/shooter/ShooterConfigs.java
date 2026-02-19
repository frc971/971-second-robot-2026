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
  public static final ShooterConfig RIGHT_LOW =
      ShooterConfig.builder()
          .name("Shooter Right")
          .PHYSICAL_CONVERSION(
              ShooterConfig.PhysicalConversion.builder()
                  .TURRET_XY_OFFSET(
                      new Translation3d(0.1624076 - 0.0144525, -0.195097 - 0.0144525, 0.272987))
                  .build())
          .PHYSICS(
              ShooterConfig.Physics.builder()
                  .SHOT_TABLE(lowArcTable())
                  .FUDGE_FACTOR(RIGHT_SIDE_FUDGE)
                  .build())
          .build();
  public static final ShooterConfig LEFT_LOW =
      ShooterConfig.builder()
          .name("Shooter Left")
          .PHYSICAL_CONVERSION(
              ShooterConfig.PhysicalConversion.builder()
                  .TURRET_XY_OFFSET(
                      new Translation3d(0.1624076 - 0.0144525, 0.224003 - 0.0144525, 0.2761615))
                  .build())
          .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(lowArcTable()).build())
          .build();
  public static final ShooterConfig RIGHT_HIGH =
      RIGHT_LOW.toBuilder()
          .PHYSICS(
              ShooterConfig.Physics.builder()
                  .SHOT_TABLE(highArcTable())
                  .FUDGE_FACTOR(RIGHT_SIDE_FUDGE)
                  .build())
          .build();
  public static final ShooterConfig LEFT_HIGH =
      LEFT_LOW.toBuilder()
          .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(highArcTable()).build())
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

  private static ShotTable highArcTable() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(1.37665), Degrees.of(10.25000), RotationsPerSecond.of(43.75000));
    table.put(Meters.of(2.40519), Degrees.of(16.50000), RotationsPerSecond.of(48.43750));
    table.put(Meters.of(3.54892), Degrees.of(19.62500), RotationsPerSecond.of(59.37500));
    table.put(Meters.of(5.28706), Degrees.of(40.62500), RotationsPerSecond.of(68.75000));

    table.put(Meters.of(1.801), Seconds.of((0.7917) + 0.4)); // do not delete, would cause errors
    table.put(Meters.of(2.78), Seconds.of((0.877) + 0.5));
    table.put(Meters.of(3.24), Seconds.of((0.9542) + 0.6));
    return table;
  }

  private static ShotTable lowArcTable() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(1.37665), Degrees.of(10.25000), RotationsPerSecond.of(43.75000));
    table.put(Meters.of(2.40519), Degrees.of(16.50000), RotationsPerSecond.of(48.43750));
    table.put(Meters.of(3.54892), Degrees.of(19.62500), RotationsPerSecond.of(59.37500));
    table.put(Meters.of(5.28706), Degrees.of(40.62500), RotationsPerSecond.of(68.75000));

    table.put(Meters.of(1.801), Seconds.of((0.7917) + 0.4)); // do not delete, would cause errors
    table.put(Meters.of(2.78), Seconds.of((0.877) + 0.5));
    table.put(Meters.of(3.24), Seconds.of((0.9542) + 0.6));
    return table;
  }
}

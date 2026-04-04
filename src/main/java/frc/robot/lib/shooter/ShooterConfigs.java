package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ShooterConfigs {
  public static final ShooterConfig ALL_DEFAULTS = ShooterConfig.builder().build();

  public static final ShooterConfig TEST_CONFIG =
      ShooterConfig.builder()
          .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(testShotTable()).build())
          .build();
  public static final ShooterConfig RIGHT =
      ShooterConfig.builder()
          .name("Shooter Right")
          .PHYSICAL_CONVERSION(
              ShooterConfig.PhysicalConversion.builder()
                  .TURRET_OFFSET(
                      new Translation3d(0.1624076 - 0.0144525, -0.195097 - 0.0144525, 0.44551))
                  .SHOOT_OFFSET(
                      new Translation2d(0.1624076 - 0.0144525 - 0.1, -0.195097 - 0.0144525 - 0.1))
                  .build())
          .PHYSICS(
              ShooterConfig.Physics.builder()
                  .EXIT_SPEED_TABLE(rightExitSpeedTable())
                  .SHOT_TABLE(shotTableRight())
                  .SHUTTLE_TABLE(shuttleTable())
                  .build())
          .build();
  public static final ShooterConfig LEFT =
      ShooterConfig.builder()
          .name("Shooter Left")
          .PHYSICAL_CONVERSION(
              ShooterConfig.PhysicalConversion.builder()
                  .TURRET_OFFSET(
                      new Translation3d(0.1624076 - 0.0144525, 0.224003 - 0.0144525, 0.44551))
                  .SHOOT_OFFSET(
                      new Translation2d(0.1624076 - 0.0144525 + 0.1, 0.224003 - 0.0144525 + 0.1))
                  .build())
          .PHYSICS(
              ShooterConfig.Physics.builder()
                  .EXIT_SPEED_TABLE(leftExitSpeedTable())
                  .SHOT_TABLE(shotTableLeft())
                  .SHUTTLE_TABLE(shuttleTable())
                  .build())
          .build();

  private static ShotTable testShotTable() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(0.5), Degrees.of(77.8668349861), RotationsPerSecond.of(9.23134170253));
    table.put(Meters.of(1.0), Degrees.of(65.9925402176), RotationsPerSecond.of(10.9919493835));
    table.put(Meters.of(1.5), Degrees.of(55.8687578859), RotationsPerSecond.of(12.8321560354));
    table.put(Meters.of(2.0), Degrees.of(47.7289563383), RotationsPerSecond.of(14.7174046972));
    table.put(Meters.of(2.5), Degrees.of(41.6492543046), RotationsPerSecond.of(15.6753316472));
    table.put(Meters.of(3.0), Degrees.of(36.8586755743), RotationsPerSecond.of(16.6385620582));

    return table;
  }

  private static ShotTable shotTableLeft() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(1.01), Degrees.of(9.37500), RotationsPerSecond.of(30.0));
    table.put(Meters.of(1.5794), Degrees.of(9.37500), RotationsPerSecond.of(46.87500));
    table.put(Meters.of(2.0), Degrees.of(13.03411), RotationsPerSecond.of(48.));
    table.put(Meters.of(2.65702), Degrees.of(18.75000), RotationsPerSecond.of(50.0));
    table.put(Meters.of(3.0), Degrees.of(23.23695), RotationsPerSecond.of(51.0));
    table.put(Meters.of(3.5), Degrees.of(29.77807), RotationsPerSecond.of(51.0));
    table.put(Meters.of(4.0), Degrees.of(37.50000), RotationsPerSecond.of(51.0));
    table.put(Meters.of(5.04561), Degrees.of(37.50000), RotationsPerSecond.of(58.0));
    table.put(Meters.of(5.23), Degrees.of(41), RotationsPerSecond.of(64.0));
    table.put(Meters.of(5.69375), Degrees.of(48.4375), RotationsPerSecond.of(69.53125));

    return table;
  }

  private static ShotTable shotTableRight() {
    ShotTable table = new ShotTable();

    table.put(Meters.of(0), Degrees.of(0), RotationsPerSecond.of(0));
    table.put(Meters.of(1.46814), Degrees.of(18.75000), RotationsPerSecond.of(43.75000 - 2.5));
    table.put(Meters.of(1.89302), Degrees.of(18.75000), RotationsPerSecond.of(43.75000 - 2.5));
    table.put(Meters.of(2.51544), Degrees.of(21.87500), RotationsPerSecond.of(50.00000 - 2.5));
    table.put(Meters.of(3.19479), Degrees.of(31.25000), RotationsPerSecond.of(50.00000 - 2.5));
    table.put(Meters.of(3.74519), Degrees.of(31.25000), RotationsPerSecond.of(54.68750));
    table.put(Meters.of(5.50000), Degrees.of(40.62500), RotationsPerSecond.of(60.93750));

    return table;
  }

  private static ShotTable shuttleTable() {
    ShotTable table = new ShotTable();
    // 100% vibed
    table.put(Meters.of(0.0), Degrees.of(40.0), RotationsPerSecond.of(35));
    table.put(Meters.of(2.5), Degrees.of(40.0), RotationsPerSecond.of(50));
    table.put(Meters.of(5.0), Degrees.of(40.0), RotationsPerSecond.of(65));
    table.put(Meters.of(10.0), Degrees.of(40.0), RotationsPerSecond.of(70));

    return table;
  }

  private static ExitSpeedTable testExitSpeedTable() {
    ExitSpeedTable table = new ExitSpeedTable();

    table.put(MetersPerSecond.of(0.0), RotationsPerSecond.of(0.0));
    table.put(MetersPerSecond.of(3.0), RotationsPerSecond.of(6.0));
    table.put(MetersPerSecond.of(6.0), RotationsPerSecond.of(12.0));
    table.put(MetersPerSecond.of(9.0), RotationsPerSecond.of(18.0));
    table.put(MetersPerSecond.of(12.0), RotationsPerSecond.of(24.0));
    table.put(MetersPerSecond.of(15.0), RotationsPerSecond.of(30.0));
    return table;
  }

  private static ExitSpeedTable rightExitSpeedTable() {
    ExitSpeedTable table = new ExitSpeedTable();

    table.put(MetersPerSecond.of(0.00000), RotationsPerSecond.of(0.00000));

    table.put(MetersPerSecond.of(5.45568), RotationsPerSecond.of(36.71875));
    table.put(MetersPerSecond.of(5.91819), RotationsPerSecond.of(40.62500));
    table.put(MetersPerSecond.of(6.61531), RotationsPerSecond.of(45.70313));
    table.put(MetersPerSecond.of(7.27521), RotationsPerSecond.of(50.00000));
    table.put(MetersPerSecond.of(7.94416), RotationsPerSecond.of(55.07813));
    table.put(MetersPerSecond.of(13.0), RotationsPerSecond.of(89.3950053912));

    return table;
  }

  private static ExitSpeedTable leftExitSpeedTable() {
    ExitSpeedTable table = new ExitSpeedTable();

    table.put(MetersPerSecond.of(0.00000), RotationsPerSecond.of(0.00000));

    table.put(MetersPerSecond.of(5.45568), RotationsPerSecond.of(36.71875));
    table.put(MetersPerSecond.of(5.91819), RotationsPerSecond.of(40.62500));
    table.put(MetersPerSecond.of(6.61531), RotationsPerSecond.of(45.70313));
    table.put(MetersPerSecond.of(7.27521), RotationsPerSecond.of(50.00000));
    table.put(MetersPerSecond.of(7.94416), RotationsPerSecond.of(55.07813));
    table.put(MetersPerSecond.of(13.0), RotationsPerSecond.of(89.3950053912));

    return table;
  }
}

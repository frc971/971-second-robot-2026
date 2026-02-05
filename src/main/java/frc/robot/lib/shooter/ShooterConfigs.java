package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import lombok.Getter;

@Getter
public enum ShooterConfigs {
  ALL_DEFAULTS(ShooterConfig.builder().build()),

  // Used by ShooterPhysicsTest - do not edit
  TEST_CONFIG(
      ShooterConfig.builder()
          .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(testShotTable()).build())
          .build()),
  RIGHT_LOW(
      ShooterConfig.builder()
          .name("Shooter Right")
          .PHYSICAL_CONVERSION(
              ShooterConfig.PhysicalConversion.builder()
                  .TURRET_XY_OFFSET(new Translation3d(0.20955, 0.147955, 0.0))
                  .build())
          .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(lowArcTable()).build())
          .build()),
  LEFT_LOW(
      ShooterConfig.builder()
          .name("Shooter Left")
          .PHYSICAL_CONVERSION(
              ShooterConfig.PhysicalConversion.builder()
                  .TURRET_XY_OFFSET(new Translation3d(-0.20955, 0.147955, 0.0))
                  .build())
          .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(lowArcTable()).build())
          .build()),
  RIGHT_HIGH(
      ShooterConfig.builder()
          .name("Shooter Right")
          .PHYSICAL_CONVERSION(
              ShooterConfig.PhysicalConversion.builder()
                  .TURRET_XY_OFFSET(new Translation3d(0.20955, 0.147955, 0.0))
                  .build())
          .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(highArcTable()).build())
          .build()),
  LEFT_HIGH(
      ShooterConfig.builder()
          .name("Shooter Left")
          .PHYSICAL_CONVERSION(
              ShooterConfig.PhysicalConversion.builder()
                  .TURRET_XY_OFFSET(new Translation3d(-0.20955, 0.147955, 0.0))
                  .build())
          .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(highArcTable()).build())
          .build());

  private final ShooterConfig config;

  ShooterConfigs(ShooterConfig config) {
    this.config = config;
  }

  private static ShotTable testShotTable() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(0.5), Degrees.of(77.8668349861), MetersPerSecond.of(9.23134170253));
    table.put(Meters.of(1.0), Degrees.of(65.9925402176), MetersPerSecond.of(10.9919493835));
    table.put(Meters.of(1.5), Degrees.of(55.8687578859), MetersPerSecond.of(12.8321560354));
    table.put(Meters.of(2.0), Degrees.of(47.7289563383), MetersPerSecond.of(14.7174046972));
    table.put(Meters.of(2.5), Degrees.of(41.6492543046), MetersPerSecond.of(15.6753316472));
    table.put(Meters.of(3.0), Degrees.of(36.8586755743), MetersPerSecond.of(16.6385620582));
    return table;
  }

  private static ShotTable highArcTable() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(0.5), Degrees.of(25.9925402176), MetersPerSecond.of(2.9919493835));
    table.put(Meters.of(1.0), Degrees.of(15.8687578859), MetersPerSecond.of(3.8321560354));
    table.put(Meters.of(1.5), Degrees.of(5.01), MetersPerSecond.of(4.2));
    return table;
  }

  private static ShotTable lowArcTable() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(0.5), Degrees.of(37.8668349861), MetersPerSecond.of(1.23134170253));
    table.put(Meters.of(1.0), Degrees.of(25.9925402176), MetersPerSecond.of(2.9919493835));
    table.put(Meters.of(1.5), Degrees.of(15.8687578859), MetersPerSecond.of(3.8321560354));
    return table;
  }
}

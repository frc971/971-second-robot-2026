package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import lombok.Builder;
import lombok.Getter;
import lombok.experimental.Accessors;

@Accessors(fluent = true) // this removes "get" prefix from getters
@Builder
@Getter
public class ShooterConfig {

  /*
   * This assumes that targetpose is predetermined
   * and that multiple target poses --> multiple config files
   */
  @Builder.Default private String name = "Shooter";
  @Builder.Default private Constraints CONSTRAINTS = Constraints.builder().build();
  @Builder.Default private Threshold THRESHOLD = Threshold.builder().build();
  @Builder.Default private Physics PHYSICS = Physics.builder().build();

  @Builder.Default
  private PhysicalConversion PHYSICAL_CONVERSION = PhysicalConversion.builder().build();

  @Builder
  @Getter
  public static class Physics {
    @Builder.Default private ShotTable SHOT_TABLE = new ShotTable();

    @Builder.Default private Time TIME_DELAY = Time.ofBaseUnits(0, Seconds);
  }

  @Getter
  @Builder
  public static class Constraints {
    // low-priority constraints
    @Builder.Default private Distance MIN_SHOT_DISTANCE = Meters.of(1);
    @Builder.Default private Distance MAX_SHOT_DISTANCE = Meters.of(1000);

    // high-priority constraints
    @Builder.Default private LinearVelocity MIN_FLYWHEEL_SPEED = MetersPerSecond.of(0);

    @Builder.Default private LinearVelocity MAX_FLYWHEEL_SPEED = MetersPerSecond.of(1000);

    @Builder.Default private Angle MIN_HOOD_ANGLE = Degrees.of(0);

    @Builder.Default private Angle MAX_HOOD_ANGLE = Degrees.of(30);
  }

  @Getter
  @Builder
  public static class PhysicalConversion {
    // flywheel conversion factors
    // NOTE: needs to be *effective* radius so that math works properly
    @Builder.Default private Distance FLYWHEEL_RADIUS = Distance.ofBaseUnits(0.05, Meters);

    @Builder.Default private double FLYWHEEL_FUDGE_FACTOR = 0;

    // ball related offsets
    // positive x is towards FRONT of the robot
    // positive y is towards PORT/LEFT side
    @Builder.Default private Translation3d TURRET_OFFSET = new Translation3d();

    @Builder.Default private Distance RADIUS_TO_BALL = Distance.ofBaseUnits(0, Meters);

    @Builder.Default private Rotation2d TURRET_ROTATION = Rotation2d.fromDegrees(0);

    // --- STATE related constants ---

    // IDLE constants
    @Builder.Default private Angle IDLE_HOOD_ANGLE = Angle.ofBaseUnits(0, Degrees);

    @Builder.Default
    private LinearVelocity IDLE_FLYWHEEL_LINEAR_VELOCITY =
        LinearVelocity.ofBaseUnits(0, MetersPerSecond);
  }

  // AIMING thresholds (once below, will transition to SHOOTING)
  @Getter
  @Builder
  public static class Threshold {
    @Builder.Default
    private AngularVelocity AIMING_FLYWHEEL_THRESHOLD =
        Radian.per(Seconds).of(300); // TODO: this is obscene, need to fix it later

    @Builder.Default
    private Angle AIMING_ROTATION_THRESHOLD = Radian.of(Math.PI / 6.0); // 45 degrees

    @Builder.Default
    private Angle AIMING_HOOD_ANGLE_THRESHOLD = Radian.of(Math.PI / 6.0); // 30 degrees

    /*
     * Note: very important that the two thresholds (AIMING & SHOOTING) overlap
     * So that robot doesn't flicker between the two states when near threshold boundary
     * (aka, shooter thresholds must be larger/greater than aiming thresholds)
     */

    // SHOOTING thresholds (once above, will abort the shot)
    @Builder.Default private AngularVelocity SHOOTING_FLYWHEEL_ABORT = Radian.per(Seconds).of(500);

    @Builder.Default
    private Angle SHOOTING_ROTATION_THRESHOLD = Radian.of(Math.PI / 8.0); // 45 degrees

    @Builder.Default
    private Angle SHOOTING_HOOD_ANGLE_THRESHOLD = Radian.of(Math.PI / 8.0); // 45 degrees
  }

  // Default config
  // changes here can break a LOT of stuff
  public static ShooterConfig allDefaults() {
    return ShooterConfig.builder().build(); // All other fields use defaults
  }

  // TODO: move these configs to year specific places, and also add configs for each turret
  // Used by ShooterPhysicsTest
  // Do NOT edit
  public static ShooterConfig testConfig() {
    ShotTable table = new ShotTable();

    // Populate shot table with test data
    // Got values from messing around with desmos
    // Target height was 2.0 m, started at 0.5 m distance with a target speed of 8 m/s, increased
    // target speed by 2 m/s every 0.5 m
    table.put(Meters.of(0.5), Degrees.of(77.8668349861), MetersPerSecond.of(9.23134170253));
    table.put(Meters.of(1.0), Degrees.of(65.9925402176), MetersPerSecond.of(10.9919493835));
    table.put(Meters.of(1.5), Degrees.of(55.8687578859), MetersPerSecond.of(12.8321560354));
    table.put(Meters.of(2.0), Degrees.of(47.7289563383), MetersPerSecond.of(14.7174046972));
    table.put(Meters.of(2.5), Degrees.of(41.6492543046), MetersPerSecond.of(15.6753316472));
    table.put(Meters.of(3.0), Degrees.of(36.8586755743), MetersPerSecond.of(16.6385620582));

    return ShooterConfig.builder().PHYSICS(Physics.builder().SHOT_TABLE(table).build()).build();
  }

  public static ShooterConfig actualConfig() {
    ShotTable table = new ShotTable();

    // Need data here to not crash robot
    table.put(Meters.of(0.5), Degrees.of(37.8668349861), MetersPerSecond.of(1.23134170253));
    table.put(Meters.of(1.0), Degrees.of(25.9925402176), MetersPerSecond.of(2.9919493835));
    table.put(Meters.of(1.5), Degrees.of(15.8687578859), MetersPerSecond.of(3.8321560354));

    return ShooterConfig.builder()
        .PHYSICAL_CONVERSION(
            PhysicalConversion.builder()
                .TURRET_OFFSET(new Translation3d(0.0215805258, 0.0, 0.0))
                .build())
        .PHYSICS(Physics.builder().SHOT_TABLE(table).build())
        .build();
  }
}

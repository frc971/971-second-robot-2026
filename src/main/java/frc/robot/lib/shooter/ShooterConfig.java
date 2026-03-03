package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import lombok.Builder;
import lombok.Getter;
import lombok.experimental.Accessors;

@Accessors(fluent = true) // this removes "get" prefix from getters
@Builder(toBuilder = true)
@Getter
public class ShooterConfig {
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

    public Time getTime(Distance distance) {
      return SHOT_TABLE.getTime(distance);
    }
  }

  @Getter
  @Builder
  public static class Constraints {
    // low-priority constraints
    @Builder.Default private Distance MIN_SHOT_DISTANCE = Meters.of(1.0);
    @Builder.Default private Distance MAX_SHOT_DISTANCE = Meters.of(1000);

    // high-priority constraints
    @Builder.Default private AngularVelocity MIN_FLYWHEEL_SPEED = RotationsPerSecond.of(0);
    @Builder.Default private AngularVelocity MAX_FLYWHEEL_SPEED = RotationsPerSecond.of(1000);
    @Builder.Default private Angle MIN_HOOD_ANGLE = Degrees.of(0);
    @Builder.Default private Angle MAX_HOOD_ANGLE = Degrees.of(70);
  }

  @Getter
  @Builder
  public static class PhysicalConversion {
    // ball related offsets (in meters!)
    // positive x is towards FRONT of the robot
    // positive y is towards PORT/LEFT side
    @Builder.Default private Translation3d TURRET_XY_OFFSET = new Translation3d();
    @Builder.Default private Distance RADIUS_TO_BALL = Inches.of(0.0);
  }

  // AIMING thresholds (once below, will transition to SHOOTING)
  @Getter
  @Builder
  public static class Threshold {
    @Builder.Default private AngularVelocity AIMING_FLYWHEEL_THRESHOLD = RotationsPerSecond.of(2.0);

    @Builder.Default private Angle AIMING_ROTATION_THRESHOLD = Degrees.of(10.0);

    @Builder.Default private Angle AIMING_HOOD_ANGLE_THRESHOLD = Degrees.of(2.0);

    /*
     * Note: very important that the two thresholds (AIMING & SHOOTING) overlap
     * So that robot doesn't flicker between the two states when near threshold boundary
     * (aka, shooter thresholds must be larger/greater than aiming thresholds)
     */

    // SHOOTING thresholds (once above, will abort the shot)
    @Builder.Default private AngularVelocity SHOOTING_FLYWHEEL_ABORT = RotationsPerSecond.of(2.5);

    @Builder.Default private Angle SHOOTING_ROTATION_THRESHOLD = Degrees.of(15.0);

    @Builder.Default private Angle SHOOTING_HOOD_ANGLE_THRESHOLD = Degrees.of(4.0);

    @Builder.Default
    private AngularVelocity SHUTTLING_FLYWHEEL_THRESHOLD = RotationsPerSecond.of(8.0);

    @Builder.Default private Angle SHUTTLING_ROTATION_THRESHOLD = Degrees.of(20.0);

    @Builder.Default private Angle SHUTTLING_HOOD_ANGLE_THRESHOLD = Degrees.of(5.0);
  }
}

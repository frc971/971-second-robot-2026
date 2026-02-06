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

    @Builder.Default private Time TIME_DELAY = Seconds.of(0);

    public Time getTime(Distance distance) {
      return SHOT_TABLE.getTime(distance);
    }
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
    @Builder.Default private Angle MAX_HOOD_ANGLE = Degrees.of(70);
  }

  @Getter
  @Builder
  public static class PhysicalConversion {
    // flywheel conversion factors
    // NOTE: needs to be *effective* radius so that math works properly
    @Builder.Default private Distance FLYWHEEL_RADIUS = Inches.of(4.0);

    // ball related offsets (in meters!)
    // positive x is towards FRONT of the robot
    // positive y is towards PORT/LEFT side
    @Builder.Default private Translation3d TURRET_XY_OFFSET = new Translation3d();
    @Builder.Default private Distance RADIUS_TO_BALL = Inches.of(3.5);
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
}

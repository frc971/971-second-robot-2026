package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.SimSwerveConstants;
import frc.robot.subsystems.superstructure.ShooterHandler;

/** Removes only motion toward the hub or the opponent half, in field coordinates. */
final class UnetDeadzone {
  static final boolean ENABLED = true;
  static final double STOP_CLEARANCE = 0.10, BUFFER_WIDTH = 0.50, HUB_HALF_SIDE = 0.60;
  static final double ROBOT_RADIUS =
      Math.hypot(
              SimSwerveConstants.BUMPER_LENGTH_X.in(Meters),
              SimSwerveConstants.BUMPER_LENGTH_Y.in(Meters))
          / 2;

  static Translation2d limit(
      Pose2d pose, Translation2d robotVelocity, Alliance alliance, boolean enabled) {
    if (!enabled) return robotVelocity;
    var velocity = robotVelocity.rotateBy(pose.getRotation());
    var hub =
        (alliance == Alliance.Red ? ShooterHandler.Targets.RED : ShooterHandler.Targets.BLUE)
            .xyPos();
    var delta = pose.getTranslation().minus(hub);
    var nearest =
        new Translation2d(
            MathUtil.clamp(delta.getX(), -HUB_HALF_SIDE, HUB_HALF_SIDE),
            MathUtil.clamp(delta.getY(), -HUB_HALF_SIDE, HUB_HALF_SIDE));
    var outward = delta.minus(nearest);
    double distance = outward.getNorm();
    if (distance > 0) {
      outward = outward.div(distance);
    } else {
      // Inside the footprint: allow escape through the nearest face, including at its center.
      boolean xFace = Math.abs(delta.getX()) >= Math.abs(delta.getY());
      outward =
          xFace
              ? new Translation2d(delta.getX() < 0 ? -1 : 1, 0)
              : new Translation2d(0, delta.getY() < 0 ? -1 : 1);
      distance = (xFace ? Math.abs(delta.getX()) : Math.abs(delta.getY())) - HUB_HALF_SIDE;
    }
    velocity = constrain(velocity, outward, distance - ROBOT_RADIUS);
    double sign = alliance == Alliance.Red ? 1 : -1;
    velocity =
        constrain(
            velocity,
            new Translation2d(sign, 0),
            sign * (pose.getX() - FlippingUtil.fieldSizeX / 2) - ROBOT_RADIUS);
    return velocity.rotateBy(pose.getRotation().unaryMinus());
  }

  private static Translation2d constrain(
      Translation2d velocity, Translation2d outward, double clearance) {
    double inward = Math.min(0, velocity.dot(outward));
    double scale = MathUtil.clamp((clearance - STOP_CLEARANCE) / BUFFER_WIDTH, 0, 1);
    return velocity.minus(outward.times(inward * (1 - scale)));
  }

  private UnetDeadzone() {}
}

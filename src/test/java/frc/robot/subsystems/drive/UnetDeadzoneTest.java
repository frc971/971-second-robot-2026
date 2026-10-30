package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.superstructure.ShooterHandler;
import org.junit.jupiter.api.Test;

class UnetDeadzoneTest {
  private static void check(
      Translation2d expected, Pose2d pose, Translation2d fieldVelocity, Alliance alliance) {
    var actual =
        UnetDeadzone.limit(
                pose, fieldVelocity.rotateBy(pose.getRotation().unaryMinus()), alliance, true)
            .rotateBy(pose.getRotation());
    assertEquals(expected.getX(), actual.getX(), 1e-9);
    assertEquals(expected.getY(), actual.getY(), 1e-9);
  }

  @Test
  void centerlinePreservesLateralAndRetreatAtAllHeadingsAndAlliances() {
    for (var alliance : Alliance.values()) {
      double sign = alliance == Alliance.Blue ? 1 : -1;
      for (double heading : new double[] {0, 90, 180, -37}) {
        for (double clearance : new double[] {-0.5, 0, 0.10, 0.35, 0.60, 1.0}) {
          double scale = Math.max(0, Math.min(1, (clearance - 0.10) / 0.50));
          for (double y : new double[] {1, FlippingUtil.fieldSizeY - 1}) {
            var pose =
                new Pose2d(
                    FlippingUtil.fieldSizeX / 2 - sign * (UnetDeadzone.ROBOT_RADIUS + clearance),
                    y,
                    Rotation2d.fromDegrees(heading));
            check(
                new Translation2d(sign * 2 * scale, 1),
                pose,
                new Translation2d(sign * 2, 1),
                alliance);
            check(new Translation2d(-sign * 2, 1), pose, new Translation2d(-sign * 2, 1), alliance);
          }
        }
      }
    }
  }

  @Test
  void hubFacesCornersAndInteriorAllowTangentialMotionAndEscape() {
    for (var alliance : Alliance.values()) {
      var hub =
          (alliance == Alliance.Blue ? ShooterHandler.Targets.BLUE : ShooterHandler.Targets.RED)
              .xyPos();
      for (double angle : new double[] {0, 45, 90, 135, 180, 225, 270, 315}) {
        var normal = new Translation2d(1, Rotation2d.fromDegrees(angle));
        var tangent = normal.rotateBy(Rotation2d.kCCW_90deg);
        var corner =
            new Translation2d(
                Math.abs(normal.getX()) < 1e-9 ? 0 : Math.copySign(0.60, normal.getX()),
                Math.abs(normal.getY()) < 1e-9 ? 0 : Math.copySign(0.60, normal.getY()));
        for (double clearance : new double[] {0.05, 0.35, 0.65}) {
          double scale = Math.max(0, Math.min(1, (clearance - 0.10) / 0.50));
          var pose =
              new Pose2d(
                  hub.plus(corner).plus(normal.times(UnetDeadzone.ROBOT_RADIUS + clearance)),
                  Rotation2d.fromDegrees(29));
          check(tangent.minus(normal.times(scale)), pose, tangent.minus(normal), alliance);
          check(tangent.plus(normal), pose, tangent.plus(normal), alliance);
        }
      }
      var inside = new Pose2d(hub.plus(new Translation2d(0.2, 0)), Rotation2d.kZero);
      check(new Translation2d(0, 1), inside, new Translation2d(-1, 1), alliance);
      check(new Translation2d(1, 1), inside, new Translation2d(1, 1), alliance);
    }
  }

  @Test
  void disabledIsExactBypass() {
    var requested = new Translation2d(2, -1);
    assertSame(requested, UnetDeadzone.limit(new Pose2d(), requested, Alliance.Blue, false));
  }
}

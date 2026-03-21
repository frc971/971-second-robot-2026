package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.lib.shooter.ShotTable.ShooterData;

public class ShooterPhysics {
  private final ShooterConfig.Physics physicsConfig;

  public ShooterPhysics(ShooterConfig.Physics physicsConfig) {
    this.physicsConfig = physicsConfig;
  }

  /*
   * Assumes that
   * - robot & target stationary
   * - constant target height
   * - empirical data is true
   * - interpolation of angle + speed is close enough (aka ~linear)
   */
  public LaunchSolution stationaryInterpolation(
      ObjectState proj, ObjectState target, ShotTable table) {
    Translation2d distance2d = target.minus(proj).xyPos();

    ShooterData shooterData = table.getShooterData(Meters.of(distance2d.getNorm()));
    Rotation2d turretRotation = distance2d.getAngle();

    return new LaunchSolution(shooterData, turretRotation);
  }

  /*
   * Only works when robot not moving at insane speeds
   * Uses a single iteration approach, approximating offset with time of flight
   */
  public LaunchSolution simpleTimeSolve(ObjectState projectile, ObjectState target) {

    Distance currentDistance =
        Meters.of(projectile.minus(target).xyPos().getNorm()); // double check if actually meters

    // get time of flight from shot table
    Time timeOfFlight = physicsConfig.getTime(currentDistance);

    // find future pose
    // TODO: account for time delay in measuring pose & actual pose
    ObjectState futureRobot = projectile.getFutureState(timeOfFlight);

    return stationaryInterpolation(futureRobot, target, physicsConfig.SHOT_TABLE());
  }

  /**
   * simpleTimeSolve but with multiple Iterations
   *
   * @param maxIterations number of iterations to do
   */
  public LaunchSolution iterativeTimeSolve(
      ObjectState projectile, ObjectState target, int maxIterations, boolean shuttle) {

    ShotTable table = shuttle ? physicsConfig.SHUTTLE_TABLE() : physicsConfig.SHOT_TABLE();

    Distance currentDistance =
        Meters.of(projectile.minus(target).xyPos().getNorm()); // double check if actually meters

    // get time of flight from shot table
    Time timeOfFlight = table.getTime(currentDistance);

    // iteratively update time of flight
    for (int i = 0; i < maxIterations; i++) {
      ObjectState futureRobot = projectile.getFutureState(timeOfFlight);
      Distance interceptDistance =
          Meters.of(
              futureRobot.minus(target).xyPos().getNorm()); // check that this is actually meters

      Time newTimeOfFlight = table.getTime(interceptDistance);

      // Quit early if already converged
      if (Math.abs(newTimeOfFlight.in(Seconds) - timeOfFlight.in(Seconds)) < 0.01) {
        break;
      }

      timeOfFlight = newTimeOfFlight;
    }

    return stationaryInterpolation(projectile.getFutureState(timeOfFlight), target, table);
  }

  /**
   * Vector based launch solver. https://ambcalc.com/docs/projectile.pdf
   *
   * @param proj current projectile ObjectState
   * @param target target ObjectState
   * @return LaunchSolution, or null if the shot is impossible
   */
  public LaunchSolution twiceSolve(ObjectState proj, ObjectState target) {

    Translation2d distance2d = target.minus(proj).xyPos();
    double currentDistance = distance2d.getNorm();

    double targetAngle = physicsConfig.VELOCITY_ANGLE_AT_TARGET().in(Radians);
    double shotAngle =
        Math.atan(
            2 * (target.position().getZ() - proj.position().getZ()) / currentDistance
                - Math.tan(targetAngle));

    double tanDiff = Math.tan(targetAngle) - Math.tan(shotAngle);
    double exitSpeed =
        (1.0 / Math.cos(shotAngle))
            * Math.sqrt((physicsConfig.GRAVITY() * currentDistance) / Math.abs(tanDiff));
    // checks if exit speed is finite and positive
    if (!Double.isFinite(exitSpeed) || exitSpeed <= 0) return null;

    // resolves velocity vector into components
    double vHoriz = exitSpeed * Math.cos(shotAngle);
    double vVertical = exitSpeed * Math.sin(shotAngle);
    // unit vector toward target
    Translation2d horizUnit = distance2d.div(currentDistance);
    Translation3d vShotField =
        new Translation3d(vHoriz * horizUnit.getX(), vHoriz * horizUnit.getY(), vVertical);
    Translation3d vRobot = new Translation3d(proj.velocity().getX(), proj.velocity().getY(), 0);
    Translation3d vBall = vShotField.minus(vRobot);

    Rotation2d turretRotation = new Rotation2d(vBall.getX(), vBall.getY());
    Angle hoodAngle = Radians.of(Math.atan2(vBall.getZ(), Math.hypot(vBall.getX(), vBall.getY())));
    AngularVelocity flywheelSpeed =
        physicsConfig.EXIT_SPEED_TABLE().calcAngularVel(MetersPerSecond.of(vBall.getNorm()));

    // TODO: find a way to resolve tof thing
    return new LaunchSolution(
        new ShooterData(hoodAngle, flywheelSpeed, Seconds.of(0)), turretRotation);
  }

  public LaunchSolution thriceSolve(ObjectState proj, ObjectState target) {
    Translation2d distance2d = target.minus(proj).xyPos();
    double currentDistance = distance2d.getNorm();

    double h0 = proj.position().getZ();
    double ht = target.position().getZ();
    double H = MathUtil.clamp(h0 + (currentDistance * 0.3), h0 + 0.2, 3.5);

    // compute helper terms
    double deltaH = H - h0;

    // check feasibility
    if (deltaH <= 0 || H < ht) return null;

    // discriminant term
    double sqrtTerm = Math.sqrt((H - ht) / deltaH);

    // choose branch:
    // "-" = lower trajectory (usually what you want)
    // "+" = higher arc
    double tanTheta = (2 * deltaH / currentDistance) * (1 + sqrtTerm);

    // compute angle
    double shotAngle = Math.atan(tanTheta);

    // compute exit speed from apex constraint
    double sinTheta = Math.sin(shotAngle);
    if (Math.abs(sinTheta) < 1e-6) return null;

    double exitSpeed = Math.sqrt(2 * physicsConfig.GRAVITY() * deltaH) / sinTheta;

    // checks if exit speed is finite and positive
    if (!Double.isFinite(exitSpeed) || exitSpeed <= 0) return null;

    // resolves velocity vector into components
    double vHoriz = exitSpeed * Math.cos(shotAngle);
    double vVertical = exitSpeed * Math.sin(shotAngle);

    // unit vector toward target
    Translation2d horizUnit = distance2d.div(currentDistance);

    Translation3d vShotField =
        new Translation3d(vHoriz * horizUnit.getX(), vHoriz * horizUnit.getY(), vVertical);

    // --- SAME COMPENSATION AS BEFORE ---
    Translation3d vRobot = new Translation3d(proj.velocity().getX(), proj.velocity().getY(), 0);

    Translation3d vBall = vShotField.minus(vRobot);

    // aiming outputs
    Rotation2d turretRotation = new Rotation2d(vBall.getX(), vBall.getY());

    Angle hoodAngle = Radians.of(Math.atan2(vBall.getZ(), Math.hypot(vBall.getX(), vBall.getY())));

    AngularVelocity flywheelSpeed =
        physicsConfig.EXIT_SPEED_TABLE().calcAngularVel(MetersPerSecond.of(vBall.getNorm()));

    // TODO: find a way to resolve tof thing
    return new LaunchSolution(
        new ShooterData(hoodAngle, flywheelSpeed, Seconds.of(0)), turretRotation);
  }
}

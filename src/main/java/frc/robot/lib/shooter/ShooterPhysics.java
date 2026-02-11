package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  public LaunchSolution stationaryInterpolation(ObjectState proj, ObjectState target) {
    Translation2d distance2d = target.minus(proj).xyPosition();

    ShooterData shooterData =
        physicsConfig.SHOT_TABLE().getShooterData(Meters.of(distance2d.getNorm()));
    Rotation2d turretRotation = distance2d.getAngle();

    return new LaunchSolution(shooterData, turretRotation);
  }

  /*
   * Only works when robot not moving at insane speeds
   * Uses a single iteration approach, approximating offset with time of flight
   */
  public LaunchSolution simpleTimeSolve(ObjectState projectile, ObjectState target) {

    Distance currentDistance =
        Meters.of(
            projectile.minus(target).xyPosition().getNorm()); // double check if actually meters

    // get time of flight from shot table
    Time timeOfFlight = physicsConfig.getTime(currentDistance);

    // find future pose
    // TODO: account for time delay in measuring pose & actual pose
    ObjectState futureRobot = projectile.getFutureState(timeOfFlight);

    return stationaryInterpolation(futureRobot, target);
  }

  /**
   * simpleTimeSolve but with multiple Iterations
   *
   * @param maxIterations number of iterations to do
   */
  public LaunchSolution iterativeTimeSolve(
      ObjectState projectile, ObjectState target, int maxIterations) {

    Distance currentDistance =
        Meters.of(
            projectile.minus(target).xyPosition().getNorm()); // double check if actually meters

    // get time of flight from shot table
    Time timeOfFlight = physicsConfig.getTime(currentDistance);

    // iteratively update time of flight
    for (int i = 0; i < maxIterations; i++) {
      ObjectState futureRobot = projectile.getFutureState(timeOfFlight);
      Distance interceptDistance =
          Meters.of(
              futureRobot
                  .minus(target)
                  .xyPosition()
                  .getNorm()); // check that this is actually meters

      Time newTimeOfFlight = physicsConfig.getTime(interceptDistance);

      // Quit early if already converged
      if (Math.abs(newTimeOfFlight.in(Seconds) - timeOfFlight.in(Seconds)) < 0.01) {
        break;
      }

      timeOfFlight = newTimeOfFlight;
    }

    return stationaryInterpolation(projectile.getFutureState(timeOfFlight), target);
  }
}

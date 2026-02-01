package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.lib.shooter.ShotTable.ShooterData;

public class ShooterPhysics {
  public static final Translation3d gravity = new Translation3d(0, 0, -9.81);

  private final ShooterConfig.Physics config;

  public ShooterPhysics(ShooterConfig.Physics config) {
    this.config = config;
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

    ShooterData shooterData = config.SHOT_TABLE().getShooterData(Meters.of(distance2d.getNorm()));
    Rotation2d turretRotation = distance2d.getAngle();

    return new LaunchSolution(shooterData, turretRotation);
  }

  public LaunchSolution alexSolve(ObjectState robot, ObjectState target) {
    // assume ball is stationary --> get LUT predicition
    Translation3d stationaryVelocity =
        stationaryInterpolation(robot.getFutureState(config.TIME_DELAY()), target).fireVector();

    // correct for robot velocity
    Translation3d fireVelocity = stationaryVelocity.minus(robot.velocity());

    return new LaunchSolution(fireVelocity);
  }

  /*
   * Since kinematics equation for fire velocity has an additional unknown (time of flight)
   * Need to pick an arbitrary time of flight based
   * This time is calculated by picking an arbitrary speed determined by the lookup table (LUT)
   * Since LUT also accounts for drag + other factors
   * Adjusting the velocity afterwards makes a *mostly* accurate guess
   */
  /// NOTE: does NOT work currently
  public LaunchSolution quadraticSolve(ObjectState proj, ObjectState target) {

    // Do calculations based on where turret & target WILL be when projectile is fired
    // * assumes stationary target *
    // TODO: more accurately predict future ObjectStates
    // NOTE: proj ≈ robot state
    // target = target; this code needs significant update if target moves

    // --- PICKING A TIME OF FLIGHT ---

    // need to remove on unknown to find the other (aka choose a speed)
    // get initial velocity guess from LUT
    LaunchSolution initGuess =
        stationaryInterpolation(proj.getFutureState(config.TIME_DELAY()), target);

    // only use speed in XY plane (2D speed), ignore vertical direction
    // this is because LUT table only stores XY (1D) distance between target & robot
    double guessSpeed = initGuess.linearFlywheelVelocity().in(MetersPerSecond);

    // get distance between target and projectile launch position
    Translation3d distance = target.minus(proj).position();

    // 2d vectors describing distance to target and robot velocity
    Vector<N3> distanceV = distance.toVector();
    Vector<N3> velocityV = proj.velocity().toVector();

    // Use following quadratic to solve for time of flight
    // 0 = (||v_robot||^2 - s_guess^2) t_of^2 - 2(d · v_robot) t_of + ||d||^2

    double a = Math.pow(velocityV.norm(), 2) - Math.pow(guessSpeed, 2);
    double b = -2.0 * distanceV.dot(velocityV);
    double c = Math.pow(distanceV.norm(), 2);

    double discriminant =
        Math.max(0.0, b * b - 4.0 * a * c); // ensures nonzero discriminant --> real solutions

    double timeOfFlight = (-b - Math.sqrt(discriminant)) / (2.0 * a); // in seconds

    // Time of Flight * Robot Velocity = lead vector (how much it would miss by)
    // the following calculations accounts for that and gives new Fire Velocity VECTOR (aka includes
    // all angles necessary)

    // --- FIRE VELOCITY (RELATIVE TO ROBOT) CALCULATION

    // V_fire = (S_target - S_ball - V_robot t_of - 1/2 g t_of^2) / t_of
    // V_fire = ( distance - V_robot t_of - 1/2 g t_of^2 ) / t_of
    Translation3d fireVelocityVector =
        distance
            .minus(proj.velocity().times(timeOfFlight))
            .minus(gravity.times(0.5 * Math.pow(timeOfFlight, 2)))
            .div(timeOfFlight); // Overall Distance Vector then divided by Time of Flight

    // convert velocity vector to LaunchSolution
    return new LaunchSolution(fireVelocityVector);
  }

  // public Solution attackAngle(ObjectState proj, ObjectState target) {}
}

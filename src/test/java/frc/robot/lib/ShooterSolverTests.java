package frc.robot.lib;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.shooter.LaunchSolution;
import frc.robot.lib.shooter.ObjectState;
import frc.robot.lib.shooter.ShooterConfig;
import frc.robot.lib.shooter.ShooterPhysics;
import frc.robot.lib.shooter.ShotTable;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

// Sanity checks for the physics solvers
public class ShooterSolverTests {

  private static final double TARGET_HEIGHT_METERS = 1.8288; // ~72 in

  private ShooterPhysics physics;

  @BeforeEach
  public void setup() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(2.0), Degrees.of(60), MetersPerSecond.of(10), Seconds.of(0.4));
    table.put(Meters.of(5.0), Degrees.of(45), MetersPerSecond.of(14), Seconds.of(0.6));
    table.put(Meters.of(10.0), Degrees.of(35), MetersPerSecond.of(18), Seconds.of(0.9));

    ShooterConfig.Physics config =
        ShooterConfig.Physics.builder().SHOT_TABLE(table).TIME_DELAY(Seconds.of(0.05)).build();
    physics = new ShooterPhysics(config);
  }

  @Test
  public void alexSolve_returnsValidSolution_stationaryRobot() {
    ObjectState robot = new ObjectState(new Translation3d(0, 0, 0), new Translation3d(0, 0, 0));
    ObjectState target =
        new ObjectState(new Translation3d(5.0, 0, TARGET_HEIGHT_METERS), new Translation3d());

    LaunchSolution solution = physics.alexSolve(robot, target);

    assertNotNull(solution);
    Translation3d fire = solution.fireVector();
    assertNotNull(fire);
    assertTrue(fire.getNorm() > 0.1, "fire speed should be positive");
    assertTrue(Double.isFinite(fire.getNorm()), "fire vector should be finite");
    // Rough sanity: fire should point generally toward target (positive X, positive
    // Z)
    assertTrue(fire.getX() > 0, "fire should have positive X toward target");
    assertTrue(fire.getZ() > 0, "fire should have upward Z component");
  }

  @Test
  public void alexSolve_returnsValidSolution_movingRobot() {
    ObjectState robot = new ObjectState(new Translation3d(0, 0, 0), new Translation3d(1.5, 0, 0));
    ObjectState target =
        new ObjectState(new Translation3d(5.0, 0, TARGET_HEIGHT_METERS), new Translation3d());

    LaunchSolution solution = physics.alexSolve(robot, target);

    assertNotNull(solution);
    Translation3d fire = solution.fireVector();
    assertNotNull(fire);
    assertTrue(fire.getNorm() > 0.1, "fire speed should be positive");
    assertTrue(Double.isFinite(fire.getNorm()), "fire vector should be finite");
  }

  @Test
  public void simpleTimeSolve_returnsValidSolution_stationaryRobot() {
    ObjectState robot = new ObjectState(new Translation3d(0, 0, 0), new Translation3d(0, 0, 0));
    ObjectState target =
        new ObjectState(new Translation3d(5.0, 0, TARGET_HEIGHT_METERS), new Translation3d());

    LaunchSolution solution = physics.simpleTimeSolve(robot, target);

    assertNotNull(solution);
    Translation3d fire = solution.fireVector();
    assertNotNull(fire);
    assertTrue(fire.getNorm() > 0.1, "fire speed should be positive");
    assertTrue(Double.isFinite(fire.getNorm()), "fire vector should be finite");
    assertTrue(fire.getX() > 0, "fire should have positive X toward target");
    assertTrue(fire.getZ() > 0, "fire should have upward Z component");
  }

  @Test
  public void simpleTimeSolve_returnsValidSolution_movingRobot() {
    ObjectState robot = new ObjectState(new Translation3d(0, 0, 0), new Translation3d(1.0, 0, 0));
    ObjectState target =
        new ObjectState(new Translation3d(5.0, 0, TARGET_HEIGHT_METERS), new Translation3d());

    LaunchSolution solution = physics.simpleTimeSolve(robot, target);

    assertNotNull(solution);
    Translation3d fire = solution.fireVector();
    assertNotNull(fire);
    assertTrue(fire.getNorm() > 0.1, "fire speed should be positive");
    assertTrue(Double.isFinite(fire.getNorm()), "fire vector should be finite");
  }
}

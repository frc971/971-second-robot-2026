package frc.robot.lib;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.shooter.LaunchSolution;
import frc.robot.lib.shooter.ObjectState;
import frc.robot.lib.shooter.ShooterConfig;
import frc.robot.lib.shooter.ShooterConfigs;
import frc.robot.lib.shooter.ShooterPhysics;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShooterPhysicsTest {
  private ShooterPhysics physics;
  private ShooterConfig config;

  @BeforeEach
  public void setup() {
    config = ShooterConfigs.TEST_CONFIG;
    physics = new ShooterPhysics(config.PHYSICS());
  }

  @Test
  public void stationaryInterpolation_returnsValidSolution() {
    ObjectState robot = new ObjectState(new Translation2d(0, 0), new Translation2d(0, 0));
    ObjectState target = new ObjectState(new Translation2d(2.0, 0), new Translation2d(0, 0));

    LaunchSolution solution = physics.stationaryInterpolation(robot, target);

    assertNotNull(solution);
    assertTrue(solution.flywheelSpeed().gt(RotationsPerSecond.of(0)));
  }

  @Test
  public void simpleTimeSolve_returnsValidSolution() {
    ObjectState robot = new ObjectState(new Translation2d(0, 0), new Translation2d(1.0, 0));
    ObjectState target = new ObjectState(new Translation2d(3.0, 0), new Translation2d(0, 0));

    LaunchSolution solution = physics.simpleTimeSolve(robot, target);

    assertNotNull(solution);
    assertTrue(solution.flywheelSpeed().gt(RotationsPerSecond.of(0)));
  }

  @Test
  public void iterativeTimeSolve_returnsValidSolution() {
    ObjectState robot = new ObjectState(new Translation2d(0, 0), new Translation2d(1.0, 0));
    ObjectState target = new ObjectState(new Translation2d(3.0, 0), new Translation2d(0, 0));

    LaunchSolution solution = physics.iterativeTimeSolve(robot, target, 5);

    assertNotNull(solution);
    assertTrue(solution.flywheelSpeed().gt(RotationsPerSecond.of(0)));
  }
}

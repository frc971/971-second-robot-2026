package frc.robot.lib;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation3d;
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
    ObjectState robot = new ObjectState(new Translation3d(0, 0, 0), new Translation3d(0, 0, 0));
    ObjectState target = new ObjectState(new Translation3d(2.0, 0, 0), new Translation3d(0, 0, 0));
    LaunchSolution solution =
        physics.stationaryInterpolation(robot, target, config.PHYSICS().SHOT_TABLE());
    assertNotNull(solution);
    assertTrue(solution.flywheelSpeed().gt(RotationsPerSecond.of(0)));
  }
}

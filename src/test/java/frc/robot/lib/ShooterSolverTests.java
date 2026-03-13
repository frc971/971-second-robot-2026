package frc.robot.lib;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.shooter.LaunchSolution;
import frc.robot.lib.shooter.ObjectState;
import frc.robot.lib.shooter.ShooterConfig;
import frc.robot.lib.shooter.ShooterPhysics;
import frc.robot.lib.shooter.ShotTable;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShooterSolverTests {
  private ShooterPhysics physics;
  private ShooterConfig config;

  @BeforeEach
  public void setup() {
    ShotTable table = new ShotTable();
    table.put(Meters.of(2.0), Degrees.of(60), RotationsPerSecond.of(10), Seconds.of(0.4));
    table.put(Meters.of(5.0), Degrees.of(45), RotationsPerSecond.of(14), Seconds.of(0.6));

    config =
        ShooterConfig.builder()
            .PHYSICS(ShooterConfig.Physics.builder().SHOT_TABLE(table).build())
            .build();
    physics = new ShooterPhysics(config.PHYSICS());
  }

  @Test
  public void iterativeTimeSolve_returnsFiniteSolution() {
    ObjectState robot = new ObjectState(new Translation2d(0, 0), new Translation2d(0.5, 0));
    ObjectState target = new ObjectState(new Translation2d(5.0, 0), new Translation2d());

    LaunchSolution solution = physics.iterativeTimeSolve(robot, target, 3, false);

    assertNotNull(solution);
    assertTrue(solution.flywheelSpeed().gt(RotationsPerSecond.of(0)));
  }
}

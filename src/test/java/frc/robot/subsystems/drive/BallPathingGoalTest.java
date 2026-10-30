package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class BallPathingGoalTest {
  private static final double MAX_SPEED = 3.5;
  private static final double MAX_OMEGA = 5.0;

  @Test
  void parsesValidTenElementGoal() {
    double[] values = {2.5, 0.0, 2.0, 0.5, 3.0, 1.0, 0.0, 12.0, 0.2, 1.5};
    BallPathingGoal goal =
        BallPathingGoal.fromNetworkTables(values, MAX_SPEED, MAX_OMEGA).orElseThrow();
    assertEquals(2.5, goal.velocityX());
    assertEquals(0.0, goal.velocityY());
    assertEquals(1.5, goal.omegaRadiansPerSecond());
    assertEquals(2.0, goal.targetX());
    assertEquals(0.5, goal.targetY());
  }

  @Test
  void rejectsLegacyInvalidAndNonFiniteGoals() {
    assertTrue(BallPathingGoal.fromNetworkTables(new double[9], MAX_SPEED, MAX_OMEGA).isEmpty());
    double[] invalid = {2.5, 0.0, 2.0, 0.5, 3.0, 0.0, 0.0, 12.0, 0.2, 1.5};
    assertTrue(BallPathingGoal.fromNetworkTables(invalid, MAX_SPEED, MAX_OMEGA).isEmpty());
    invalid[5] = 1.0;
    for (int index : new int[] {0, 1, 2, 3, 5, 9}) {
      double saved = invalid[index];
      for (double value :
          new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
        invalid[index] = value;
        assertTrue(BallPathingGoal.fromNetworkTables(invalid, MAX_SPEED, MAX_OMEGA).isEmpty());
      }
      invalid[index] = saved;
    }
  }

  @Test
  void clampsTranslationAndYawToDrivetrainLimits() {
    double[] values = {6.0, 8.0, 2.0, 0.5, 3.0, 1.0, 0.0, 12.0, 0.2, -8.0};
    BallPathingGoal goal =
        BallPathingGoal.fromNetworkTables(values, MAX_SPEED, MAX_OMEGA).orElseThrow();
    assertEquals(MAX_SPEED, Math.hypot(goal.velocityX(), goal.velocityY()), 1e-12);
    assertEquals(-MAX_OMEGA, goal.omegaRadiansPerSecond());
  }
}

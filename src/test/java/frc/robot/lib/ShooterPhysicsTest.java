package frc.robot.lib;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.shooter.LaunchSolution;
import frc.robot.lib.shooter.ObjectState;
import frc.robot.lib.shooter.ShooterConfig;
import frc.robot.lib.shooter.ShooterPhysics;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShooterPhysicsTest {
  private ShooterPhysics physics;
  private ShooterConfig config;

  @BeforeEach
  public void setup() {
    config = ShooterConfig.testConfig();
    physics = new ShooterPhysics(config.PHYSICS());
  }

  @Test
  public void testCompareSolvers_SameScenario() {
    System.out.println("\n=== Comparing All Solvers: multiple distances and motion cases ===");

    // Distances (close, medium, far). Target height fixed at 2.0 m (shot table tuned for 2m)
    double[] ranges = {0.5, 2.0, 3.0};
    double height = 2.0;

    // Motion cases in order: stationary, forwards, sideways, backwards, diagonal
    Translation3d[] robotVelocities =
        new Translation3d[] {
          new Translation3d(0, 0, 0), // stationary
          new Translation3d(1.5, 0, 0), // forwards
          new Translation3d(0, 1.5, 0), // sideways
          new Translation3d(-1.5, 0, 0), // backwards
          new Translation3d(1.0, 1.0, 0) // diagonal
        };

    String[] motionNames =
        new String[] {"stationary", "forwards", "sideways", "backwards", "diagonal"};

    for (int i = 0; i < ranges.length; i++) {
      double range = ranges[i];

      for (int m = 0; m < robotVelocities.length; m++) {
        Translation3d rv = robotVelocities[m];

        System.out.println(
            String.format(
                "\n--- Scenario: range=%.2fm height=%.2fm motion=%s ---",
                range, height, motionNames[m]));

        ObjectState robot = new ObjectState(new Translation3d(0, 0, 0), rv);
        ObjectState target =
            new ObjectState(new Translation3d(range, 0, height), new Translation3d(0, 0, 0));

        System.out.println(
            "Robot Position: "
                + formatTranslation3d(robot.position())
                + "  Velocity: "
                + formatTranslation3d(robot.velocity()));
        System.out.println("Target Position: " + formatTranslation3d(target.position()));

        System.out.println("\n  -> stationaryInterpolation:");
        LaunchSolution s1 = physics.stationaryInterpolation(robot, target);
        printSolutionCompact(s1);
        double[] miss1 = missAndTime(robot.position(), s1, robot.velocity(), target.position());
        System.out.println(String.format("    Error: %.3f m (t=%.3f s)", miss1[0], miss1[1]));
        // threshold: near-zero for stationary case, larger tolerance otherwise
        double thresh1 = motionNames[m].equals("stationary") ? 1e-3 : 0.6;
        checkMiss("stationaryInterpolation", s1, robot, target, miss1[0], miss1[1], thresh1);

        System.out.println("\n  -> alexSolve:");
        LaunchSolution s2 = physics.alexSolve(robot, target);
        printSolutionCompact(s2);
        double[] miss2 = missAndTime(robot.position(), s2, robot.velocity(), target.position());
        System.out.println(String.format("    Error: %.3f m (t=%.3f s)", miss2[0], miss2[1]));
        double thresh2 = 1e-3; // acceptable error for alexSolve
        checkMiss("alexSolve", s2, robot, target, miss2[0], miss2[1], thresh2);

        System.out.println("\n  -> quadraticSolve:");
        LaunchSolution s3 = physics.quadraticSolve(robot, target);
        printSolutionCompact(s3);
        double[] miss3 = missAndTime(robot.position(), s3, robot.velocity(), target.position());
        System.out.println(String.format("    Error: %.3f m (t=%.3f s)", miss3[0], miss3[1]));
        double thresh3 = 1e-3; // acceptable error for quadraticSolve
        checkMiss("quadraticSolve", s3, robot, target, miss3[0], miss3[1], thresh3);
      }
    }
  }

  private void checkMiss(
      String solverName,
      LaunchSolution sol,
      ObjectState robot,
      ObjectState target,
      double miss,
      double time,
      double threshold) {
    if (miss > threshold) {
      System.out.println("\n*** FAILED: " + solverName + " exceeded threshold");
      System.out.println("Scenario:");
      System.out.println("  Robot Position: " + formatTranslation3d(robot.position()));
      System.out.println("  Robot Velocity: " + formatTranslation3d(robot.velocity()));
      System.out.println("  Target Position: " + formatTranslation3d(target.position()));
      printSolution(solverName, robot, target, sol);
      System.out.println(
          String.format("  Miss: %.3f m (t=%.3f s) threshold=%.3f", miss, time, threshold));
    }

    assertTrue(
        miss <= threshold,
        String.format("%s miss %.3f > threshold %.3f (t=%.3f)", solverName, miss, threshold, time));
  }

  /**
   * Compute miss distance (meters) and the time (seconds) when projectile is closest to the target
   * height. Uses the formula: pos(t) = r0 + (v_fire + v_robot) * t + 0.5 * g * t^2. Returns an
   * array {missMeters, timeSeconds}. If no positive analytic crossing time exists, numerically
   * searches times 0..5s for the minimal distance.
   */
  private double[] missAndTime(
      Translation3d r0, LaunchSolution sol, Translation3d robotVel, Translation3d target) {
    Translation3d vFire = sol.fireVector();
    Translation3d v0 = vFire.plus(robotVel); // world-frame initial velocity

    double z0 = r0.getZ();
    double tz = target.getZ();
    double vz = v0.getZ();
    double gz = ShooterPhysics.gravity.getZ(); // negative

    double a = 0.5 * gz;
    double b = vz;
    double c = z0 - tz;

    double bestT = -1.0;
    double bestMiss = Double.POSITIVE_INFINITY;

    double disc = b * b - 4 * a * c;
    if (disc >= 0 && Math.abs(a) > 1e-12) {
      double sqrt = Math.sqrt(disc);
      double t1 = (-b + sqrt) / (2 * a);
      double t2 = (-b - sqrt) / (2 * a);
      for (double t : new double[] {t1, t2}) {
        if (t > 1e-6) {
          Translation3d pos = r0.plus(v0.times(t)).plus(ShooterPhysics.gravity.times(0.5 * t * t));
          double miss = pos.minus(target).getNorm();
          if (miss < bestMiss) {
            bestMiss = miss;
            bestT = t;
          }
        }
      }
    }

    // fallback: numerical search over 0..5s
    if (bestT < 0) {
      double maxT = 5.0;
      double dt = 0.01;
      for (double t = 0.0; t <= maxT; t += dt) {
        Translation3d pos = r0.plus(v0.times(t)).plus(ShooterPhysics.gravity.times(0.5 * t * t));
        double miss = pos.minus(target).getNorm();
        if (miss < bestMiss) {
          bestMiss = miss;
          bestT = t;
        }
      }
    }

    if (bestT < 0) {
      return new double[] {Double.POSITIVE_INFINITY, -1.0};
    }

    return new double[] {bestMiss, bestT};
  }

  // Helper method for compact solution printing
  private void printSolutionCompact(LaunchSolution solution) {
    // Single-line compact summary: Hood° | Flywheel m/s | Turret° | FireVec(x,y,z) m | |v| m/s
    double hoodDeg = solution.shooterData.hoodAngle().in(Degrees);
    double flywheel = solution.shooterData.flywheelSpeed().in(MetersPerSecond);
    double turretDeg = solution.turretRotation.getDegrees();
    Translation3d fv = solution.fireVector();
    System.out.println(
        String.format(
            "H: %.2f° | Fw: %.3f m/s | Tur: %.2f° | V: (%.3f, %.3f, %.3f) m | |v|: %.3f m/s",
            hoodDeg, flywheel, turretDeg, fv.getX(), fv.getY(), fv.getZ(), fv.getNorm()));
  }

  // Detailed solution printer used when a test fails
  private void printSolution(
      String solverName, ObjectState robot, ObjectState target, LaunchSolution solution) {
    System.out.println("\nSolver: " + solverName);

    System.out.println("\nInput States:");
    System.out.println("  Robot Position:  " + formatTranslation3d(robot.position()));
    System.out.println("  Robot Velocity:  " + formatTranslation3d(robot.velocity()));
    System.out.println("  Target Position: " + formatTranslation3d(target.position()));
    System.out.println("  Target Velocity: " + formatTranslation3d(target.velocity()));

    System.out.println("\nOutput Solution:");
    System.out.println("  Hood Angle:      " + formatAngle(solution.shooterData.hoodAngle()));
    System.out.println(
        "  Flywheel Speed:  " + formatVelocity(solution.shooterData.flywheelSpeed()));
    System.out.println("  Turret Rotation: " + formatRotation2d(solution.turretRotation));
    System.out.println("  Fire Vector:     " + formatTranslation3d(solution.fireVector()));
    System.out.println(
        "  Fire Speed:      " + String.format("%.3f m/s", solution.fireVector().getNorm()));
  }

  // Formatting helpers
  private String formatTranslation3d(Translation3d t) {
    return String.format("(%.3f, %.3f, %.3f) m", t.getX(), t.getY(), t.getZ());
  }

  private String formatAngle(edu.wpi.first.units.measure.Angle angle) {
    return String.format("%.2f° (%.4f rad)", angle.in(Degrees), angle.in(Radians));
  }

  private String formatVelocity(edu.wpi.first.units.measure.LinearVelocity velocity) {
    return String.format("%.3f m/s", velocity.in(MetersPerSecond));
  }

  private String formatRotation2d(Rotation2d rotation) {
    return String.format("%.2f° (%.4f rad)", rotation.getDegrees(), rotation.getRadians());
  }
}

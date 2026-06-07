/*
 * ProjectileSimulator.java - RK4 projectile physics with drag and Magnus lift
 *
 * MIT License
 *
 * Copyright (c) 2026 FRC Team 5962 perSEVERE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 */

package frc.robot.lib.simulation;

import java.util.ArrayList;
import java.util.List;

/**
 * Simulates a ball flying through the air with drag and Magnus lift. Uses RK4 integration in the
 * vertical plane (x, z). For each distance, binary searches RPM until the ball arrives at the
 * target height. Then generates a 91-point lookup table from 0.50m to 5.00m.
 *
 * <p>Basically, you plug in your robot's measurements from CAD, run generateLUT(), and it gives you
 * a complete shooter table. No more hand-tuning RPM values from match videos.
 *
 * <p>Usage:
 *
 * <pre>
 *   SimParameters params = new SimParameters(
 *       0.215,   // ball mass kg
 *       0.1501,  // ball diameter m
 *       0.47,    // drag coeff (smooth sphere)
 *       0.2,     // Magnus coeff
 *       1.225,   // air density kg/m^3
 *       0.43,    // exit height from floor, measure from CAD
 *       0.1016,  // wheel diameter, measure with calipers
 *       1.83,    // target height, from game manual
 *       0.6,     // slip factor (0=no grip, 1=perfect), tune on robot
 *       45.0,    // launch angle degrees from horizontal
 *       0.001,   // sim timestep
 *       1500, 6000, 25, 5.0  // RPM range, search iters, max sim time
 *   );
 *   ProjectileSimulator sim = new ProjectileSimulator(params);
 *   GeneratedLUT lut = sim.generateLUT();
 *   for (LUTEntry entry : lut.entries()) {
 *       if (entry.reachable()) {
 *           System.out.printf("%.2fm -> %.0f RPM, %.3fs TOF%n",
 *               entry.distanceM(), entry.rpm(), entry.tof());
 *       }
 *   }
 * </pre>
 */
public class ProjectileSimulator {

  // Your robot's physical measurements, from CAD and the game manual
  public record SimParameters(
      double ballMassKg,
      double ballDiameterM,
      double dragCoeff,
      double magnusCoeff,
      double airDensity,
      double exitHeightM,
      double wheelDiameterM,
      double targetHeightM,
      double slipFactor,
      double fixedLaunchAngleDeg,
      double dt,
      double rpmMin,
      double rpmMax,
      int binarySearchIters,
      double maxSimTime) {}

  public record TrajectoryResult(
      double zAtTarget, double tof, boolean reachedTarget, double maxHeight, double apexX) {}

  // One row: distance -> RPM that lands it, TOF, reachable flag
  public record LUTEntry(double distanceM, double rpm, double tof, boolean reachable) {}

  /** Full LUT with generation stats. */
  public record GeneratedLUT(
      List<LUTEntry> entries,
      SimParameters params,
      int reachableCount,
      int unreachableCount,
      double maxRangeM,
      long generationTimeMs) {}

  private final SimParameters params;

  // Precomputed aero constants
  private final double kDrag;
  private final double kMagnus;

  // +1 for topspin (upward lift, the default), -1 for backspin (downward push).
  // Depends on how the flywheel contacts the ball.
  private double magnusSign = 1.0;

  public ProjectileSimulator(SimParameters params) {
    this.params = params;
    double area = Math.PI * (params.ballDiameterM() / 2.0) * (params.ballDiameterM() / 2.0);
    this.kDrag = (params.airDensity() * params.dragCoeff() * area) / (2.0 * params.ballMassKg());
    this.kMagnus =
        (params.airDensity() * params.magnusCoeff() * area) / (2.0 * params.ballMassKg());
  }

  /**
   * @param magnusSign +1.0 for topspin (lift), -1.0 for backspin (downward push). Most top-contact
   *     flywheels impart topspin. Drum shooters that contact the bottom often impart backspin.
   */
  public ProjectileSimulator(SimParameters params, double magnusSign) {
    this(params);
    this.magnusSign = magnusSign;
  }

  public void setMagnusSign(double sign) {
    this.magnusSign = sign;
  }

  /**
   * RPM to ball exit speed. slipFactor (0-1) = how much surface speed transfers to ball. Calibrate
   * on robot.
   */
  public double exitVelocity(double rpm) {
    return params.slipFactor() * rpm * Math.PI * params.wheelDiameterM() / 60.0;
  }

  public TrajectoryResult simulate(double rpm, double targetDistanceM) {
    return simulate(rpm, targetDistanceM, params.fixedLaunchAngleDeg());
  }

  /** Simulate with a specific launch angle (degrees from horizontal). */
  public TrajectoryResult simulate(double rpm, double targetDistanceM, double launchAngleDeg) {
    double v0 = exitVelocity(rpm);
    double launchRad = Math.toRadians(launchAngleDeg);
    double vx = v0 * Math.cos(launchRad);
    double vz = v0 * Math.sin(launchRad);

    double x = 0;
    double z = params.exitHeightM();
    double dt = params.dt();
    double maxHeight = z;
    double apexX = 0;

    double t = 0;
    double maxTime = params.maxSimTime();

    while (t < maxTime) {
      // Save pre-step state for accurate interpolation
      double prevX = x;
      double prevZ = z;

      // RK4 step
      double[] state = {x, z, vx, vz};
      double[] k1 = derivatives(state);
      double[] s2 = addScaled(state, k1, dt / 2.0);
      double[] k2 = derivatives(s2);
      double[] s3 = addScaled(state, k2, dt / 2.0);
      double[] k3 = derivatives(s3);
      double[] s4 = addScaled(state, k3, dt);
      double[] k4 = derivatives(s4);

      x += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
      z += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
      vx += dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
      vz += dt / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
      t += dt;

      if (z > maxHeight) {
        maxHeight = z;
        apexX = x;
      }

      // Check if we've passed or reached the target distance
      if (x >= targetDistanceM) {
        // Linear interpolation using saved pre-step state
        double frac = (targetDistanceM - prevX) / (x - prevX);
        double zAtTarget = prevZ + frac * (z - prevZ);
        double tofAtTarget = t - dt + frac * dt;
        return new TrajectoryResult(zAtTarget, tofAtTarget, true, maxHeight, apexX);
      }

      // Ball hit the ground
      if (z < 0) {
        return new TrajectoryResult(0, t, false, maxHeight, apexX);
      }
    }

    // Timed out without reaching target
    return new TrajectoryResult(0, maxTime, false, maxHeight, apexX);
  }

  // state = [x, z, vx, vz]
  // ax = -kDrag * |v| * vx
  // az = -g - kDrag * |v| * vz + kMagnus * |v|^2 (Magnus acts as upward lift)
  private double[] derivatives(double[] state) {
    double svx = state[2];
    double svz = state[3];
    double speed = Math.hypot(svx, svz);

    double ax = -kDrag * speed * svx;
    double az = -9.81 - kDrag * speed * svz + magnusSign * kMagnus * speed * speed;

    return new double[] {svx, svz, ax, az};
  }

  private static double[] addScaled(double[] base, double[] delta, double scale) {
    return new double[] {
      base[0] + delta[0] * scale,
      base[1] + delta[1] * scale,
      base[2] + delta[2] * scale,
      base[3] + delta[3] * scale
    };
  }

  public LUTEntry findRPMForDistance(double distanceM) {
    return findRPMForDistance(distanceM, params.fixedLaunchAngleDeg());
  }

  /**
   * Binary search for the RPM at a specific launch angle that puts the ball at the target height.
   */
  public LUTEntry findRPMForDistance(double distanceM, double launchAngleDeg) {
    double heightTolerance = 0.02; // 2cm
    double lo = params.rpmMin();
    double hi = params.rpmMax();

    // Quick feasibility check: can max RPM even reach this distance?
    TrajectoryResult maxCheck = simulate(hi, distanceM, launchAngleDeg);
    if (!maxCheck.reachedTarget()) {
      return new LUTEntry(distanceM, 0, 0, false);
    }

    double bestRpm = hi;
    double bestTof = maxCheck.tof();
    double bestError = Math.abs(maxCheck.zAtTarget() - params.targetHeightM());

    for (int i = 0; i < params.binarySearchIters(); i++) {
      double mid = (lo + hi) / 2.0;
      TrajectoryResult result = simulate(mid, distanceM, launchAngleDeg);

      if (!result.reachedTarget()) {
        // Too slow, need more RPM
        lo = mid;
        continue;
      }

      double error = result.zAtTarget() - params.targetHeightM();
      double absError = Math.abs(error);

      if (absError < bestError) {
        bestRpm = mid;
        bestTof = result.tof();
        bestError = absError;
      }

      if (absError < heightTolerance) {
        return new LUTEntry(distanceM, mid, result.tof(), true);
      }

      if (error > 0) {
        // Ball too high, reduce RPM
        hi = mid;
      } else {
        // Ball too low, increase RPM
        lo = mid;
      }
    }

    // Return best found even if not perfectly converged (0.004 RPM precision after 25 iters)
    return new LUTEntry(distanceM, bestRpm, bestTof, bestError < 0.10);
  }

  /** Generate the full lookup table: 0.50m to 5.00m in 5cm steps (91 entries). Takes ~200ms. */
  public GeneratedLUT generateLUT() {
    return generateLUT(0.50, 5.00, 0.05);
  }

  public GeneratedLUT generateLUT(double minDistM, double maxDistM, double stepM) {
    long startMs = System.currentTimeMillis();
    List<LUTEntry> entries = new ArrayList<>();
    int reachable = 0;
    int unreachable = 0;
    double maxRange = 0;

    for (double distance = minDistM; distance <= maxDistM + stepM * 0.01; distance += stepM) {
      // Round to avoid floating-point drift
      distance = Math.round(distance * 100.0) / 100.0;

      LUTEntry entry = findRPMForDistance(distance);
      entries.add(entry);

      if (entry.reachable()) {
        reachable++;
        maxRange = distance;
      } else {
        unreachable++;
      }
    }

    long elapsed = System.currentTimeMillis() - startMs;
    return new GeneratedLUT(entries, params, reachable, unreachable, maxRange, elapsed);
  }

  //   /** Same as generateLUT() but returns a ShotLUT with the fixed angle baked into each entry.
  // */
  //   public ShotLUT generateShotLUT() {
  //     GeneratedLUT gen = generateLUT();
  //     ShotLUT lut = new ShotLUT();
  //     for (LUTEntry entry : gen.entries()) {
  //       if (entry.reachable()) {
  //         lut.put(entry.distanceM(),
  //             new ShotParameters(entry.rpm(), params.fixedLaunchAngleDeg(), entry.tof()));
  //       }
  //     }
  //     return lut;
  //   }

  //   /**
  //    * Sweep both launch angle and RPM at each distance, picking the angle that needs the
  //    * lowest RPM (fastest flywheel recovery between shots). For adjustable hood mechanisms.
  //    *
  //    * <p>Slower than generateLUT() since it searches across the angle range at every distance.
  //    * Still runs in a few seconds for typical ranges.
  //    */
  //   public ShotLUT generateVariableAngleShotLUT(
  //       double minAngleDeg, double maxAngleDeg, double angleStepDeg) {
  //     ShotLUT lut = new ShotLUT();

  //     for (double distance = 0.50; distance <= 5.00 + 0.001; distance += 0.05) {
  //       distance = Math.round(distance * 100.0) / 100.0;

  //       double bestRPM = Double.MAX_VALUE;
  //       double bestAngle = 0;
  //       double bestTOF = 0;
  //       boolean found = false;

  //       for (double angle = minAngleDeg; angle <= maxAngleDeg + 0.001; angle += angleStepDeg) {
  //         LUTEntry entry = findRPMForDistance(distance, angle);
  //         if (entry.reachable() && entry.rpm() < bestRPM) {
  //           bestRPM = entry.rpm();
  //           bestAngle = angle;
  //           bestTOF = entry.tof();
  //           found = true;
  //         }
  //       }

  //       if (found) {
  //         lut.put(distance, new ShotParameters(bestRPM, bestAngle, bestTOF));
  //       }
  //     }
  //     return lut;
  //   }

  //   /** RPM to ball exit speed without needing a ProjectileSimulator instance. */
  //   public static double rpmToExitVelocity(double rpm, double wheelDiameterM, double slipFactor)
  // {
  //     return slipFactor * rpm * Math.PI * wheelDiameterM / 60.0;
  //   }

  //   /** Ball exit speed to RPM. Inverse of rpmToExitVelocity. */
  //   public static double exitVelocityToRPM(
  //       double exitVelMps, double wheelDiameterM, double slipFactor) {
  //     return exitVelMps * 60.0 / (slipFactor * Math.PI * wheelDiameterM);
  //   }

  //   // Package-private for testing
  //   double getKDrag() {
  //     return kDrag;
  //   }

  //   double getKMagnus() {
  //     return kMagnus;
  //   }
}

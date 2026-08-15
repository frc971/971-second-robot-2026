// Copyright (c) 2025-2026 KAISER 6989
// https://github.com/haar09/FRC-Rebuilt-BumpSim
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Claude Sonnet 4.6 is used for code generation and refactoring.

package frc.robot.lib.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * RobotBumpSim — standalone robot-bump physics simulation for MapleSim.
 *
 * <h2>What this class does</h2>
 *
 * <p>Simulates the robot's 3D pose (Z height, pitch, and roll) as it drives over the raised bumps
 * on the 2026 FRC REBUILT field. It also implements a frictionless-slide model that prevents the
 * robot from "ghosting" through a bump when it lacks the speed to crest it.
 *
 * <h2>How it works</h2>
 *
 * <ul>
 *   <li>Four swerve module contact points are tracked independently in the XZ plane.
 *   <li>When any module first touches a bump's ascending face the sim enters <em>ramp mode</em>: it
 *       captures the robot's current field-X velocity ({@code simXVel}) and absolute field-X
 *       position ({@code simXPos}), then owns them for the duration of the crossing attempt.
 *   <li>While on the ramp, only gravity acts along X (frictionless surface). If {@code simXVel}
 *       decays to zero before the peak the robot slides back to flat ground.
 *   <li>The robot exits ramp mode when it either backs off (all module Z ≈ 0) or successfully
 *       crosses over the peak (modules come back to flat ground on the far side).
 *   <li>A diagonal approach reduces effective deceleration via {@code contactFactor = |cos(2 *
 *       robotYaw)|}, making 45° crossings easier.
 * </ul>
 *
 * <p>Minimum crossing speed: {@code sqrt(2·g·h) ≈ sqrt(2·9.81·0.165) ≈ 1.80 m/s}.
 *
 * <h2>Quick-start integration with MapleSim</h2>
 *
 * <pre>{@code
 * // 1. Construct once, typically in startSimThread() alongside your MapleSim drivetrain.
 * RobotBumpSim robotBumpSim = new RobotBumpSim(drivetrain.getModuleLocations());
 *
 * // 2. Call every simulationPeriodic() AFTER MapleSim has stepped.
 * Pose2d simPose = mapleSimDrive.getSimulatedDriveTrainPose();
 * ChassisSpeeds fieldRelativeSpeeds = mapleSimSwerveDrivetrain.mapleSimDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative();
 *
 * // subticks should match the number of physics sub-steps you pass to your ball/object sim.
 * // A value of 5 works well for a 20 ms loop (4 ms sub-steps).
 * Pose3d simPose3d = robotBumpSim.update(simPose, fieldSpeeds, subticks);
 *
 * // 3. When on the ramp, override MapleSim's pose so the robot physically slides back
 * //    instead of the correction being purely visual.
 * if (robotBumpSim.isOnRamp()) {
 *     mapleSimDrive.setSimulationWorldPose(robotBumpSim.getSimWorldPose(simPose));
 * }
 *
 * // 4. Log or visualise the 3D pose (e.g. with AdvantageScope).
 * Logger.recordOutput("Drive/Pose3d", simPose3d);
 * }</pre>
 *
 * <h2>Tunable constants</h2>
 *
 * <ul>
 *   <li>{@link #WHEEL_RADIUS} — effective contact radius of a wheel against the ramp surface
 *       (metres). Increase this to make the robot appear to "float" higher above the bump.
 *   <li>{@link #CHASSIS_HEIGHT} — offset from the average module-contact Z to the robot body
 *       origin. Set to your chassis clearance height if you want a visually accurate robot body Z.
 *   <li>{@link #BUMP_COR} — coefficient of restitution for vertical collisions with the bump. 0 =
 *       perfectly inelastic (no bounce), 1 = perfectly elastic.
 * </ul>
 *
 * <h2>Field geometry</h2>
 *
 * <p>The bump geometry is encoded as XZ line segments with Y-range guards. Each bump has two
 * segments per side (ascending face + descending face) at X positions symmetric about field centre.
 * See {@link #BUMP_LINE_STARTS} / {@link #BUMP_LINE_ENDS} for the raw coordinates. All positions
 * are in metres, origin at the Blue Alliance driver-station corner.
 */
public class RobotBumpSim {

  // -------------------------------------------------------------------------
  // Field / physics constants
  // -------------------------------------------------------------------------

  /** Robot control-loop period (seconds). Matches the WPILib default of 20 ms. */
  private static final double PERIOD = 0.02;

  /** Gravitational acceleration vector (m/s², pointing in the -Z direction). */
  private static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81);

  /** Full length of the REBUILT field (metres). */
  private static final double FIELD_LENGTH = 16.51;

  /** Full width of the REBUILT field (metres). */
  private static final double FIELD_WIDTH = 8.04;

  /**
   * Start points of the eight bump XZ line segments (four per alliance, ascending + descending).
   *
   * <p>Each {@link Translation3d} stores {@code (fieldX, yMin, fieldZ)}: the world-X start of the
   * ramp face, the minimum field-Y at which this segment is present, and the ramp Z at that X. The
   * companion {@link #BUMP_LINE_ENDS} array stores the end point including the maximum Y.
   *
   * <p>Indices 0–3 are the Blue-side bump (near X ≈ 3.96–5.18 m); indices 4–7 are the Red-side bump
   * (near X ≈ FIELD_LENGTH−5.18 – FIELD_LENGTH−3.96 m).
   */
  static final Translation3d[] BUMP_LINE_STARTS = {
    // Blue bump — ascending faces (Z rises from 0 → 0.165 m)
    new Translation3d(3.96, 1.57, 0),
    new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
    // Blue bump — descending faces (Z falls from 0.165 → 0 m)
    new Translation3d(4.61, 1.57, 0.165),
    new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    // Red bump — ascending faces
    new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
    // Red bump — descending faces
    new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
  };

  /**
   * End points of the eight bump XZ line segments.
   *
   * <p>Each {@link Translation3d} stores {@code (fieldX, yMax, fieldZ)}: the world-X end of the
   * ramp face, the maximum field-Y at which this segment is present, and the ramp Z at that X.
   */
  static final Translation3d[] BUMP_LINE_ENDS = {
    // Blue bump — ascending faces
    new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
    // Blue bump — descending faces
    new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
    new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
    // Red bump — ascending faces
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
    // Red bump — descending faces
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
  };

  /** Index of the first bump segment in {@link #BUMP_LINE_STARTS} (inclusive). */
  private static final int BUMP_LINE_FIRST = 0;

  /** Index of the last bump segment in {@link #BUMP_LINE_STARTS} (inclusive). */
  private static final int BUMP_LINE_LAST = BUMP_LINE_STARTS.length - 1;

  // -------------------------------------------------------------------------
  // Tunable physics constants — adjust for your robot
  // -------------------------------------------------------------------------

  /** Effective wheel contact radius against the bump surface (metres). */
  private static final double WHEEL_RADIUS = 0.048;

  /** Height offset from the average module-contact Z to the robot-body origin (metres). */
  private static final double CHASSIS_HEIGHT = 0.0;

  /**
   * Coefficient of restitution for vertical (Z) robot-bump collisions. 0 = perfectly inelastic (no
   * bounce), 1 = perfectly elastic.
   */
  private static final double BUMP_COR = 0.15;

  /**
   * Floating-point epsilon used to decide whether a projected point falls within a line segment. A
   * small positive tolerance prevents false misses caused by rounding when the module lands exactly
   * on a segment endpoint.
   */
  private static final double SEGMENT_PROJECTION_TOLERANCE = 1e-6;

  // -------------------------------------------------------------------------
  // Per-instance state
  // -------------------------------------------------------------------------

  /** Robot-relative module positions, order: FL(0), FR(1), BL(2), BR(3). */
  private final Translation2d[] moduleOffsets;

  /** Absolute Z position of each module contact point (metres above the floor). */
  private final double[] moduleZPos;

  /** Z velocity of each module contact point (m/s). */
  private final double[] moduleZVel;

  /** Distance between front and back module pairs along the robot X axis (metres). */
  private final double frontBackDist;

  /** Distance between left and right module pairs along the robot Y axis (metres). */
  private final double leftRightDist;

  /**
   * True while the robot is on the ramp and this sim owns the field-X position. The caller must
   * call {@link #getSimWorldPose(Pose2d)} and apply it to MapleSim via {@code
   * setSimulationWorldPose} whenever this is true.
   */
  private boolean onRamp = false;

  /**
   * Absolute field-X position of the robot while on the ramp (frictionless model). Ignored when
   * {@link #onRamp} is false.
   */
  private double simXPos = 0.0;

  /**
   * Field-X velocity of the robot on the ramp (m/s, frictionless model). Initialized to the robot's
   * field-Vx at first ramp contact; decelerated by gravity-along-ramp each subtick. Ignored when
   * {@link #onRamp} is false.
   */
  private double simXVel = 0.0;

  // -------------------------------------------------------------------------
  // Constructor
  // -------------------------------------------------------------------------

  /**
   * Creates a new {@code RobotBumpSim} for a swerve drivetrain.
   *
   * @param moduleOffsets Robot-relative module positions in order FL, FR, BL, BR (metres).
   *     Typically obtained via {@code CommandSwerveDrivetrain#getModuleLocations()}.
   */
  public RobotBumpSim(Translation2d[] moduleOffsets) {
    this.moduleOffsets = moduleOffsets;
    this.moduleZPos = new double[4];
    this.moduleZVel = new double[4];

    double frontX = (moduleOffsets[0].getX() + moduleOffsets[1].getX()) / 2.0;
    double backX = (moduleOffsets[2].getX() + moduleOffsets[3].getX()) / 2.0;
    frontBackDist = Math.max(Math.abs(frontX - backX), 1e-3);

    double leftY = (moduleOffsets[0].getY() + moduleOffsets[2].getY()) / 2.0;
    double rightY = (moduleOffsets[1].getY() + moduleOffsets[3].getY()) / 2.0;
    leftRightDist = Math.max(Math.abs(leftY - rightY), 1e-3);
  }

  // -------------------------------------------------------------------------
  // Public API
  // -------------------------------------------------------------------------

  /**
   * Returns {@code true} while the robot is on the ramp in frictionless-slide mode.
   *
   * <p>When {@code true} the caller must call {@link #getSimWorldPose(Pose2d)} to obtain the
   * corrected 2D pose and feed it to MapleSim via {@code setSimulationWorldPose}, so the robot
   * actually slides backward rather than just appearing to.
   */
  public boolean isOnRamp() {
    return onRamp;
  }

  /**
   * Returns the 2D pose that should be set on MapleSim while on the ramp.
   *
   * <p>The X coordinate is replaced with the frictionless {@link #simXPos}; Y and rotation are
   * taken from {@code latestMaplePose} so MapleSim continues to own lateral motion.
   *
   * @param latestMaplePose The most recent 2D pose read from MapleSim (used for Y and rotation).
   * @return A corrected {@link Pose2d} to pass to {@code setSimulationWorldPose}.
   */
  public Pose2d getSimWorldPose(Pose2d latestMaplePose) {
    return new Pose2d(simXPos, latestMaplePose.getY(), latestMaplePose.getRotation());
  }

  /**
   * Advances the bump simulation by one 20 ms period and returns the robot's 3D pose.
   *
   * <p>When the robot is on the ramp the returned pose uses {@link #simXPos} for X, giving a
   * physically accurate visual position. The caller must also call {@link #getSimWorldPose(Pose2d)}
   * and apply it to MapleSim so the actual simulation position matches (see the class-level Javadoc
   * for a complete integration example).
   *
   * @param robotPose2d Robot's 2D pose from the MapleSim drivetrain.
   * @param fieldRelativeSpeeds Field-relative chassis speeds from the MapleSim drivetrain.
   * @param subticks Physics sub-steps per period. Must match the {@code subticks} value used by any
   *     companion ball/object sim so they stay in sync. Typical value: 5 (= 4 ms sub-steps per 20
   *     ms loop).
   * @return A {@link Pose3d} with physically correct X, Z, pitch, and roll.
   */
  public Pose3d update(Pose2d robotPose2d, ChassisSpeeds fieldRelativeSpeeds, int subticks) {
    double vx = fieldRelativeSpeeds.vxMetersPerSecond;
    double dt = PERIOD / subticks;

    // contactFactor reduces effective gravity deceleration for diagonal crossings.
    //   yaw = 0°  -> |cos(0)|    = 1.0  -> full deceleration (straight-on, hardest to cross)
    //   yaw = 45° -> |cos(90°)|  = 0.71 -> reduced deceleration (diagonal, easier)
    //   yaw = 90° -> |cos(180°)| = 1.0  -> full deceleration (sideways approach also hard)
    double contactFactor = Math.abs(Math.cos(2 * robotPose2d.getRotation().getRadians()));

    // Y positions of each module (MapleSim-owned, constant for the whole period)
    double[] worldY = new double[4];
    for (int i = 0; i < 4; i++) {
      Translation2d wo = moduleOffsets[i].rotateBy(robotPose2d.getRotation());
      worldY[i] = robotPose2d.getY() + wo.getY();
    }

    for (int tick = 0; tick < subticks; tick++) {
      // Use simXPos (frictionless) when on ramp, else follow MapleSim's X
      double currentRobotX = onRamp ? simXPos : robotPose2d.getX();

      double gravAccelXSum = 0.0;
      int contactCount = 0;

      for (int i = 0; i < 4; i++) {
        // Compute this module's world-X from current robot-X
        Translation2d wo = moduleOffsets[i].rotateBy(robotPose2d.getRotation());
        double wx = currentRobotX + wo.getX();

        // Z gravity and integration
        moduleZVel[i] += GRAVITY.getZ() * dt;
        moduleZPos[i] += moduleZVel[i] * dt;

        // Bump collisions. Stop at the first matching segment: adjacent segments (e.g.
        // ascending/descending faces) share an endpoint at the bump peak, and resolving
        // more than one per module per substep double-applies the Z correction/impulse
        // there, which is what was causing the violent spin right before cresting.
        for (int lineIdx = BUMP_LINE_FIRST; lineIdx <= BUMP_LINE_LAST; lineIdx++) {
          double gax = handleModuleBumpCollision(i, wx, worldY[i], onRamp ? simXVel : vx, lineIdx);
          if (!Double.isNaN(gax)) {
            gravAccelXSum += gax;
            contactCount++;
            break;
          }
        }

        // Floor
        if (moduleZPos[i] < 0.0) {
          moduleZPos[i] = 0.0;
          if (moduleZVel[i] < 0.0) moduleZVel[i] = -moduleZVel[i] * BUMP_COR;
        }
      }

      if (contactCount > 0) {
        if (!onRamp) {
          // First ramp contact: capture current kinematics
          onRamp = true;
          simXPos = robotPose2d.getX();
          simXVel = vx;
        }
        // Apply average gravity-along-ramp deceleration (frictionless surface)
        double avgGravAccelX = (gravAccelXSum / contactCount) * contactFactor;
        simXVel += avgGravAccelX * dt;
        simXPos += simXVel * dt;
      } else if (onRamp) {
        // No ramp contact; check if all modules have settled onto flat ground
        boolean allFlat = true;
        for (int i = 0; i < 4; i++) {
          if (moduleZPos[i] > 0.01) {
            allFlat = false;
            break;
          }
        }
        if (allFlat) {
          // Robot has backed out or crossed — MapleSim reclaims X
          onRamp = false;
        } else {
          // Briefly airborne after cresting the peak: keep sliding under simXVel
          simXPos += simXVel * dt;
        }
      }
    }

    return computePose3d(robotPose2d);
  }

  // -------------------------------------------------------------------------
  // Private helpers
  // -------------------------------------------------------------------------

  /**
   * Handles the XZ-plane bump collision for module {@code moduleIdx} against line segment {@code
   * lineIdx}. Applies a Z position correction and Z velocity impulse. Returns the
   * gravity-along-ramp X acceleration (m/s²) if in contact, or {@link Double#NaN} if not.
   *
   * <p>The returned acceleration is:
   *
   * <pre>
   *   a_X = -g * normalX * normalZ
   * </pre>
   *
   * Ascending face (normalX &lt; 0, normalZ &gt; 0): a_X &lt; 0 — gravity pulls robot back.<br>
   * Descending face (normalX &gt; 0, normalZ &gt; 0): a_X &gt; 0 — gravity helps robot forward.
   *
   * @param moduleIdx Index of the module (0–3).
   * @param worldX Module's world-X position (metres).
   * @param worldY Module's world-Y position (metres).
   * @param currentXVel The robot's current field-X velocity (simXVel when on ramp, else vx).
   * @param lineIdx Index into {@link #BUMP_LINE_STARTS} / {@link #BUMP_LINE_ENDS}.
   * @return Gravity-along-ramp X acceleration, or {@link Double#NaN} if not in contact.
   */
  private double handleModuleBumpCollision(
      int moduleIdx, double worldX, double worldY, double currentXVel, int lineIdx) {
    Translation3d lineStart = BUMP_LINE_STARTS[lineIdx];
    Translation3d lineEnd = BUMP_LINE_ENDS[lineIdx];

    // Y-range guard
    if (worldY < lineStart.getY() || worldY > lineEnd.getY()) return Double.NaN;

    // Project into the XZ plane
    Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
    Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
    Translation2d pos2d = new Translation2d(worldX, moduleZPos[moduleIdx]);
    Translation2d lineVec = end2d.minus(start2d);

    // Closest point on the XZ segment to the module (parametric projection)
    Translation2d toModule = pos2d.minus(start2d);
    double projectionT = toModule.dot(lineVec) / lineVec.getSquaredNorm();
    Translation2d projected = start2d.plus(lineVec.times(projectionT));

    if (projected.getDistance(start2d) + projected.getDistance(end2d)
        > lineVec.getNorm() + SEGMENT_PROJECTION_TOLERANCE) return Double.NaN; // off segment

    double dist = pos2d.getDistance(projected);
    if (dist > WHEEL_RADIUS) return Double.NaN; // not intersecting

    // Outward normal in XZ: lineVec = (deltaX, deltaZ) -> normal = (-deltaZ, deltaX) / |lineVec|
    double normalX = -lineVec.getY() / lineVec.getNorm();
    double normalZ = lineVec.getX() / lineVec.getNorm();

    // Z position correction: push module to WHEEL_RADIUS from ramp surface
    moduleZPos[moduleIdx] += normalZ * (WHEEL_RADIUS - dist);

    // Z velocity impulse (visual tilt only)
    double velDotNormal = currentXVel * normalX + moduleZVel[moduleIdx] * normalZ;
    if (velDotNormal < 0.0) {
      moduleZVel[moduleIdx] += normalZ * (-(1.0 + BUMP_COR) * velDotNormal);
    }

    // Gravity-along-ramp X acceleration (frictionless surface, no drive force contribution).
    // GRAVITY.getZ() = -9.81 m/s².
    //   Ascending face: normalX < 0, normalZ > 0
    //     -> a_X = -(-9.81) * (negative) * (positive) < 0  -- decelerates +X travel
    //   Descending face: normalX > 0, normalZ > 0
    //     -> a_X = -(-9.81) * (positive) * (positive) > 0  -- accelerates +X travel
    return -GRAVITY.getZ() * normalX * normalZ;
  }

  /**
   * Derives a {@link Pose3d} from the robot's 2D pose and the four module Z positions. Pitch and
   * roll come from front/back and left/right height differences. X uses {@link #simXPos} when on
   * the ramp for visual accuracy.
   */
  private Pose3d computePose3d(Pose2d robotPose2d) {
    // FL=0, FR=1, BL=2, BR=3
    double frontZ = (moduleZPos[0] + moduleZPos[1]) / 2.0;
    double backZ = (moduleZPos[2] + moduleZPos[3]) / 2.0;
    double leftZ = (moduleZPos[0] + moduleZPos[2]) / 2.0;
    double rightZ = (moduleZPos[1] + moduleZPos[3]) / 2.0;
    double centerZ = (frontZ + backZ) / 2.0 + CHASSIS_HEIGHT;

    // Pitch: positive -> nose up (front higher than back)
    // Negated because WPILib Rotation3d pitch positive = nose-down (right-hand rule)
    double pitch = -Math.atan2(frontZ - backZ, frontBackDist);
    // Roll: positive -> left side higher than right side
    double roll = Math.atan2(leftZ - rightZ, leftRightDist);

    // X: use frictionless sim position on ramp, else follow MapleSim
    double visualX = onRamp ? simXPos : robotPose2d.getX();

    return new Pose3d(
        visualX,
        robotPose2d.getY(),
        centerZ,
        new Rotation3d(roll, pitch, robotPose2d.getRotation().getRadians()));
  }
}

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.lib.shooter.*;
import frc.robot.lib.superstructure.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controllers;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterHandler {
  public static final class Targets {
    private static final double X_DISTANCE_FROM_CENTER = 3.6448975;

    public static final ObjectState BLUE =
        new ObjectState(
            new Translation2d(
                (FlippingUtil.fieldSizeX / 2) - X_DISTANCE_FROM_CENTER,
                (FlippingUtil.fieldSizeY / 2)),
            new Translation2d());
    public static final ObjectState RED =
        new ObjectState(FlippingUtil.flipFieldPosition(BLUE.position()), new Translation2d());

    // offset from the corner
    private static final Translation2d SHUTTLE_OFFSET = new Translation2d(1.8, 2.0);

    public static final ObjectState RIGHT_BLUE_SHUTTLE =
        new ObjectState(SHUTTLE_OFFSET, new Translation2d());
    public static final ObjectState LEFT_BLUE_SHUTTLE =
        new ObjectState(
            new Translation2d(
                SHUTTLE_OFFSET.getX(), FlippingUtil.fieldSizeY - SHUTTLE_OFFSET.getY()),
            new Translation2d());
    public static final ObjectState RIGHT_RED_SHUTTLE =
        new ObjectState(
            FlippingUtil.flipFieldPosition(RIGHT_BLUE_SHUTTLE.position()), new Translation2d());
    public static final ObjectState LEFT_RED_SHUTTLE =
        new ObjectState(
            FlippingUtil.flipFieldPosition(LEFT_BLUE_SHUTTLE.position()), new Translation2d());
  }

  // config + physics model
  private ShooterConfig config;
  private ShooterPhysics physics;
  private ObjectState projectileState;
  private @Setter @Getter ObjectState targetState;
  private final String name;

  @AutoLogOutput(key = "{name}/flywheelOffset")
  private AngularVelocity flywheelOffset = RotationsPerSecond.of(0.0);

  @AutoLogOutput(key = "{name}/turretOffset")
  private Angle turretOffset = Degrees.of(0.0);

  @AutoLogOutput(key = "{name}/desiredFlywheel")
  private AngularVelocity desiredFlywheel = RotationsPerSecond.of(0.0);

  @AutoLogOutput(key = "{name}/desiredTurret")
  private Angle desiredTurretRel = Degrees.of(0.0);

  private static final AngularVelocity FLYWHEEL_STEP = RotationsPerSecond.of(2.0);
  private static final Angle TURRET_STEP = Degrees.of(2.0);

  private static final Time TURRET_TIME_DELAY = Seconds.of(0.1);

  // state machine
  public enum State {
    NOT_READY,
    AIMING,
    FIRING
  }

  public enum Goal {
    NONE,
    ACTIVE
  }

  @AutoLogOutput(key = "{name}/shooterState")
  @Getter
  private ShooterHandler.State shooterState;

  @AutoLogOutput(key = "{name}/shooterGoal")
  @Getter
  @Setter
  private ShooterHandler.Goal shooterGoal;

  private LaunchSolution launchSolution = null;

  // needs access DriveTrain (for robotState)
  private final CommandSwerveDrivetrain drivetrain;
  private final AngularSubsystem turret;
  private final AngularSubsystem hood;
  private final AngularSubsystem flywheel;

  @AutoLogOutput(key = "{name}/TuningEnabled")
  @Setter
  private boolean tuningEnabled = false;

  public ShooterHandler(
      AngularSubsystem turret,
      AngularSubsystem hood,
      AngularSubsystem flywheel,
      CommandSwerveDrivetrain drivetrain,
      ShooterConfig config) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.flywheel = flywheel;
    this.hood = hood;
    this.config = config;
    this.physics = new ShooterPhysics(this.config.PHYSICS());
    this.name = config.name();

    this.shooterState = State.NOT_READY;
    this.shooterGoal = Goal.NONE;
    this.targetState = Targets.BLUE;
    this.projectileState = Targets.BLUE;

    this.launchSolution = physics.iterativeTimeSolve(getProjectileState(), Targets.BLUE, 1);
  }

  public void periodic() {
    projectileState = getProjectileState();

    // calculate physics
    launchSolution = physics.iterativeTimeSolve(projectileState, targetState, 20);

    liveTuning(); // live tuning during matches & superstructure decides which one is enabled
    if (shooterGoal == Goal.NONE) {
      shooterState = State.NOT_READY;
      return;
    }

    if (!satisfiesConstraints()) {
      shooterState = State.NOT_READY;
    }

    // state transitions
    switch (shooterState) {
      case NOT_READY -> {
        if (satisfiesConstraints()) {
          shooterState = State.AIMING;
        }
      }
      case AIMING -> {
        if (launchSolution != null && canTransitionToReady()) {
          shooterState = State.FIRING;
        }
      }
      case FIRING -> {
        if (canTransitionToNotReady()) {
          shooterState = State.AIMING;
        }
      }
    }

    // --- compute tuned + clamped desired goals (used for outputs AND error) ---
    if (launchSolution == null || shooterState == State.NOT_READY) {
      desiredFlywheel = RotationsPerSecond.of(0.0);
      desiredTurretRel = Degrees.of(0.0);
    } else {
      // Base goals from physics
      AngularVelocity flywheelGoal = getFlywheelSpeed(); // includes fudge factor
      Angle turretGoalRel = getRelativeTurretAngle();

      // Apply live-tuning offsets TO THE GOALS
      flywheelGoal = flywheelGoal.plus(flywheelOffset);
      turretGoalRel = turretGoalRel.plus(turretOffset);

      // Clamp flywheel goal (do not exceed constraints)
      flywheelGoal =
          RadiansPerSecond.of(
              MathUtil.clamp(
                  flywheelGoal.in(RadiansPerSecond),
                  config.CONSTRAINTS().MIN_FLYWHEEL_SPEED().in(RadiansPerSecond),
                  config.CONSTRAINTS().MAX_FLYWHEEL_SPEED().in(RadiansPerSecond)));

      desiredFlywheel = flywheelGoal;
      desiredTurretRel = turretGoalRel;
    }

    logStates();

    // set output
    if (shooterState != State.NOT_READY) {
      flywheel.setVelocity(desiredFlywheel);

      // turret has its own hard-stop clamp in TurretLeft/Right.setPosition()
      turret.setPosition(desiredTurretRel);
    }

    // ShooterHandler no longer commands hood here.
    // Superstructure applies hood via Optional<Angle> when indexing.
  }

  private void liveTuning() {
    if (!tuningEnabled) return;

    if (Controllers.FLYWHEEL_UP.rising()) flywheelOffset = flywheelOffset.plus(FLYWHEEL_STEP);
    if (Controllers.FLYWHEEL_DOWN.rising()) flywheelOffset = flywheelOffset.minus(FLYWHEEL_STEP);
    if (Controllers.TURRET_LEFT.rising()) turretOffset = turretOffset.plus(TURRET_STEP);
    if (Controllers.TURRET_RIGHT.rising()) turretOffset = turretOffset.minus(TURRET_STEP);
  }

  public Optional<Angle> getDesiredHoodAngle() {
    if (launchSolution == null || shooterState == State.NOT_READY) {
      return Optional.empty();
    }

    return Optional.of(launchSolution.hoodAngle());
  }

  public AngularVelocity getFlywheelSpeed() {
    return launchSolution.flywheelSpeed();
  }

  public Angle getRelativeTurretAngle() {
    Angle absolute = Degrees.of(launchSolution.turretRotation.getDegrees());
    Angle relative =
        absolute.minus(Degrees.of(drivetrain.getState().Pose.getRotation().getDegrees()));

    return Radians.of(MathUtil.angleModulus(relative.in(Radians)));
  }

  public Angle getFutureRelativeTurretAngle() {
    LaunchSolution turretAheadSolution =
        physics.iterativeTimeSolve(
            projectileState.getFutureState(this.TURRET_TIME_DELAY), targetState, 20);

    Angle absolute = Degrees.of(turretAheadSolution.turretRotation.getDegrees());
    Angle relative =
        absolute.minus(Degrees.of(drivetrain.getState().Pose.getRotation().getDegrees()));

    return Radians.of(MathUtil.angleModulus(relative.in(Radians)));
  }

  private void logStates() {
    if (launchSolution != null) {
      Logger.recordOutput(
          name + "/LaunchGoals/Flywheel (rps)",
          launchSolution.flywheelSpeed().in(RotationsPerSecond));
      Logger.recordOutput(
          name + "/LaunchGoals/Turret (deg)",
          Radians.of(launchSolution.turretRotation.getRadians()).in(Degrees));
      Logger.recordOutput(
          name + "/LaunchGoals/Turret Rel (deg)", getRelativeTurretAngle().in(Degrees));
      Logger.recordOutput(
          name + "/LaunchGoals/Turret Future Rel (deg)",
          getFutureRelativeTurretAngle().in(Degrees));
      Logger.recordOutput(name + "/LaunchGoals/Hood (deg)", launchSolution.hoodAngle().in(Degrees));
      Translation2d distance2d = targetState.minus(projectileState).position();
      Logger.recordOutput(name + "/Distance/2D", distance2d);
      Logger.recordOutput(name + "/Distance/1D", distance2d.getNorm());

      Logger.recordOutput(
          name + "/Error/Flywheel (rps)", flywheelSpeedAbsDiff().in(RotationsPerSecond));
      Logger.recordOutput(name + "/Error/Turret (deg)", turretRotationAbsDiff().in(Degrees));
      Logger.recordOutput(name + "/Error/Hood (physics deg)", hoodAngleAbsDiff().in(Degrees));
    }

    if (projectileState != null) {
      Logger.recordOutput(name + "/Projectile Position", projectileState.position());
      Logger.recordOutput(name + "/Projectile Velocity", projectileState.velocity());
      Logger.recordOutput(name + "/Target Position", targetState.position());
      Logger.recordOutput(name + "/Target Velocity", targetState.velocity());
    }
  }

  @AutoLogOutput(key = "{name}/canTransitionToReady")
  private boolean canTransitionToReady() {
    if (launchSolution == null) {
      return false;
    }

    return flywheelSpeedAbsDiff().lt(config.THRESHOLD().AIMING_FLYWHEEL_THRESHOLD())
        && turretRotationAbsDiff().lt(config.THRESHOLD().AIMING_ROTATION_THRESHOLD())
        && hoodAngleAbsDiff().lt(config.THRESHOLD().AIMING_HOOD_ANGLE_THRESHOLD());
  }

  @AutoLogOutput(key = "{name}/canTransitionToNotReady")
  private boolean canTransitionToNotReady() {
    if (launchSolution == null) {
      return true;
    }

    if (targetState == Targets.BLUE || targetState == Targets.RED) {
      return flywheelSpeedAbsDiff().gt(config.THRESHOLD().SHOOTING_FLYWHEEL_ABORT())
          || turretRotationAbsDiff().gt(config.THRESHOLD().SHOOTING_ROTATION_THRESHOLD())
          || hoodAngleAbsDiff().gt(config.THRESHOLD().SHOOTING_HOOD_ANGLE_THRESHOLD());
    } else {
      return flywheelSpeedAbsDiff().gt(config.THRESHOLD().SHUTTLING_FLYWHEEL_THRESHOLD())
          || turretRotationAbsDiff().gt(config.THRESHOLD().SHUTTLING_ROTATION_THRESHOLD())
          || hoodAngleAbsDiff().gt(config.THRESHOLD().SHUTTLING_HOOD_ANGLE_THRESHOLD());
    }
  }

  // --- Threshold helper functions ---
  private AngularVelocity flywheelSpeedAbsDiff() {
    return RadiansPerSecond.of(
        getFlywheelSpeed().minus(flywheel.getVelocity()).abs(RadiansPerSecond));
  }

  private Angle turretRotationAbsDiff() {
    return Radians.of(
        Math.abs(
            MathUtil.angleModulus(
                launchSolution.turretRotation.getMeasure().in(Radians)
                    - MathUtil.angleModulus(getTurretAbsRotation().in(Radians)))));
  }

  private Angle hoodAngleAbsDiff() {
    return Radians.of(launchSolution.hoodAngle().minus(hood.getPosition()).abs(Radians));
  }

  private Angle getTurretAbsRotation() {
    return turret.getPosition().plus(drivetrain.getState().Pose.getRotation().getMeasure());
  }

  // Check if within hardware (+ other) constraints for shooting
  // determines if shot is even remotely possible
  @AutoLogOutput(key = "{name}/satisfiesConstraints")
  public boolean satisfiesConstraints() {
    if (launchSolution == null) {
      DataLogManager.log("WARNING: Launch solution is null");
      return false;
    }

    // --- HIGH PRIORITY CONSTRAINTS ---
    AngularVelocity speed = launchSolution.flywheelSpeed();
    if (speed.lt(config.CONSTRAINTS().MIN_FLYWHEEL_SPEED())
        || speed.gt(config.CONSTRAINTS().MAX_FLYWHEEL_SPEED())) {
      DataLogManager.log("WARNING: Calculated flywheel speed is out constraints");
      return false;
    }

    Angle angle = launchSolution.hoodAngle();
    if (angle.lt(config.CONSTRAINTS().MIN_HOOD_ANGLE())
        || angle.gt(config.CONSTRAINTS().MAX_HOOD_ANGLE())) {
      DataLogManager.log("WARNING: Calculated hood angle is out of constraints");
      return false;
    }

    // --- LOW PRIORITY CONSTRAINTS ---
    Distance targetDistance =
        Meters.of(targetState.position().minus(projectileState.position()).getNorm());
    if (targetDistance.lt(config.CONSTRAINTS().MIN_SHOT_DISTANCE())
        || targetDistance.gt(config.CONSTRAINTS().MAX_SHOT_DISTANCE())) {
      DataLogManager.log("WARNING: Target distance is outside shot distance constraints");
      return false;
    }

    // TODO: add more constraints to check against (such as time)
    return true;
  }

  private ObjectState getProjectileState() {
    // This code is hella chopped but works maybe??
    SwerveDriveState drivetrainState = drivetrain.getState();

    double radiusToBall = config.PHYSICAL_CONVERSION().RADIUS_TO_BALL().in(Meters);
    double robotOmega = drivetrainState.Speeds.omegaRadiansPerSecond;

    double turretAngle = getTurretAbsRotation().in(Radians);
    double turretOmega = turret.getVelocity().in(RadiansPerSecond);

    Translation2d turretCenterToBall = new Translation2d(radiusToBall, new Rotation2d(turretAngle));

    Translation2d robotCenterToTurret =
        config
            .PHYSICAL_CONVERSION()
            .TURRET_XY_OFFSET()
            .toTranslation2d()
            .rotateBy(drivetrainState.Pose.getRotation());

    Translation2d v_robotRot =
        new Translation2d(
            -robotOmega * robotCenterToTurret.getY(), robotOmega * robotCenterToTurret.getX());
    Translation2d v_turRot =
        new Translation2d(
            -turretOmega * turretCenterToBall.getY(), turretOmega * turretCenterToBall.getX());

    Translation2d projPoseOffset = robotCenterToTurret.plus(turretCenterToBall);
    Translation2d projVelOffset = v_robotRot.plus(v_turRot);

    ObjectState robotState = new ObjectState(drivetrainState);

    return robotState.plus(projPoseOffset, projVelOffset);
  }

  public Angle physicsHoodAngle() {
    return launchSolution.hoodAngle();
  }

  public Distance currentDistance() {
    return Meters.of(targetState.minus(projectileState).position().getNorm());
  }

  public ShooterConfig getConfig() {
    return config;
  }
}

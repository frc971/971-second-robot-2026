package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private static final double HUB_HEIGHT = 1.430425;
    private static final Distance CENTER_TO_BACK_HUB_OFFSET = Meters.of(0.52);
    private static final double BACKPLATE_Z = 1.7324;

    public static final ObjectState BLUE =
        new ObjectState(
            new Translation3d(
                (FlippingUtil.fieldSizeX / 2) - X_DISTANCE_FROM_CENTER,
                (FlippingUtil.fieldSizeY / 2),
                HUB_HEIGHT),
            new Translation3d());
    public static final ObjectState RED =
        new ObjectState(
            new Translation3d(
                FlippingUtil.flipFieldPosition(BLUE.xyPos()).getX(),
                FlippingUtil.flipFieldPosition(BLUE.xyPos()).getY(),
                HUB_HEIGHT),
            new Translation3d());

    // offset from the corner
    private static final Translation3d SHUTTLE_OFFSET = new Translation3d(0.8, 1.3, 0.0);

    public static final ObjectState RIGHT_BLUE_SHUTTLE =
        new ObjectState(SHUTTLE_OFFSET, new Translation3d());
    public static final ObjectState LEFT_BLUE_SHUTTLE =
        new ObjectState(
            new Translation3d(
                SHUTTLE_OFFSET.getX(),
                FlippingUtil.fieldSizeY - SHUTTLE_OFFSET.getY(),
                SHUTTLE_OFFSET.getZ()),
            new Translation3d());

    public static final ObjectState RIGHT_RED_SHUTTLE =
        new ObjectState(
            new Translation3d(
                FlippingUtil.flipFieldPosition(RIGHT_BLUE_SHUTTLE.xyPos()).getX(),
                FlippingUtil.flipFieldPosition(RIGHT_BLUE_SHUTTLE.xyPos()).getY(),
                RIGHT_BLUE_SHUTTLE.position().getZ()),
            new Translation3d());

    public static final ObjectState LEFT_RED_SHUTTLE =
        new ObjectState(
            new Translation3d(
                FlippingUtil.flipFieldPosition(LEFT_BLUE_SHUTTLE.xyPos()).getX(),
                FlippingUtil.flipFieldPosition(LEFT_BLUE_SHUTTLE.xyPos()).getY(),
                LEFT_BLUE_SHUTTLE.position().getZ()),
            new Translation3d());
  }

  public enum Side {
    LEFT,
    RIGHT
  }

  // config + physics model
  private ShooterConfig config;
  private ShooterPhysics physics;
  private ObjectState projectileState;
  private @Setter @Getter ObjectState targetState;
  private final String name;
  private final Side side;

  @Setter
  @Getter
  @AutoLogOutput(key = "{name}/Use OTF")
  private boolean useOTF = true;

  @AutoLogOutput(key = "{name}/flywheelOffset")
  @Getter
  private AngularVelocity flywheelOffset = RotationsPerSecond.of(0.0);

  @AutoLogOutput(key = "{name}/turretOffset")
  @Getter
  private Angle turretOffset = Degrees.of(0.0);

  private AngularVelocity desiredFlywheel = RotationsPerSecond.of(0.0);

  @AutoLogOutput(key = "{name}/desiredTurret")
  private Angle desiredTurretRel = Degrees.of(0.0);

  private static final AngularVelocity FLYWHEEL_STEP = RotationsPerSecond.of(2.0);
  private static final Angle TURRET_STEP = Degrees.of(2.0);

  private static final Distance PERPENDICULAR_TURRET_OFFSET = Meters.of(0.1);

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

  @Getter private LaunchSolution launchSolution = null;

  // needs access DriveTrain (for robotState)
  private final CommandSwerveDrivetrain drivetrain;
  private final AngularSubsystem turret;
  private final Hood hood;
  private final AngularSubsystem flywheel;

  @Setter private boolean tuningEnabled = false;

  public ShooterHandler(
      AngularSubsystem turret,
      Hood hood,
      AngularSubsystem flywheel,
      CommandSwerveDrivetrain drivetrain,
      ShooterConfig config,
      Side side) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.flywheel = flywheel;
    this.hood = hood;
    this.config = config;
    this.physics = new ShooterPhysics(this.config.PHYSICS());
    this.name = config.name();
    this.side = side;

    this.shooterState = State.NOT_READY;
    this.shooterGoal = Goal.NONE;
    this.targetState = Targets.BLUE;
    this.projectileState = Targets.BLUE;

    this.launchSolution = new LaunchSolution();
  }

  public void periodic() {
    projectileState = getProjectileState();

    // clear robot velocity if not using OTF
    if (!useOTF) {
      projectileState = new ObjectState(projectileState.position(), new Translation3d());
    }

    // calculate physics
    if (isShuttle(targetState)) {
      launchSolution = physics.thriceSolve(projectileState, targetState);
    } else {
      // use offset
      Translation2d perpOffset =
          new Translation2d(
              PERPENDICULAR_TURRET_OFFSET.in(Meters),
              new Rotation2d(
                  targetState.minus(projectileState).xyPos().getAngle().getRadians()
                      + ((side == Side.LEFT) ? Math.PI / 2 : -(Math.PI / 2))));

      ObjectState adjustedTargetState =
          targetState.plus(new Translation3d(perpOffset), new Translation3d());

      launchSolution = physics.twiceSolve(projectileState, adjustedTargetState);
    }

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

  public ObjectState getHubTargetPoint(Pose2d robotPose) {
    Translation2d robot = robotPose.getTranslation();

    ObjectState currentHub =
        (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            ? ShooterHandler.Targets.BLUE
            : ShooterHandler.Targets.RED;

    Translation2d centerToRobot = robot.minus(currentHub.xyPos());
    double distance = centerToRobot.getNorm();
    Translation2d unitDir = centerToRobot.div(distance);
    Translation2d goal2D =
        currentHub.xyPos().minus(unitDir.times(Targets.CENTER_TO_BACK_HUB_OFFSET.in(Meters)));

    ObjectState goal3D =
        new ObjectState(
            new Translation3d(goal2D.getX(), goal2D.getY(), Targets.BACKPLATE_Z),
            new Translation3d());
    Logger.recordOutput(
        name + "/Hub Target Point", new Pose3d(goal3D.position(), Rotation3d.kZero));

    return goal3D;
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
      Logger.recordOutput(name + "/LaunchGoals/Hood (deg)", launchSolution.hoodAngle().in(Degrees));
      Translation2d distance2d = targetState.minus(projectileState).xyPos();
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
      return flywheelSpeedAbsDiff().gt(config.THRESHOLD().FIRING_FLYWHEEL_ABORT())
          || turretRotationAbsDiff().gt(config.THRESHOLD().FIRING_ROTATION_THRESHOLD())
          || hoodAngleAbsDiff().gt(config.THRESHOLD().FIRING_HOOD_ANGLE_THRESHOLD());
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

  public Translation2d getDirectRelativeTranslation() {
    return targetState
        .minus(projectileState)
        .xyPos()
        .rotateBy(drivetrain.getState().Pose.getRotation().unaryMinus());
  }

  private Angle hoodAngleAbsDiff() {
    return Radians.of(launchSolution.hoodAngle().minus(hood.getHoodAngle()).abs(Radians));
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
      return false;
    }

    Angle angle = launchSolution.hoodAngle();
    if (angle.lt(config.CONSTRAINTS().MIN_HOOD_ANGLE())
        || angle.gt(config.CONSTRAINTS().MAX_HOOD_ANGLE())) {
      return false;
    }

    // --- LOW PRIORITY CONSTRAINTS ---
    Distance targetDistance =
        Meters.of(targetState.xyPos().minus(projectileState.xyPos()).getNorm());
    if (targetDistance.lt(config.CONSTRAINTS().MIN_SHOT_DISTANCE())
        || targetDistance.gt(config.CONSTRAINTS().MAX_SHOT_DISTANCE())) {
      return false;
    }

    // TODO: add more constraints to check against (such as time)
    return true;
  }

  public ObjectState getProjectileState() {
    SwerveDriveState drivetrainState = drivetrain.getState();

    double radiusToBall = config.PHYSICAL_CONVERSION().RADIUS_TO_BALL().in(Meters);
    double robotOmega = drivetrainState.Speeds.omegaRadiansPerSecond;

    double turretAngle = getTurretAbsRotation().in(Radians);
    double turretOmega = turret.getVelocity().in(RadiansPerSecond);

    Translation2d turretCenterToBall = new Translation2d(radiusToBall, new Rotation2d(turretAngle));

    Translation2d robotCenterToTurret =
        config
            .PHYSICAL_CONVERSION()
            .TURRET_OFFSET()
            .toTranslation2d()
            .rotateBy(drivetrainState.Pose.getRotation());

    Vector<N3> W_robot = new Translation3d(0, 0, robotOmega).toVector();
    Vector<N3> W_turret = new Translation3d(0, 0, turretOmega).toVector();

    Vector<N3> v_robotRot =
        Vector.cross(W_robot, new Translation3d(robotCenterToTurret).toVector());
    Vector<N3> v_turRot = Vector.cross(W_turret, new Translation3d(turretCenterToBall).toVector());

    Translation3d projPoseOffset =
        new Translation3d(robotCenterToTurret)
            .plus(new Translation3d(turretCenterToBall))
            .plus(new Translation3d(0.0, 0.0, config.PHYSICAL_CONVERSION().TURRET_OFFSET().getZ()));

    Translation3d projVelOffset = new Translation3d(v_robotRot.plus(v_turRot));

    ObjectState robotState = new ObjectState(drivetrainState);

    return robotState.plus(projPoseOffset, projVelOffset);
  }

  private boolean isShuttle(ObjectState target) {
    return target == Targets.LEFT_BLUE_SHUTTLE
        || target == Targets.RIGHT_BLUE_SHUTTLE
        || target == Targets.LEFT_RED_SHUTTLE
        || target == Targets.RIGHT_RED_SHUTTLE;
  }

  public Distance currentDistance() {
    return Meters.of(targetState.minus(projectileState).xyPos().getNorm());
  }

  public ShooterConfig getConfig() {
    return config;
  }
}

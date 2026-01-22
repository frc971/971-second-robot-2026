package frc.robot.lib.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterHandler {

  // config + physics model
  private final ShooterConfig config;
  private final ShooterPhysics physics;
  private ObjectState projectileState;
  private ObjectState targetState;

  // state machine
  public enum State {
    READY,
    NOT_READY
  }

  @AutoLogOutput private ShooterHandler.State shooterState;

  public ShooterHandler.State state() {
    return this.shooterState;
  }

  private LaunchSolution launchSolution = null;

  // needs access DriveTrain (for robotState)
  private final CommandSwerveDrivetrain drivetrain;
  private final RobotContainer robotContainer;

  public ShooterHandler(RobotContainer robotContainer, ShooterConfig config) {
    this.robotContainer = robotContainer;
    this.drivetrain = robotContainer.drivetrain;
    this.config = config;
    this.physics = new ShooterPhysics(this.config);

    this.shooterState = State.NOT_READY;
    this.targetState = config.TARGET_POSE(); // predetermined target from config
  }

  public void periodic() {
    if (launchSolution != null) {
      Logger.recordOutput(
          "ShooterHandler/LaunchGoals/Flywheel (mps)",
          launchFlywheelSpeedMps().in(MetersPerSecond));
      Logger.recordOutput(
          "ShooterHandler/LaunchGoals/Turret (deg)", launchTurretRotation().in(Degrees));
      Logger.recordOutput(
          "ShooterHandler/LaunchGoals/Turret Rel (deg)", getRelativeTurretAngle().in(Degrees));
      Logger.recordOutput("ShooterHandler/LaunchGoals/Hood (deg)", launchHoodAngle().in(Degrees));
      Translation2d distance2d = targetState.minus(projectileState).xyPosition();
      Logger.recordOutput("ShooterHandler/LaunchGoals/distance (x,y)", distance2d);
      Logger.recordOutput("ShooterHandler/LaunchGoals/distance", distance2d.getNorm());
    }

    // --- UPDATE objectStates --
    projectileState = getProjectileState();
    // add getTargetState() here if decide not to store target in config

    // --- CALCULATE physics ---
    launchSolution = physics.alexSolve(projectileState, targetState);

    // --- STATE MACHINE ---
    switch (shooterState) {
      case NOT_READY -> {
        if (meetsThresholds() && launchSolution != null) {
          shooterState = State.READY;
        }
      }
      case READY -> {
        if (!meetsThresholds()) {
          shooterState = State.NOT_READY;
          System.out.println("Outside thresholds");
        }
      }
      default -> {
        // TODO: critical error log
        System.out.println("ShooterHandler State Machine error: entered 'default'");
      }
    }
  }

  // --- Interaction with hardware ---

  public AngularVelocity getFlywheelAngularVelocity() {
    double omega =
        launchSolution.flywheelSpeed().in(MetersPerSecond) / config.FLYWHEEL_RADIUS.in(Meters);

    return Radians.per(Seconds).of(omega);
  }

  public Angle getRelativeTurretAngle() {
    Angle absolute = Degrees.of(launchSolution.turretRotation.getDegrees());
    Angle relative =
        absolute.minus(Degrees.of(drivetrain.getState().Pose.getRotation().getDegrees()));

    return Radians.of(MathUtil.angleModulus(relative.in(Radians)));
  }

  // --- Interaction with hardware ^^^ ---

  public boolean meetsThresholds() {
    if (launchSolution == null) return false;

    Logger.recordOutput(
        "ShooterHandler/WithinThresholds/Flywheel (rps)",
        flywheelAngularVelocityAbsDiff().in(RotationsPerSecond));
    Logger.recordOutput(
        "ShooterHandler/WithinThresholds/Turret (deg)", turretRotationAbsDiff().in(Degrees));
    Logger.recordOutput(
        "ShooterHandler/WithinThresholds/Hood (deg)", hoodAngleAbsDiff().in(Degrees));

    switch (shooterState) {
      case NOT_READY:
        if (flywheelAngularVelocityAbsDiff().lt(config.AIMING_FLYWHEEL_THRESHOLD())
            && turretRotationAbsDiff().lt(config.AIMING_ROTATION_THRESHOLD())
            && hoodAngleAbsDiff().lt(config.AIMING_HOOD_ANGLE_THRESHOLD())) {
          return true;
        } else {
          return false;
        }

      case READY:
        if (flywheelAngularVelocityAbsDiff().gt(config.SHOOTING_FLYWHEEL_ABORT())
            && turretRotationAbsDiff().gt(config.SHOOTING_ROTATION_THRESHOLD())
            && hoodAngleAbsDiff().gt(config.SHOOTING_HOOD_ANGLE_THRESHOLD())) {
          return false;
        } else {
          return true;
        }

      default:
        return true;
    }
  }

  // --- Threshold helper functions ---

  private AngularVelocity flywheelAngularVelocityAbsDiff() {
    AngularVelocity diff = getFlywheelAngularVelocity().minus(flywheelVelocity());
    double absoluteOmega = Math.abs(diff.in(RadiansPerSecond));
    return Radians.per(Seconds).of(absoluteOmega);
  }

  private Angle turretRotationAbsDiff() {
    return Radians.of(
        launchSolution.turretRotation.getMeasure().minus(getTurretAbsRotation()).abs(Radians));
  }

  private Angle hoodAngleAbsDiff() {
    Angle diff = launchSolution.hoodAngle().minus(getHoodAngle());
    double absoluteOmega = diff.in(Radians);
    return Radians.of(absoluteOmega);
  }

  private AngularVelocity flywheelVelocity() {
    return robotContainer.superstructure.flywheel.getVelocity();
  }

  private Angle getTurretRotation() {
    return robotContainer.superstructure.turret.getPosition();
  }

  private Angle getTurretAbsRotation() {
    return robotContainer
        .superstructure
        .turret
        .getPosition()
        .plus(drivetrain.getState().Pose.getRotation().getMeasure());
  }

  private AngularVelocity getTurretOmega() {
    return robotContainer.superstructure.turret.getVelocity();
  }

  private Angle getHoodAngle() {
    return robotContainer.superstructure.hood.getPosition();
  }

  // --- Threshold helper functions ^^^ ---

  // Check if within hardware (+ other) constraints for shooting
  // determines if shot is even remotely possible
  public boolean satisfiesConstraints() {
    // --- HIGH PRIORITY CONSTRAINTS ---
    LinearVelocity speed = launchSolution.flywheelSpeed();
    if (speed.lt(config.MIN_FLYWHEEL_SPEED()) || speed.gt(config.MAX_FLYWHEEL_SPEED())) {
      // TODO: log errors
      return false;
    }

    Angle angle = launchSolution.hoodAngle();
    if (angle.lt(config.MIN_HOOD_ANGLE()) || angle.gt(config.MAX_HOOD_ANGLE())) {
      // TODO: log errors
      return false;
    }

    // --- LOW PRIORITY CONSTRAINTS ---

    Distance targetDistance =
        Meters.of(targetState.position().minus(projectileState.position()).getNorm());
    if (targetDistance.lt(config.MIN_SHOT_DISTANCE())
        || targetDistance.gt(config.MAX_SHOT_DISTANCE())) {
      // TODO: log errors
      return false;
    }

    // TODO: add more constraints to check against (such as time)

    return true;
  }

  private ObjectState getProjectileState() {
    // This code is hella chopped but works maybe??
    SwerveDriveState drivetrainState = drivetrain.getState();

    double radiusToBall = config.RADIUS_TO_BALL().in(Meters);
    double robotOmega = drivetrainState.Speeds.omegaRadiansPerSecond;

    double turretAngle = getTurretAbsRotation().in(Radians);
    double turretOmega = getTurretOmega().in(RadiansPerSecond);

    Translation2d turretCenterToBall = new Translation2d(radiusToBall, new Rotation2d(turretAngle));

    Translation2d robotCenterToTurret =
        config.TURRET_OFFSET().toTranslation2d().rotateBy(drivetrainState.Pose.getRotation());

    Vector<N3> W_robot = new Translation3d(0, 0, robotOmega).toVector();
    Vector<N3> W_turret = new Translation3d(0, 0, turretOmega).toVector();

    Vector<N3> v_robotRot =
        Vector.cross(W_robot, new Translation3d(robotCenterToTurret).toVector());
    Vector<N3> v_turRot = Vector.cross(W_turret, new Translation3d(turretCenterToBall).toVector());

    Translation3d projPoseOffset =
        new Translation3d(robotCenterToTurret)
            .plus(new Translation3d(turretCenterToBall))
            .plus(new Translation3d(0.0, 0.0, config.TURRET_OFFSET.getZ()));

    Translation3d projVelOffset = new Translation3d(v_robotRot.plus(v_turRot));

    ObjectState robotState = new ObjectState(drivetrainState);

    return robotState.plus(projPoseOffset, projVelOffset);
  }

  // calculate stationary shot (using stationaryInterpolation)
  private LaunchSolution calcStaticShot(ObjectState projTrue, ObjectState target) {
    return physics.stationaryInterpolation(projTrue, target);
  }

  // --- LOGGING VALUES ---

  // Log launchSolutions that are calculated
  private LinearVelocity launchFlywheelSpeedMps() {
    return launchSolution == null ? null : launchSolution.flywheelSpeed();
  }

  private Angle launchHoodAngle() {
    return launchSolution == null ? null : launchSolution.hoodAngle();
  }

  private Angle launchTurretRotation() {
    return launchSolution == null ? null : Radians.of(launchSolution.turretRotation.getRadians());
  }

  // Log Objectstates
  @AutoLogOutput
  private Pose3d projectilePose() {
    return projectileState == null
        ? null
        : new Pose3d(projectileState.position(), new Rotation3d());
  }

  @AutoLogOutput
  private Translation3d projectileVelocity() {
    return projectileState == null ? null : projectileState.velocity();
  }

  @AutoLogOutput
  private Pose3d targetPose() {
    return targetState == null ? null : new Pose3d(targetState.position(), new Rotation3d());
  }

  @AutoLogOutput
  private Translation3d targetVelocity() {
    return targetState == null ? null : targetState.velocity();
  }

  public LaunchSolution getLaunchSolution() {
    return launchSolution;
  }

  public boolean ready() {
    return shooterState == State.READY;
  }
}

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
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
import frc.robot.lib.shooter.ShooterConfig;
import frc.robot.lib.superstructure.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure.Setpoint.Side;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterHandler {
  public static final double X_DISTANCE_FROM_CENTER = 3.6448975;
  public static final double HEIGHT = 1.4370877;

  public static final ObjectState TARGET_BLUE =
      new ObjectState(
          new Translation3d(
              (FlippingUtil.fieldSizeX / 2) - X_DISTANCE_FROM_CENTER,
              (FlippingUtil.fieldSizeY / 2),
              HEIGHT),
          new Translation3d());

  public static final ObjectState TARGET_RED =
      new ObjectState(
          new Translation3d(
              (FlippingUtil.fieldSizeX / 2) + X_DISTANCE_FROM_CENTER,
              (FlippingUtil.fieldSizeY / 2),
              HEIGHT),
          new Translation3d());

  // config + physics model
  private ShooterConfig config;
  private ShooterPhysics physics;
  private ObjectState projectileState;
  private ObjectState targetState;
  private final String name;

  // state machine
  public enum State {
    NOT_READY,
    AIMING,
    FIRING
  }

  public enum Goal {
    NONE,
    SHOOT
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
  private final MotorSubsystem indexer;

  private final Side side;

  public ShooterHandler(
      AngularSubsystem turret,
      AngularSubsystem hood,
      AngularSubsystem flywheel,
      MotorSubsystem indexer,
      CommandSwerveDrivetrain drivetrain,
      ShooterConfig config,
      Side side) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.flywheel = flywheel;
    this.hood = hood;
    this.indexer = indexer;
    this.config = config;
    this.physics = new ShooterPhysics(this.config.PHYSICS());
    this.name = config.name() + "/ShooterHandler";

    this.shooterState = State.NOT_READY;
    this.shooterGoal = Goal.NONE;
    this.side = side;
    this.targetState = TARGET_BLUE;
  }

  public void periodic() {
    targetState =
        DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)
            ? TARGET_BLUE
            : TARGET_RED;
    projectileState = getProjectileState();

    // calculate physics
    launchSolution = physics.iterativeTimeSolve(projectileState, targetState, 20);

    logStates();

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

    // set output
    if (shooterState != State.NOT_READY) {
      flywheel.setVelocity(getFlywheelAngularVelocity());
      hood.setPosition(launchSolution.hoodAngle());
      turret.setPosition(getRelativeTurretAngle());
    }

    if (shooterState == State.FIRING) {
      indexer.setVoltage(SetpointGoal.INDEX.getSetpoint().getSide(side).get().getIndexer().get());
    } else {
      indexer.setVoltage(SetpointGoal.NEUTRAL.getSetpoint().getSide(side).get().getIndexer().get());
    }
  }

  public AngularVelocity getFlywheelAngularVelocity() {
    double omega =
        launchSolution.linearFlywheelVelocity().in(MetersPerSecond)
            / config.PHYSICAL_CONVERSION().FLYWHEEL_RADIUS().in(Meters);

    return RadiansPerSecond.of(omega);
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
          name + "/LaunchGoals/Flywheel (mps)",
          launchSolution.linearFlywheelVelocity().in(MetersPerSecond));
      Logger.recordOutput(
          name + "/LaunchGoals/Turret (deg)",
          Radians.of(launchSolution.turretRotation.getRadians()).in(Degrees));
      Logger.recordOutput(
          name + "/LaunchGoals/Turret Rel (deg)", getRelativeTurretAngle().in(Degrees));
      Logger.recordOutput(name + "/LaunchGoals/Hood (deg)", launchSolution.hoodAngle().in(Degrees));
      Translation2d distance2d = targetState.minus(projectileState).xyPosition();
      Logger.recordOutput(name + "/Distance/2D", distance2d);
      Logger.recordOutput(name + "/Distance/1D", distance2d.getNorm());

      Logger.recordOutput(
          name + "/Error/Flywheel (rps)", flywheelAngularVelocityAbsDiff().in(RotationsPerSecond));
      Logger.recordOutput(name + "/Error/Turret (deg)", turretRotationAbsDiff().in(Degrees));
      Logger.recordOutput(name + "/Error/Hood (physics deg)", hoodAngleAbsDiff().in(Degrees));
    }

    if (projectileState != null) {
      Logger.recordOutput(name + "/Projectile Position", projectileState.position());
      Logger.recordOutput(name + "/Projectile Velocity", projectileState.velocity());
      Logger.recordOutput(name + "/Target Position", targetState.position());
      Logger.recordOutput(
          name + "/Target Velocity", new Pose3d(targetState.velocity(), new Rotation3d()));
    }
  }

  public void setPhysics(ShooterConfig.Physics shooterConfig) {
    this.physics = new ShooterPhysics(shooterConfig);
  }

  @AutoLogOutput(key = "{name}/canTransitionToReady")
  private boolean canTransitionToReady() {
    if (launchSolution == null) {
      return false;
    }

    return flywheelAngularVelocityAbsDiff().lt(config.THRESHOLD().AIMING_FLYWHEEL_THRESHOLD())
        && turretRotationAbsDiff().lt(config.THRESHOLD().AIMING_ROTATION_THRESHOLD())
        && hoodAngleAbsDiff().lt(config.THRESHOLD().AIMING_HOOD_ANGLE_THRESHOLD());
  }

  @AutoLogOutput(key = "{name}/canTransitionToNotReady")
  private boolean canTransitionToNotReady() {
    if (launchSolution == null) {
      return true;
    }

    return flywheelAngularVelocityAbsDiff().gt(config.THRESHOLD().SHOOTING_FLYWHEEL_ABORT())
        || turretRotationAbsDiff().gt(config.THRESHOLD().SHOOTING_ROTATION_THRESHOLD())
        || hoodAngleAbsDiff().gt(config.THRESHOLD().SHOOTING_HOOD_ANGLE_THRESHOLD());
  }

  // --- Threshold helper functions ---
  private AngularVelocity flywheelAngularVelocityAbsDiff() {
    return getFlywheelAngularVelocity().minus(flywheel.getVelocity());
  }

  private Angle turretRotationAbsDiff() {
    return Radians.of(
        Math.abs(
            MathUtil.angleModulus(
                launchSolution.turretRotation.getMeasure().in(Radians)
                    - MathUtil.angleModulus(getTurretAbsRotation().in(Radians)))));
  }

  private Angle hoodAngleAbsDiff() {
    return launchSolution.hoodAngle().minus(hood.getPosition());
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
    LinearVelocity speed = launchSolution.linearFlywheelVelocity();
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

    Vector<N3> W_robot = new Translation3d(0, 0, robotOmega).toVector();
    Vector<N3> W_turret = new Translation3d(0, 0, turretOmega).toVector();

    Vector<N3> v_robotRot =
        Vector.cross(W_robot, new Translation3d(robotCenterToTurret).toVector());
    Vector<N3> v_turRot = Vector.cross(W_turret, new Translation3d(turretCenterToBall).toVector());

    Translation3d projPoseOffset =
        new Translation3d(robotCenterToTurret)
            .plus(new Translation3d(turretCenterToBall))
            .plus(
                new Translation3d(
                    0.0, 0.0, config.PHYSICAL_CONVERSION().TURRET_XY_OFFSET().getZ()));

    Translation3d projVelOffset = new Translation3d(v_robotRot.plus(v_turRot));

    ObjectState robotState = new ObjectState(drivetrainState);

    return robotState.plus(projPoseOffset, projVelOffset);
  }
}

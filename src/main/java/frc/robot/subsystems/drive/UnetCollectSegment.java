package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** A timed, robot-relative COS collection sweep; never used as an auto's first segment. */
class UnetCollectSegment implements Segment {
  static final double UNET_SWEEP_SECONDS = 3.0;
  private static final long GOAL_TIMEOUT_MICROS = 250_000;
  private static final double MAX_OMEGA = 0.8 * 2.0 * Math.PI;
  private final Subsystem drivetrain;
  private final Consumer<SwerveRequest> output;
  private final NetworkTableInstance instance;
  private final Supplier<Command> intakeDown;
  private final SwerveRequest.RobotCentric drive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private DoubleArraySubscriber subscriber;
  private TimestampedDoubleArray latest;
  private long startedAt;

  UnetCollectSegment(CommandSwerveDrivetrain drivetrain, Supplier<Command> intakeDown) {
    this(drivetrain, drivetrain::setRequest, NetworkTableInstance.getDefault(), intakeDown);
  }

  UnetCollectSegment(
      Subsystem drivetrain,
      Consumer<SwerveRequest> output,
      NetworkTableInstance instance,
      Supplier<Command> intakeDown) {
    this.drivetrain = drivetrain;
    this.output = output;
    this.instance = instance;
    this.intakeDown = intakeDown;
  }

  public Command getCommand(FollowPath.Builder builder) {
    return Commands.sequence(
        intakeDown.get(),
        new FunctionalCommand(
                this::initialize, this::execute, interrupted -> stop(), () -> false, drivetrain)
            .withTimeout(UNET_SWEEP_SECONDS));
  }

  private void initialize() {
    latest = null;
    startedAt = NetworkTablesJNI.now();
    subscriber =
        instance
            .getTable("COS")
            .getDoubleArrayTopic("BallVelocityGoal")
            .subscribe(
                new double[0],
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(20));
    output.accept(brake);
    Logger.recordOutput("Auto/UNetCollect/Active", true);
  }

  private void execute() {
    for (var update : subscriber.readQueue()) {
      latest = update;
    }
    long now = NetworkTablesJNI.now();
    boolean fresh =
        latest != null
            && latest.timestamp >= startedAt
            && now - latest.timestamp >= 0
            && now - latest.timestamp <= GOAL_TIMEOUT_MICROS;
    var goal =
        BallPathingGoal.fromNetworkTables(
            latest == null ? new double[0] : latest.value,
            Drive.MAX_SPEED_METERS_PER_SECOND,
            MAX_OMEGA);
    Logger.recordOutput("Auto/UNetCollect/Fresh", fresh);
    Logger.recordOutput("Auto/UNetCollect/Valid", goal.isPresent());
    Logger.recordOutput("Auto/UNetCollect/ElapsedSeconds", (now - startedAt) / 1e6);
    if (fresh && goal.isPresent()) {
      var value = goal.orElseThrow();
      output.accept(
          drive
              .withVelocityX(value.velocityX())
              .withVelocityY(value.velocityY())
              .withRotationalRate(value.omegaRadiansPerSecond()));
      logVelocity(value.velocityX(), value.velocityY(), value.omegaRadiansPerSecond());
      Logger.recordOutput("Auto/UNetCollect/TargetX", value.targetX());
      Logger.recordOutput("Auto/UNetCollect/TargetY", value.targetY());
    } else {
      output.accept(brake);
      logVelocity(0, 0, 0);
    }
  }

  private void logVelocity(double x, double y, double omega) {
    Logger.recordOutput("Auto/UNetCollect/VelocityX", x);
    Logger.recordOutput("Auto/UNetCollect/VelocityY", y);
    Logger.recordOutput("Auto/UNetCollect/OmegaRadiansPerSecond", omega);
  }

  private void stop() {
    output.accept(brake);
    logVelocity(0, 0, 0);
    Logger.recordOutput("Auto/UNetCollect/Active", false);
    if (subscriber != null) subscriber.close();
    subscriber = null;
    latest = null;
  }

  public Pose2d getStartPose(boolean mirrored) {
    return null;
  }

  public Path copy() {
    return new Path();
  }
}

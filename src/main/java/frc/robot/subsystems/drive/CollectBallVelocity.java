package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Follows COS velocity updates until interrupted or timed out by the collection segment. */
class CollectBallVelocity extends Command {
  private static final double GOAL_TIMEOUT_SECONDS = 0.25;
  private static final double MAX_ANGULAR_RATE = 0.8 * 2.0 * Math.PI;
  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<DoubleArraySubscriber> subscribe;
  private final DoubleSupplier clock;
  private final SwerveRequest.RobotCentric request =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private DoubleArraySubscriber subscriber;
  private double[] goal;
  private double receivedAt;

  CollectBallVelocity(CommandSwerveDrivetrain drivetrain) {
    this(
        drivetrain,
        () ->
            NetworkTableInstance.getDefault()
                .getTable("COS")
                .getDoubleArrayTopic("BallVelocityGoal")
                .subscribe(
                    new double[0],
                    PubSubOption.keepDuplicates(true),
                    PubSubOption.sendAll(true),
                    PubSubOption.pollStorage(20)),
        () -> WPIUtilJNI.now() / 1_000_000.0);
  }

  CollectBallVelocity(
      CommandSwerveDrivetrain drivetrain,
      Supplier<DoubleArraySubscriber> subscribe,
      DoubleSupplier clock) {
    this.drivetrain = drivetrain;
    this.subscribe = subscribe;
    this.clock = clock;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    subscriber = subscribe.get();
    goal = new double[0];
    receivedAt = Double.NEGATIVE_INFINITY;
    drivetrain.setRequest(brake);
  }

  @Override
  public void execute() {
    for (var update : subscriber.readQueue()) {
      goal = update.value;
      // NT timestamps use the local WPI clock. Keep queued or retained old goals stale.
      receivedAt = update.timestamp / 1_000_000.0;
    }
    var parsed =
        BallPathingGoal.fromNetworkTables(
            goal, Drive.MAX_SPEED_METERS_PER_SECOND, MAX_ANGULAR_RATE);
    double age = clock.getAsDouble() - receivedAt;
    boolean fresh = age >= 0.0 && age <= GOAL_TIMEOUT_SECONDS;
    Logger.recordOutput("Drive/AutoBallCollection/Fresh", fresh);
    Logger.recordOutput("Drive/AutoBallCollection/Valid", parsed.isPresent());
    if (fresh && parsed.isPresent()) {
      BallPathingGoal command = parsed.orElseThrow();
      drivetrain.setRequest(
          request
              .withVelocityX(command.velocityX())
              .withVelocityY(command.velocityY())
              .withRotationalRate(command.omegaRadiansPerSecond()));
    } else {
      drivetrain.setRequest(brake);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setRequest(brake);
    subscriber.close();
  }
}

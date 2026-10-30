package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CollectBallVelocityTest {
  private CommandSwerveDrivetrain drivetrain;
  private DoubleArraySubscriber subscriber;
  private CollectBallVelocity command;
  private final AtomicReference<Double> now = new AtomicReference<>(10.0);
  private final AtomicReference<SwerveRequest> lastRequest = new AtomicReference<>();

  @BeforeEach
  void setUp() {
    drivetrain = mock(CommandSwerveDrivetrain.class);
    subscriber = mock(DoubleArraySubscriber.class);
    when(subscriber.readQueue()).thenReturn(new TimestampedDoubleArray[0]);
    doAnswer(
            invocation -> {
              lastRequest.set(invocation.getArgument(0));
              return null;
            })
        .when(drivetrain)
        .setRequest(any());
    command = new CollectBallVelocity(drivetrain, () -> subscriber, now::get);
    command.initialize();
  }

  private double[] goal(double vx, double vy, double omega) {
    return new double[] {vx, vy, 10.0, 5.0, 3.0, 1.0, 0.0, 12.0, 0.2, omega};
  }

  private void publish(double[] goal) {
    when(subscriber.readQueue())
        .thenReturn(
            new TimestampedDoubleArray[] {
              new TimestampedDoubleArray((long) (now.get() * 1_000_000), 0, goal)
            },
            new TimestampedDoubleArray[0]);
    command.execute();
  }

  @Test
  void followsChangingRobotRelativeGoalsWithoutDriverInput() {
    assertTrue(command.getRequirements().contains(drivetrain));
    publish(goal(1.2, -0.4, 0.7));
    var drive = assertInstanceOf(SwerveRequest.RobotCentric.class, lastRequest.get());
    assertEquals(1.2, drive.VelocityX);
    assertEquals(-0.4, drive.VelocityY);
    assertEquals(0.7, drive.RotationalRate);

    publish(goal(-0.5, 0.8, -0.3));
    drive = assertInstanceOf(SwerveRequest.RobotCentric.class, lastRequest.get());
    assertEquals(-0.5, drive.VelocityX);
    assertEquals(0.8, drive.VelocityY);
    assertEquals(-0.3, drive.RotationalRate);
  }

  @Test
  void brakesWhileWaitingAndWhenGoalsExpireThenResumesOnFreshGoal() {
    command.execute();
    assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, lastRequest.get());
    publish(goal(1.0, 0.0, 0.0));
    now.set(10.24);
    command.execute();
    assertInstanceOf(SwerveRequest.RobotCentric.class, lastRequest.get());
    now.set(10.26);
    command.execute();
    assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, lastRequest.get());
    publish(goal(0.0, 1.0, 0.0));
    assertInstanceOf(SwerveRequest.RobotCentric.class, lastRequest.get());
  }

  @Test
  void invalidGoalImmediatelyReplacesValidGoalWithBrake() {
    publish(goal(1.0, 0.0, 0.0));
    double[] invalid = goal(1.0, 0.0, 0.0);
    invalid[5] = 0.0;
    publish(invalid);
    assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, lastRequest.get());
    publish(goal(Double.NaN, 0.0, 0.0));
    assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, lastRequest.get());
    publish(new double[0]);
    assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, lastRequest.get());
  }

  @Test
  void rejectsOldQueuedOrRetainedGoals() {
    when(subscriber.readQueue())
        .thenReturn(
            new TimestampedDoubleArray[] {
              new TimestampedDoubleArray(9_000_000, 0, goal(1.0, 0.0, 0.0))
            });
    command.execute();
    assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, lastRequest.get());
  }

  @Test
  void limitsTranslationAndRotation() {
    publish(goal(6.0, 8.0, -20.0));
    var drive = assertInstanceOf(SwerveRequest.RobotCentric.class, lastRequest.get());
    assertEquals(
        Drive.MAX_SPEED_METERS_PER_SECOND, Math.hypot(drive.VelocityX, drive.VelocityY), 1e-12);
    assertEquals(-0.8 * 2.0 * Math.PI, drive.RotationalRate, 1e-12);
  }

  @Test
  void stopsAndClosesSubscriberOnCompletionOrCancellationAndResetsOnRestart() {
    for (boolean interrupted : new boolean[] {false, true}) {
      publish(goal(1.0, 0.0, 0.0));
      command.end(interrupted);
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, lastRequest.get());
      command.initialize();
      command.execute();
      assertInstanceOf(SwerveRequest.SwerveDriveBrake.class, lastRequest.get());
    }
    verify(subscriber, times(2)).close();
  }
}

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/** Collects using live COS robot-relative velocities rather than a generated waypoint path. */
public class DynamicSegmentCollect implements Segment {
  // Bound collection so the next hub segment can run even while more fuel is visible.
  private static final double COLLECTION_SECONDS = 5.0;
  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Command> intakeDown;

  public DynamicSegmentCollect(CommandSwerveDrivetrain drivetrain, Supplier<Command> intakeDown) {
    this.drivetrain = drivetrain;
    this.intakeDown = intakeDown;
  }

  @Override
  public Command getCommand(FollowPath.Builder builder) {
    return intakeDown
        .get()
        .andThen(new CollectBallVelocity(drivetrain).withTimeout(COLLECTION_SECONDS));
  }

  @Override
  public Pose2d getStartPose(boolean mirrored) {
    return null;
  }

  @Override
  public Path copy() {
    return new Path();
  }
}

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;

/** Publishes each robot-periodic pose sample, including unchanged poses. */
public class DrivePosePublisher implements AutoCloseable {
  private final NetworkTableInstance instance;
  private final StructPublisher<Pose2d> publisher;

  public DrivePosePublisher(NetworkTableInstance instance) {
    this.instance = instance;
    publisher =
        instance
            .getStructTopic("/Drive/Pose", Pose2d.struct)
            .publish(
                PubSubOption.periodic(0.02),
                PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(true));
  }

  public void publish(Pose2d pose) {
    publisher.set(pose);
    // Wake the NT server to process this value and bypass its default 100 ms batching timer.
    instance.flush();
  }

  @Override
  public void close() {
    publisher.close();
  }
}

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class COS {
  private final CommandSwerveDrivetrain drivetrain;
  private Pose2d lastVisionPose = new Pose2d();
  IntegerPublisher numTagsPerControlLoopPublisher;
  DoubleArraySubscriber tagEstimationSubscribers;

  public COS(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable("COS");
    double[] blank = {-1};

    tagEstimationSubscribers =
        table
            .getDoubleArrayTopic("PositionEstimate")
            .subscribe(
                blank,
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(200));

    numTagsPerControlLoopPublisher = table.getIntegerTopic("NumTagsPerControlLoop").publish();
  }

  public void updatePose() {
    double[][] tagEstimations = tagEstimationSubscribers.readQueueValues();
    if (tagEstimations.length == 0) {
      return;
    }
    numTagsPerControlLoopPublisher.set(tagEstimations.length);

    for (int i = 0; i < tagEstimations.length; i++) {
      // 0 x
      // 1 y
      // 2 z
      // 3 variance
      // 4 timestamp
      if (tagEstimations[i][0] == -1) {
        continue;
      }

      Pose2d estimate =
          new Pose2d(
              tagEstimations[i][0], tagEstimations[i][1], new Rotation2d(tagEstimations[i][2]));

      drivetrain.addVisionMeasurement(
          estimate,
          tagEstimations[i][4],
          VecBuilder.fill(
              tagEstimations[i][3] / 4.0,
              tagEstimations[i][3] / 4.0,
              tagEstimations[i][3] * 2.0 / 3.0));
      lastVisionPose = estimate;
    }
  }

  public Pose2d getLastVisionPose() {
    return lastVisionPose;
  }
}

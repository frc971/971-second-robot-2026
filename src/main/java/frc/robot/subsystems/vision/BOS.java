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

public class BOS {
  private final CommandSwerveDrivetrain drivetrain;
  public static final String[] CAMERA_NAMES = {
    "PoseEstimate/Right/", "PoseEstimate/Left/", "PoseEstimate/Front/"
  };
  IntegerPublisher num_tags_per_control_loop_publisher;

  DoubleArraySubscriber[] tag_estimation_subscribers =
      new DoubleArraySubscriber[CAMERA_NAMES.length];

  Pose2d lastVisionPose = new Pose2d(0, 0, new Rotation2d());

  public BOS(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable("Orin");

    double[] blank = {-1};

    for (int i = 0; i < CAMERA_NAMES.length; i++) {
      tag_estimation_subscribers[i] =
          table
              .getDoubleArrayTopic(CAMERA_NAMES[i] + "TagEstimation")
              .subscribe(
                  blank,
                  PubSubOption.keepDuplicates(true),
                  PubSubOption.sendAll(true),
                  PubSubOption.pollStorage(200));
    }

    num_tags_per_control_loop_publisher = table.getIntegerTopic("NumTagsPerControlLoop").publish();
  }

  public void updatePose() {
    for (DoubleArraySubscriber topic : tag_estimation_subscribers) {
      double[][] tagEstimations = topic.readQueueValues();
      if (tagEstimations.length == 0) {
        continue;
      }
      num_tags_per_control_loop_publisher.set(tagEstimations.length);

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
  }

  public Pose2d getLastVisionPose() {
    return lastVisionPose;
  }
}

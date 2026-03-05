package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class BOS {
  private final CommandSwerveDrivetrain drivetrain;

  DoubleArraySubscriber[] topics = new DoubleArraySubscriber[3];

  public BOS(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable("Orin");

    double[] blank = {-1, -1, -1, -1, -1};

    topics[0] =
        table
            .getDoubleArrayTopic("PoseEstimate/Front/TagEstimation")
            .subscribe(blank, PubSubOption.keepDuplicates(true));
    topics[1] =
        table
            .getDoubleArrayTopic("PoseEstimate/Right/TagEstimation")
            .subscribe(blank, PubSubOption.keepDuplicates(true));
    topics[2] =
        table
            .getDoubleArrayTopic("PoseEstimate/Left/TagEstimation")
            .subscribe(blank, PubSubOption.keepDuplicates(true));
  }

  public void updatePose() {
    for (DoubleArraySubscriber topic : topics) {
      double[][] tagEstimations = topic.readQueueValues();

      for (int i = 0; i < tagEstimations.length; i++) {
        // 0 x
        // 1 y
        // 2 z
        // 3 variance
        // 4 timestamp
        if (tagEstimations[i][0] == -1) {
          continue;
        }
        drivetrain.addVisionMeasurement(
            new Pose2d(
                tagEstimations[i][0], tagEstimations[i][1], new Rotation2d(tagEstimations[i][2])),
            tagEstimations[i][4],
            VecBuilder.fill(
                tagEstimations[i][3] / 3.0,
                tagEstimations[i][3] / 3.0,
                tagEstimations[i][3] * 2.0 / 3.0));
      }
    }
  }
}

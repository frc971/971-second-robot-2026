package frc.robot.tools;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.concurrent.TimeUnit;

/** Publishes a fixed fake cluster position for testing dynamic pathing. */
public final class FakeClusterPublisher {
  private static final boolean ENABLED = true;
  private static final double CLUSTER_X = 12.8;
  private static final double CLUSTER_Y = 4.035;

  public static void main(String[] args) throws InterruptedException {
    if (!ENABLED) return;

    var instance = NetworkTableInstance.create();
    instance.startClient4("FakeClusterPublisher");
    instance.setServer(args.length > 0 ? args[0] : "localhost");
    var table = instance.getTable("DriveState");
    try (DoublePublisher x = table.getDoubleTopic("ClusterPositionsX").publish();
        DoublePublisher y = table.getDoubleTopic("ClusterPositionsY").publish()) {
      while (!Thread.currentThread().isInterrupted()) {
        x.set(CLUSTER_X);
        y.set(CLUSTER_Y);
        TimeUnit.MILLISECONDS.sleep(20);
      }
    } finally {
      instance.stopClient();
      instance.close();
    }
  }

  private FakeClusterPublisher() {}
}

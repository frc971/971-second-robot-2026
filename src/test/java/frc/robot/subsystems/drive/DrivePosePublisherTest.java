package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedObject;
import java.net.ServerSocket;
import java.util.ArrayList;
import java.util.concurrent.locks.LockSupport;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class DrivePosePublisherTest {
  @ParameterizedTest
  @ValueSource(doubles = {0.02, 0.1, 1.0})
  void flushDeliversEachSampleBefore100msEvenWithSlowSubscriberPeriod(double period)
      throws Exception {
    int port;
    try (var socket = new ServerSocket(0)) {
      port = socket.getLocalPort();
    }
    try (var server = NetworkTableInstance.create();
        var client = NetworkTableInstance.create();
        var publisher = new DrivePosePublisher(server);
        var subscriber =
            client
                .getStructTopic("/Drive/Pose", Pose2d.struct)
                .subscribe(
                    new Pose2d(-2, 0, Rotation2d.kZero),
                    PubSubOption.periodic(period),
                    PubSubOption.sendAll(true),
                    PubSubOption.keepDuplicates(true),
                    PubSubOption.pollStorage(256))) {
      server.startServer("", "127.0.0.1", 0, port);
      client.setServer("127.0.0.1", port);
      client.startClient4("pose-rate-test");
      long deadline = System.nanoTime() + 5_000_000_000L;
      while (subscriber.get().getX() != -1 && System.nanoTime() < deadline) {
        publisher.publish(new Pose2d(-1, 0, Rotation2d.kZero));
        Thread.sleep(20);
      }
      assertEquals(-1, subscriber.get().getX(), "Remote subscription must be ready");
      subscriber.readQueue();

      var samples = new ArrayList<TimestampedObject<Pose2d>>();
      long start = System.nanoTime() + 20_000_000L;
      long maxLatency = 0;
      for (int i = 0; i < 50; i++) {
        long scheduled = start + i * 20_000_000L;
        while (System.nanoTime() < scheduled) {
          LockSupport.parkNanos(scheduled - System.nanoTime());
        }
        long sent = System.nanoTime();
        publisher.publish(new Pose2d(Math.min(i, 24), 0, Rotation2d.kZero));
        while (samples.size() <= i && System.nanoTime() - sent < 100_000_000L) {
          for (var sample : subscriber.readQueue()) {
            if (sample.value.getX() >= 0) samples.add(sample);
          }
          if (samples.size() <= i) Thread.sleep(1);
        }
        long latency = System.nanoTime() - sent;
        maxLatency = Math.max(maxLatency, latency);
        assertEquals(i + 1, samples.size(), "Sample must arrive without waiting for batching");
        assertTrue(latency < 100_000_000L, "Delivery exceeded 100 ms: " + latency / 1e6);
      }
      System.out.printf(
          "Subscriber period %.2f s: max delivery %.3f ms%n", period, maxLatency / 1e6);
      assertEquals(50, samples.size());
      assertEquals(50, samples.stream().map(sample -> sample.timestamp).distinct().count());
      for (int i = 0; i < 50; i++) assertEquals(Math.min(i, 24), samples.get(i).value.getX());
      assertEquals(980_000, samples.get(49).timestamp - samples.get(0).timestamp, 150_000);
    }
  }
}

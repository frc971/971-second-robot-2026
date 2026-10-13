package frc.robot.lib.can;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Timer;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class CanBusScheduler implements AutoCloseable {
  private final String busName;
  private final ExecutorService executor;
  private final Map<String, AtomicLong> latestGenerations = new ConcurrentHashMap<>();
  private final AtomicLong rejectedStaleOperations = new AtomicLong();
  private final AtomicLong failedOperations = new AtomicLong();
  private final AtomicLong activeOperations = new AtomicLong();

  CanBusScheduler(String busName, int parallelism) {
    this.busName = busName;
    executor = Executors.newFixedThreadPool(parallelism);
  }

  public void submit(String operationName, Supplier<StatusCode> operation) {
    AtomicLong generation =
        latestGenerations.computeIfAbsent(operationName, ignored -> new AtomicLong());
    long requestGeneration = generation.incrementAndGet();

    executor.submit(
        () -> {
          if (generation.get() != requestGeneration) {
            rejectedStaleOperations.incrementAndGet();
            recordBusMetrics();
            return;
          }

          double startedAt = Timer.getFPGATimestamp();
          activeOperations.incrementAndGet();
          StatusCode status = operation.get();

          if (status == null || !status.isOK()) {
            failedOperations.incrementAndGet();
          }

          activeOperations.decrementAndGet();
          Logger.recordOutput(
              "CAN/" + busName + "/LastOperationDurationMs",
              (Timer.getFPGATimestamp() - startedAt) * 1000.0);

          recordBusMetrics();
        });
  }

  private void recordBusMetrics() {
    String prefix = "CAN/" + busName;
    Logger.recordOutput(prefix + "/ActiveOperations", activeOperations.get());
    Logger.recordOutput(prefix + "/RejectedStaleOperations", rejectedStaleOperations.get());
    Logger.recordOutput(prefix + "/FailedOperations", failedOperations.get());
  }

  @Override
  public void close() {
    executor.shutdownNow();
  }
}

package frc.robot.lib;

import com.ctre.phoenix6.StatusCode;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class CanConfigExecutor {
  private static final ExecutorService executor = Executors.newSingleThreadExecutor();
  private static final Map<String, Request> pending = new ConcurrentHashMap<>();
  private static final Map<String, AtomicLong> submittedCounts = new ConcurrentHashMap<>();
  private static final Map<String, AtomicLong> supersededCounts = new ConcurrentHashMap<>();
  private static final Map<String, AtomicLong> appliedCounts = new ConcurrentHashMap<>();
  private static final AtomicBoolean workerRunning = new AtomicBoolean();

  private CanConfigExecutor() {}

  public static void submit(String name, Supplier<StatusCode> apply) {
    submittedCounts.computeIfAbsent(name, ignored -> new AtomicLong()).incrementAndGet();
    if (pending.put(name, new Request(name, System.nanoTime(), apply)) != null) {
      supersededCounts.computeIfAbsent(name, ignored -> new AtomicLong()).incrementAndGet();
    }
    startWorker();
  }

  private static void startWorker() {
    if (workerRunning.compareAndSet(false, true)) {
      executor.submit(
          () -> {
            try {
              Request request;
              while ((request = takeNext()) != null) {
                long started = System.nanoTime();
                StatusCode status;
                try {
                  status = request.apply.get();
                  appliedCounts.computeIfAbsent(request.name, ignored -> new AtomicLong()).incrementAndGet();
                } catch (RuntimeException exception) {
                  status = null;
                }
                long finished = System.nanoTime();
                log(request, started, finished, status);
              }
            } finally {
              workerRunning.set(false);
              if (!pending.isEmpty()) startWorker();
            }
          });
    }
  }

  private static Request takeNext() {
    for (Map.Entry<String, Request> entry : pending.entrySet()) {
      if (pending.remove(entry.getKey(), entry.getValue())) return entry.getValue();
    }
    return null;
  }

  private static void log(Request request, long started, long finished, StatusCode status) {
    String key = "Config/CurrentLimit/" + request.name;
    Logger.recordOutput(key + "/QueueMs", (started - request.queued) / 1e6);
    Logger.recordOutput(key + "/ApplyMs", (finished - started) / 1e6);
    Logger.recordOutput(key + "/StatusCode", status == null ? "Exception" : status.toString());
    Logger.recordOutput(key + "/SubmittedCount", submittedCounts.get(request.name).get());
    Logger.recordOutput(key + "/SupersededCount", supersededCounts.getOrDefault(request.name, new AtomicLong()).get());
    Logger.recordOutput(key + "/AppliedCount", appliedCounts.getOrDefault(request.name, new AtomicLong()).get());
  }

  private static final class Request {
    final String name;
    final long queued;
    final Supplier<StatusCode> apply;

    Request(String name, long queued, Supplier<StatusCode> apply) {
      this.name = name;
      this.queued = queued;
      this.apply = apply;
    }
  }
}

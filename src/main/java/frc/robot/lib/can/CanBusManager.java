package frc.robot.lib.can;

import com.ctre.phoenix6.StatusCode;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.function.Supplier;

public final class CanBusManager implements AutoCloseable {
  private static final CanBusManager INSTANCE = new CanBusManager(2);
  private final Map<String, CanBusScheduler> schedulers = new HashMap<>();
  private final int operationsPerBus;

  private CanBusManager(int operationsPerBus) {
    if (operationsPerBus < 1) {
      throw new IllegalArgumentException("operationsPerBus must be at least one");
    }
    this.operationsPerBus = operationsPerBus;
  }

  public static CanBusManager getInstance() {
    return INSTANCE;
  }

  public synchronized CanBusScheduler getScheduler(String busName) {
    String normalizedBusName = normalize(busName);
    return schedulers.computeIfAbsent(
        normalizedBusName, name -> new CanBusScheduler(name, operationsPerBus));
  }

  public void submit(String busName, String operationName, Supplier<StatusCode> operation) {
    getScheduler(busName).submit(operationName, operation);
  }

  @Override
  public synchronized void close() {
    schedulers.values().forEach(CanBusScheduler::close);
    schedulers.clear();
  }

  private static String normalize(String busName) {
    if (busName == null || busName.isBlank()) {
      throw new IllegalArgumentException("CAN bus name must not be blank");
    }
    return busName.trim().toLowerCase(Locale.ROOT);
  }
}

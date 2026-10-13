package frc.robot.lib.can;

import com.ctre.phoenix6.StatusCode;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public final class CanBusManager implements AutoCloseable {
  private static final int OPERATIONS_PER_BUS = 2;
  private static final CanBusManager INSTANCE = new CanBusManager();
  private final Map<String, CanBusScheduler> schedulers = new HashMap<>();

  private CanBusManager() {}

  public static CanBusManager getInstance() {
    return INSTANCE;
  }

  private synchronized CanBusScheduler getScheduler(String busName) {
    if (busName == null || busName.isBlank()) {
      throw new IllegalArgumentException("CAN bus name must not be blank");
    }
    return schedulers.computeIfAbsent(busName, name -> new CanBusScheduler(name, OPERATIONS_PER_BUS));
  }

  public void submit(String busName, String operationName, Supplier<StatusCode> operation) {
    getScheduler(busName).submit(operationName, operation);
  }

  @Override
  public synchronized void close() {
    schedulers.values().forEach(CanBusScheduler::close);
    schedulers.clear();
  }
}

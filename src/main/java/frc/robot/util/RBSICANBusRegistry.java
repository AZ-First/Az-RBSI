package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/** Centralized CAN bus singleton registry + SIM/REAL indirection. */
public final class RBSICANBusRegistry {
  private static final Map<String, CANBus> realBuses = new ConcurrentHashMap<>();
  private static final Map<String, CANBusLike> likeBuses = new ConcurrentHashMap<>();
  private static volatile boolean initialized = false;
  private static volatile boolean sim = false;

  public static void initReal(String... busNames) {
    sim = false;
    for (String name : busNames) {
      CANBus bus = realBuses.computeIfAbsent(name, CANBus::new);
      likeBuses.computeIfAbsent(name, n -> new RealCANBusAdapter(bus));
    }
    initialized = true;
  }

  public static void initSim(String... busNames) {
    sim = true;
    for (String name : busNames) {
      likeBuses.computeIfAbsent(name, SimCANBusStub::new);
    }
    initialized = true;
  }

  /** For Phoenix device constructors (REAL/REPLAY only). */
  public static CANBus getBus(String name) {
    checkInit();
    if (sim) {
      throw new IllegalStateException("No real CANBus in SIM. Use getLike() or skip CTRE devices.");
    }
    CANBus bus = realBuses.get(name);
    if (bus == null) throwUnknown(name, realBuses.keySet());
    return bus;
  }

  /** For health logging (REAL or SIM). */
  public static CANBusLike getLike(String name) {
    checkInit();
    CANBusLike bus = likeBuses.get(name);
    if (bus == null) throwUnknown(name, likeBuses.keySet());
    return bus;
  }

  private static void checkInit() {
    if (!initialized) throw new IllegalStateException("RBSICANBusRegistry not initialized.");
  }

  private static void throwUnknown(String name, java.util.Set<String> known) {
    throw new IllegalArgumentException("Unknown CAN bus '" + name + "'. Known: " + known);
  }

  // ---- nested types ----

  public interface CANBusLike {
    String getName();

    CANBus.CANBusStatus getStatus();
  }

  static final class RealCANBusAdapter implements CANBusLike {
    private final CANBus bus;

    RealCANBusAdapter(CANBus bus) {
      this.bus = bus;
    }

    @Override
    public String getName() {
      return bus.getName();
    }

    @Override
    public CANBus.CANBusStatus getStatus() {
      return bus.getStatus();
    }
  }

  static final class SimCANBusStub implements CANBusLike {
    private final String name;

    SimCANBusStub(String name) {
      this.name = name;
    }

    @Override
    public String getName() {
      return name;
    }

    @Override
    public CANBus.CANBusStatus getStatus() {
      return new CANBus.CANBusStatus();
    }
  }
}

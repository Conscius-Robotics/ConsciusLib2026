package com.team10043.lib.util.control;

import com.team10043.frc2026.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A tunable numeric value that can be adjusted at runtime via NetworkTables when tuning mode is
 * enabled. Falls back to a fixed default value otherwise.
 *
 * <p>Designed for safe use in competition, simulation, and AdvantageKit logging.
 */
public class LoggedTunableNumber implements DoubleSupplier {

  /** Root NetworkTables path for all tunable values. */
  private static final String TABLE_ROOT = "/Tuning";

  /** Full NetworkTables key for this value. */
  private final String key;

  /** Default (fallback) value. */
  private double defaultValue;

  /** Whether a default value has been initialized. */
  private boolean hasDefault = false;

  /** NetworkTables-backed value used during tuning. */
  private LoggedNetworkNumber dashboardNumber;

  /**
   * Stores last-seen values per caller ID for change detection. Allows the same tunable number to
   * be shared safely.
   */
  private final Map<Integer, Double> lastValuesByCaller = new HashMap<>();

  /**
   * Creates a tunable number without a default value.
   *
   * @param dashboardKey Key relative to /Tuning
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = TABLE_ROOT + "/" + dashboardKey;
  }

  /**
   * Creates a tunable number with a default value.
   *
   * @param dashboardKey Key relative to /Tuning
   * @param defaultValue Initial default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Initializes the default value. This method is safe to call multiple times but will only take
   * effect once.
   *
   * @param defaultValue The default value to use when tuning is disabled
   */
  public void initDefault(double defaultValue) {
    if (hasDefault) {
      return;
    }

    this.hasDefault = true;
    this.defaultValue = defaultValue;

    if (isTuningActive()) {
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
  }

  /**
   * Returns the current value.
   *
   * <ul>
   *   <li>If tuning is active, returns the NetworkTables value
   *   <li>Otherwise, returns the default value
   * </ul>
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    }

    return (isTuningActive() && dashboardNumber != null) ? dashboardNumber.get() : defaultValue;
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  /**
   * Checks whether this value has changed since the last call by the given caller.
   *
   * @param callerId Unique identifier for the caller (usually hashCode())
   * @return True if the value has changed
   */
  public boolean hasChanged(int callerId) {
    double currentValue = get();
    Double lastValue = lastValuesByCaller.get(callerId);

    if (lastValue == null || Double.compare(currentValue, lastValue) != 0) {
      lastValuesByCaller.put(callerId, currentValue);
      return true;
    }
    return false;
  }

  /**
   * Runs an action if any of the provided tunable numbers have changed.
   *
   * @param callerId Unique identifier for the caller
   * @param action Action to run with current values
   * @param numbers Tunable numbers to monitor
   */
  public static void ifChanged(
      int callerId, Consumer<double[]> action, LoggedTunableNumber... numbers) {

    boolean anyChanged = Arrays.stream(numbers).anyMatch(number -> number.hasChanged(callerId));

    if (anyChanged) {
      double[] values = Arrays.stream(numbers).mapToDouble(LoggedTunableNumber::get).toArray();
      action.accept(values);
    }
  }

  /**
   * Runs an action if any of the provided tunable numbers have changed.
   *
   * @param callerId Unique identifier for the caller
   * @param action Action to run
   * @param numbers Tunable numbers to monitor
   */
  public static void ifChanged(int callerId, Runnable action, LoggedTunableNumber... numbers) {
    ifChanged(callerId, values -> action.run(), numbers);
  }

  public static void ifChanged(int callerId, DoubleConsumer action, LoggedTunableNumber number) {
    if (number.hasChanged(callerId)) {
      action.accept(number.get());
    }
  }

  private static boolean isTuningActive() {
    return Constants.TUNING_MODE && !Constants.DISABLE_HAL;
  }
}

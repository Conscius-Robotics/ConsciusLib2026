package com.team10043.lib.util.control;

import java.util.function.Consumer;

/**
 * Manages runtime tuning of SysId characterization parameters using NetworkTables-backed {@link
 * LoggedTunableNumber}s.
 *
 * <p>This class provides a centralized way to manage SysId test parameters (ramp rate, max
 * current/voltage, tolerance, etc.) that can be adjusted via AdvantageScope when tuning mode is
 * enabled.
 *
 * <p><b>Static vs. Per-Instance Usage:</b>
 *
 * <ul>
 *   <li>For <b>shared characterization commands</b> (e.g., static friction tests), use static
 *       instances with fixed defaults set at construction.
 *   <li>For <b>per-subsystem tuning</b>, create per-instance managers and call {@link
 *       #initDefaults(double, double, double)} once during subsystem initialization.
 * </ul>
 *
 * <p>Typical usage (static commands):
 *
 * <pre>
 * // Defaults are fixed at construction
 * private static final TunableSysIdManager sysId =
 *     new TunableSysIdManager("SysId/Motor/StaticTest", 1.0, 30.0, 0.01, params -> {});
 * </pre>
 */
public class TunableSysIdManager {

  private final LoggedTunableNumber rampRate;
  private final LoggedTunableNumber maxCurrent;
  private final LoggedTunableNumber tolerance;

  private final int callerId = hashCode();
  private final Consumer<SysIdParams> onChange;

  /**
   * Constructs a new {@code TunableSysIdManager} for static friction testing.
   *
   * @param prefix NetworkTables path prefix for SysId values
   * @param defaultRampRate default current/voltage ramp rate (A/s or V/s)
   * @param defaultMaxCurrent default maximum current or voltage
   * @param defaultTolerance default velocity tolerance for movement detection
   * @param onChange callback invoked when any parameter changes
   */
  public TunableSysIdManager(
      String prefix,
      double defaultRampRate,
      double defaultMaxCurrent,
      double defaultTolerance,
      Consumer<SysIdParams> onChange) {

    this.rampRate = new LoggedTunableNumber(prefix + "/RampRate", defaultRampRate);
    this.maxCurrent = new LoggedTunableNumber(prefix + "/MaxCurrent", defaultMaxCurrent);
    this.tolerance = new LoggedTunableNumber(prefix + "/Tolerance", defaultTolerance);
    this.onChange = onChange;
  }

  /**
   * Checks for updated SysId parameters and invokes the change callback if any value has been
   * modified since the last update.
   *
   * <p>This method should be called periodically or before test execution.
   */
  public void update() {
    LoggedTunableNumber.ifChanged(
        callerId,
        () -> onChange.accept(new SysIdParams(rampRate.get(), maxCurrent.get(), tolerance.get())),
        rampRate,
        maxCurrent,
        tolerance);
  }

  /**
   * Initializes the tunable parameters with default values.
   *
   * <p><b>IMPORTANT:</b> Can only be called once - subsequent calls are ignored due to how {@link
   * LoggedTunableNumber#initDefault(double)} works. This is intentional to prevent accidental
   * overwriting of defaults.
   *
   * <p>For static command managers (shared across multiple subsystems), defaults should be set in
   * the constructor. For per-subsystem managers, call this once during subsystem initialization.
   *
   * @param defaultRampRate default ramp rate value
   * @param defaultMaxCurrent default max current value
   * @param defaultTolerance default tolerance value
   */
  public void initDefaults(
      double defaultRampRate, double defaultMaxCurrent, double defaultTolerance) {
    rampRate.initDefault(defaultRampRate);
    maxCurrent.initDefault(defaultMaxCurrent);
    tolerance.initDefault(defaultTolerance);
  }

  /** Returns the current SysId parameters. */
  public SysIdParams get() {
    return new SysIdParams(rampRate.get(), maxCurrent.get(), tolerance.get());
  }

  /**
   * SysId characterization parameters.
   *
   * @param rampRate current or voltage ramp rate
   * @param maxCurrent maximum current or voltage to apply
   * @param tolerance velocity tolerance for movement detection
   */
  public record SysIdParams(double rampRate, double maxCurrent, double tolerance) {}
}

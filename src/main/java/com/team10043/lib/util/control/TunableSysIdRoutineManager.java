package com.team10043.lib.util.control;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Consumer;

/**
 * Manages runtime tuning of WPILib SysIdRoutine parameters using NetworkTables-backed {@link
 * LoggedTunableNumber}s.
 *
 * <p>This class provides a centralized way to manage SysIdRoutine test parameters (ramp rate, step
 * voltage, timeout) that can be adjusted via AdvantageScope when tuning mode is enabled.
 *
 * <p>Similar to {@link TunablePIDManager}, this follows the reactive pattern where the callback is
 * invoked automatically when parameters change.
 *
 * <p><b>Thread Safety:</b> This class is designed to be called from the main robot thread during
 * periodic loops. The {@link #update()} method should only be invoked when no commands are actively
 * using the associated SysIdRoutine to prevent race conditions.
 *
 * <p><b>Initialization:</b> You MUST call {@link #initDefaults(Velocity, Voltage, Time)} before
 * using this manager. The constructor does not set defaults to allow subsystems to provide their
 * own default values (e.g., different step voltages for different mechanisms).
 *
 * <p>Typical usage:
 *
 * <pre>
 * TunableSysIdRoutineManager sysIdManager = new TunableSysIdRoutineManager(
 *     "SysId/Motor/Shooter",
 *     params -> this.sysIdRoutine = createSysIdRoutineInternal(params));
 *
 * // In createSysIdRoutine() - set subsystem-specific defaults
 * sysIdManager.initDefaults(
 *     Units.Volts.of(1.0).per(Units.Second),
 *     Units.Volts.of(7.0),
 *     Units.Seconds.of(10.0));
 *
 * // In periodic() - only update when subsystem is idle
 * if (getCurrentCommand() == null) {
 *   sysIdManager.update();
 * }
 * </pre>
 */
public class TunableSysIdRoutineManager {

  private final LoggedTunableNumber rampRateVoltsPerSec;
  private final LoggedTunableNumber stepVoltage;
  private final LoggedTunableNumber timeoutSeconds;

  private final int callerId = hashCode();
  private final Consumer<SysIdRoutineParams> onChange;

  /**
   * Constructs a new {@code TunableSysIdRoutineManager}.
   *
   * <p><b>Important:</b> This constructor does NOT set default values. You must call {@link
   * #initDefaults(Velocity, Voltage, Time)} to set the default values before using the manager.
   *
   * @param prefix NetworkTables path prefix for SysId values
   * @param onChange callback invoked when any parameter changes
   */
  public TunableSysIdRoutineManager(String prefix, Consumer<SysIdRoutineParams> onChange) {
    this.rampRateVoltsPerSec = new LoggedTunableNumber(prefix + "/RampRate");
    this.stepVoltage = new LoggedTunableNumber(prefix + "/StepVoltage");
    this.timeoutSeconds = new LoggedTunableNumber(prefix + "/Timeout");
    this.onChange = onChange;
  }

  /**
   * Checks for updated SysId parameters and invokes the change callback if any value has been
   * modified since the last update.
   *
   * <p>This method should be called periodically (e.g. in a subsystem periodic loop) but ONLY when
   * no commands are actively using the SysIdRoutine to avoid race conditions during
   * characterization tests.
   *
   * <p><b>Important:</b> Call this only when {@code getCurrentCommand() == null} to ensure thread
   * safety.
   */
  public void update() {
    LoggedTunableNumber.ifChanged(
        callerId,
        () ->
            onChange.accept(
                new SysIdRoutineParams(
                    Units.Volts.of(rampRateVoltsPerSec.get()).per(Units.Second),
                    Units.Volts.of(stepVoltage.get()),
                    Units.Seconds.of(timeoutSeconds.get()))),
        rampRateVoltsPerSec,
        stepVoltage,
        timeoutSeconds);
  }

  /**
   * Initializes the tunable parameters with default values.
   *
   * <p><b>MUST be called before using the manager.</b> This sets the default values that will be
   * used when tuning mode is disabled. Can only be called once - subsequent calls are ignored.
   *
   * @param rampRate default ramp rate
   * @param stepVoltage default step voltage
   * @param timeout default timeout
   */
  public void initDefaults(Velocity<VoltageUnit> rampRate, Voltage stepVoltage, Time timeout) {
    rampRateVoltsPerSec.initDefault(rampRate.in(Units.Volts.per(Units.Second)));
    this.stepVoltage.initDefault(stepVoltage.in(Units.Volts));
    this.timeoutSeconds.initDefault(timeout.in(Units.Seconds));
  }

  /** Returns the current SysId routine parameters. */
  public SysIdRoutineParams get() {
    return new SysIdRoutineParams(
        Units.Volts.of(rampRateVoltsPerSec.get()).per(Units.Second),
        Units.Volts.of(stepVoltage.get()),
        Units.Seconds.of(timeoutSeconds.get()));
  }

  /**
   * SysId routine configuration parameters.
   *
   * @param rampRate voltage ramp rate for quasistatic tests
   * @param stepVoltage step voltage for dynamic tests
   * @param timeout maximum test duration
   */
  public record SysIdRoutineParams(
      Velocity<VoltageUnit> rampRate, Voltage stepVoltage, Time timeout) {}
}

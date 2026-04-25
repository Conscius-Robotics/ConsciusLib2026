package com.team10043.lib.util.control;

import com.team10043.lib.util.control.ControlGains.PIDGains;
import java.util.function.Consumer;

/**
 * Manages runtime tuning of PID gains using NetworkTables-backed {@link LoggedTunableNumber}s.
 *
 * <p>This class is generic and intentionally decoupled from any specific control system (swerve,
 * arm, elevator, etc.). It observes changes to tunable PID values and notifies a consumer when
 * updates occur.
 *
 * <p>Typical usage:
 *
 * <pre>
 * TunablePIDManager tuner = new TunablePIDManager(
 *     "Swerve/Translation",
 *     1.0, 0.0, 0.0,
 *     gains -> controller.setTranslationPID(
 *         gains.kP(), gains.kI(), gains.kD()));
 *
 * tuner.update(); // called periodically
 * </pre>
 */
public class TunablePIDManager {

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  private final int callerId = hashCode();
  private final Consumer<PIDGains> onChange;

  /**
   * Constructs a new {@code TunablePIDManager}.
   *
   * @param prefix NetworkTables path prefix for PID values
   * @param defaultKP default proportional gain
   * @param defaultKI default integral gain
   * @param defaultKD default derivative gain
   * @param onChange callback invoked when any PID value changes
   */
  public TunablePIDManager(
      String prefix,
      double defaultKP,
      double defaultKI,
      double defaultKD,
      Consumer<PIDGains> onChange) {

    this.kP = new LoggedTunableNumber(prefix + "/kP", defaultKP);
    this.kI = new LoggedTunableNumber(prefix + "/kI", defaultKI);
    this.kD = new LoggedTunableNumber(prefix + "/kD", defaultKD);
    this.onChange = onChange;
  }

  /**
   * Checks for updated PID values and invokes the change callback if any value has been modified
   * since the last update.
   *
   * <p>This method should be called periodically (e.g. in a subsystem periodic loop or command
   * execute).
   */
  public void update() {
    LoggedTunableNumber.ifChanged(
        callerId, () -> onChange.accept(new PIDGains(kP.get(), kI.get(), kD.get())), kP, kI, kD);
  }

  /** Returns the current PID gains. */
  public PIDGains get() {
    return new PIDGains(kP.get(), kI.get(), kD.get());
  }
}

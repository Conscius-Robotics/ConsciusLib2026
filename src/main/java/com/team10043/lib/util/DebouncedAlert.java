package com.team10043.lib.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.function.BooleanSupplier;
import lombok.Getter;

/**
 * Debounced boolean condition monitor that drives a WPILib {@link Alert}.
 *
 * <p>This class provides noise-resistant fault detection for:
 *
 * <ul>
 *   <li>Hardware connection monitoring
 *   <li>Limit switch debouncing
 *   <li>Sensor validity checks
 *   <li>Any transient fault signal
 * </ul>
 */
public class DebouncedAlert {

  /** Default debounce time in seconds. */
  public static final double DEFAULT_DEBOUNCE_SECONDS = 0.5;

  private final BooleanSupplier conditionSupplier;
  private final BooleanSupplier alertsEnabledSupplier;
  private final Debouncer debouncer;
  private final Alert alert;

  /** Whether the alert is currently active. */
  @Getter private boolean alertActive = false;

  /**
   * Creates a debounced alert with full configuration.
   *
   * @param conditionSupplier healthy condition ({@code true} = OK, {@code false} = fault)
   * @param alertsEnabledSupplier whether alerts should fire
   * @param alertText alert message
   * @param debounceSeconds debounce duration
   * @param alertType severity level
   */
  public DebouncedAlert(
      BooleanSupplier conditionSupplier,
      BooleanSupplier alertsEnabledSupplier,
      String alertText,
      double debounceSeconds,
      AlertType alertType) {
    this.conditionSupplier = conditionSupplier;
    this.alertsEnabledSupplier = alertsEnabledSupplier;
    this.alert = new Alert(alertText, alertType);
    this.debouncer = new Debouncer(debounceSeconds, Debouncer.DebounceType.kFalling);
  }

  /**
   * Creates a debounced error alert with default timing.
   *
   * @param conditionSupplier healthy condition
   * @param alertsEnabledSupplier whether alerts should fire
   * @param alertText alert message
   * @param debounceSeconds debounce duration
   */
  public DebouncedAlert(
      BooleanSupplier conditionSupplier,
      BooleanSupplier alertsEnabledSupplier,
      String alertText,
      double debounceSeconds) {
    this(conditionSupplier, alertsEnabledSupplier, alertText, debounceSeconds, AlertType.kError);
  }

  /**
   * Creates a debounced error alert with default timing (0.5s).
   *
   * @param conditionSupplier healthy condition
   * @param alertsEnabledSupplier whether alerts should fire
   * @param alertText alert message
   */
  public DebouncedAlert(
      BooleanSupplier conditionSupplier, BooleanSupplier alertsEnabledSupplier, String alertText) {
    this(conditionSupplier, alertsEnabledSupplier, alertText, DEFAULT_DEBOUNCE_SECONDS);
  }

  /**
   * Updates the debounced condition and alert state.
   *
   * <p>This method should be called periodically.
   */
  public void update() {
    boolean healthy = conditionSupplier.getAsBoolean();
    boolean debouncedHealthy = debouncer.calculate(healthy);
    alertActive = !debouncedHealthy && alertsEnabledSupplier.getAsBoolean();
    alert.set(alertActive);
  }

  /**
   * Creates a connection monitor alert.
   *
   * @param connectionSupplier supplies connection status
   * @param deviceName name for the alert message
   * @return configured alert
   */
  public static DebouncedAlert forConnection(
      BooleanSupplier connectionSupplier, String deviceName) {
    return new DebouncedAlert(connectionSupplier, () -> true, deviceName + " disconnected");
  }

  /**
   * Creates an always-enabled fault alert.
   *
   * @param conditionSupplier fault condition (true = OK)
   * @param message alert message
   * @return configured alert
   */
  public static DebouncedAlert alwaysEnabled(BooleanSupplier conditionSupplier, String message) {
    return new DebouncedAlert(conditionSupplier, () -> true, message);
  }
}

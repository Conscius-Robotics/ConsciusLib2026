package com.team10043.lib.util.revlib;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * Utility methods for working with REV Robotics Spark MAX controllers.
 *
 * <p>This class provides safe wrappers around common REV configuration calls, including retry logic
 * and standardized error reporting to the Driver Station.
 */
public final class REVUtil {

  /** Default number of retries for REV API calls that may fail on CAN bus contention. */
  public static final int DEFAULT_RETRY_COUNT = 10;

  /** Recommended timeout (in seconds) for Spark MAX configuration operations. */
  public static final double CONFIG_TIMEOUT_SECONDS = 0.05;

  private REVUtil() {
    // Prevent instantiation
  }

  /**
   * Repeatedly executes a REV API call until it succeeds or the retry limit is reached.
   *
   * <p>This is commonly used for Spark MAX configuration calls, which may intermittently fail due
   * to CAN bus load during robot initialization.
   *
   * @param revCall the REV API call to execute
   * @param deviceId CAN device ID of the motor controller (for error reporting)
   * @param actionDescription human-readable description of the attempted action
   * @return the final {@link REVLibError} result
   */
  public static REVLibError tryUntilOK(
      Supplier<REVLibError> revCall, int deviceId, String actionDescription) {

    REVLibError status = REVLibError.kOk;

    for (int attempt = 1; attempt <= DEFAULT_RETRY_COUNT; attempt++) {
      status = revCall.get();

      if (status == REVLibError.kOk) {
        return status;
      }
    }

    DriverStation.reportError(
        String.format(
            "REV call failed after %d attempts on device %d (%s): %s",
            DEFAULT_RETRY_COUNT, deviceId, actionDescription, status),
        true);

    return status;
  }

  /**
   * Applies a Spark MAX configuration with retry logic and error reporting.
   *
   * @param motor the Spark MAX motor controller
   * @param config the configuration to apply
   * @param resetMode specifies how safe parameters are reset
   * @param persistMode specifies whether parameters are saved to flash
   * @return the resulting {@link REVLibError}
   */
  public static REVLibError applyConfiguration(
      SparkMax motor, SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {

    return tryUntilOK(
        () -> motor.configure(config, resetMode, persistMode),
        motor.getDeviceId(),
        "SparkMaxConfiguration");
  }

  /**
   * Applies a Spark MAX configuration using safe defaults.
   *
   * <p>This method resets safe parameters and persists the configuration to flash, making it
   * suitable for use during robot initialization.
   *
   * @param motor the Spark MAX motor controller
   * @param config the configuration to apply
   * @return the resulting {@link REVLibError}
   */
  public static REVLibError applyConfigurationBlocking(SparkMax motor, SparkBaseConfig config) {
    return applyConfiguration(
        motor, config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Applies a Spark MAX configuration without retries, resets, or flash persistence.
   *
   * <p>This is intended for fast, non-blocking updates during runtime where configuration
   * persistence is not required.
   *
   * @param motor the Spark MAX motor controller
   * @param config the configuration to apply
   * @return the resulting {@link REVLibError}
   */
  public static REVLibError applyConfigurationNonBlocking(SparkMax motor, SparkBaseConfig config) {
    return motor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Converts a numeric PID slot index to a {@link ClosedLoopSlot}.
   *
   * <p>If the index is out of range, {@link ClosedLoopSlot#kSlot0} is returned.
   *
   * @param slot PID slot index (0–3)
   * @return corresponding {@link ClosedLoopSlot}
   */
  public static ClosedLoopSlot getSlotByIndex(int slot) {

    if (slot < 0 || slot > 3) {
      DriverStation.reportWarning(
          String.format("Invalid PID slot index %d; defaulting to slot 0", slot), false);
    }

    return switch (slot) {
      case 0 -> ClosedLoopSlot.kSlot0;
      case 1 -> ClosedLoopSlot.kSlot1;
      case 2 -> ClosedLoopSlot.kSlot2;
      case 3 -> ClosedLoopSlot.kSlot3;
      default -> ClosedLoopSlot.kSlot0;
    };
  }
}

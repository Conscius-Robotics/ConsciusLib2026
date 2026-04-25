package com.team10043.lib.util.phoenix6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonDevice;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

/**
 * Utility methods for working with CTRE Phoenix 6 devices.
 *
 * <p>This class provides retry-safe configuration helpers, CAN bus–aware signal synchronization,
 * and standardized error reporting for TalonFX and CANcoder devices on both RIO and CANivore buses.
 */
public final class CTREUtil {

  /**
   * Container for refresh status results from both CANivore and RIO buses.
   *
   * @param canbusStatusCode status of CANivore signal refresh
   * @param rioStatusCode status of RIO signal refresh
   */
  public record CANStatus(StatusCode canbusStatusCode, StatusCode rioStatusCode) {}

  /** Default number of retries for CTRE configuration calls. */
  public static final int DEFAULT_RETRY_COUNT = 5;

  /** Default timeout (in seconds) for CTRE configuration calls. */
  public static final double DEFAULT_TIMEOUT_SEC = 0.25;

  /** Signals registered for synchronized refresh on CANivore buses. */
  private static final List<BaseStatusSignal> canivoreSignalsList = new ArrayList<>();

  /** Signals registered for synchronized refresh on the RIO CAN bus. */
  private static final List<BaseStatusSignal> rioSignalsList = new ArrayList<>();

  /** Lock protecting access to signal registration and refresh operations. */
  private static final Object signalLock = new Object();

  private CTREUtil() {
    // Prevent instantiation
  }

  /**
   * Repeatedly executes a CTRE API call until it succeeds or the retry limit is reached.
   *
   * <p>This is primarily used for configuration calls that may intermittently fail due to CAN bus
   * load during startup.
   *
   * @param maxAttempts Maximum number of retry attempts before the CTRE call is considered failed
   *     and an error is reported.
   * @param ctreCall the CTRE API call to execute
   * @param deviceId CAN device ID (used for error reporting)
   * @param actionDescription human-readable description of the attempted action
   * @return the final {@link StatusCode} result
   */
  public static StatusCode tryUntilOK(
      int maxAttempts, Supplier<StatusCode> ctreCall, int deviceId, String actionDescription) {

    StatusCode status = StatusCode.OK;

    for (int attempt = 0; attempt < maxAttempts; attempt++) {
      status = ctreCall.get();
      if (status.isOK()) {
        return status;
      }
    }

    DriverStation.reportError(
        buildErrorMessage(maxAttempts, deviceId, actionDescription, status), true);

    return status;
  }

  /**
   * Repeatedly executes a CTRE API call until it succeeds or the retry limit is reached.
   *
   * <p>This is primarily used for configuration calls that may intermittently fail due to CAN bus
   * load during startup.
   *
   * @param maxAttempts Maximum number of retry attempts before the CTRE call is considered failed
   *     and an error is reported.
   * @param ctreCall the CTRE API call to execute
   * @param actionDescription human-readable description of the attempted action
   * @return the final {@link StatusCode} result
   */
  public static StatusCode tryUntilOK(
      int maxAttempts, Supplier<StatusCode> ctreCall, String actionDescription) {
    return tryUntilOK(maxAttempts, ctreCall, -1, actionDescription);
  }

  /**
   * Repeatedly executes a CTRE API call until it succeeds or the retry limit is reached.
   *
   * <p>This is primarily used for configuration calls that may intermittently fail due to CAN bus
   * load during startup.
   *
   * @param ctreCall the CTRE API call to execute
   * @param deviceId CAN device ID (used for error reporting)
   * @param actionDescription human-readable description of the attempted action
   * @return the final {@link StatusCode} result
   */
  public static StatusCode tryUntilOK(
      Supplier<StatusCode> ctreCall, int deviceId, String actionDescription) {
    return tryUntilOK(DEFAULT_RETRY_COUNT, ctreCall, deviceId, actionDescription);
  }

  /**
   * Repeatedly executes a CTRE API call until it succeeds or the retry limit is reached.
   *
   * <p>This is primarily used for configuration calls that may intermittently fail due to CAN bus
   * load during startup.
   *
   * @param ctreCall the CTRE API call to execute
   * @param actionDescription human-readable description of the attempted action
   * @return the final {@link StatusCode} result
   */
  public static StatusCode tryUntilOK(Supplier<StatusCode> ctreCall, String actionDescription) {
    return tryUntilOK(DEFAULT_RETRY_COUNT, ctreCall, -1, actionDescription);
  }

  /**
   * Builds a standardized error message for failed CTRE API calls.
   *
   * @param maxAttempts number of attempts made
   * @param deviceId CAN device ID (if applicable)
   * @param actionDescription description of the attempted action
   * @param status final {@link StatusCode} result
   * @return formatted error message
   */
  private static String buildErrorMessage(
      int maxAttempts, int deviceId, String actionDescription, StatusCode status) {
    return deviceId > 0
        ? String.format(
            "CTRE call failed after %d attempts on device %d (%s): %s",
            maxAttempts, deviceId, actionDescription, status)
        : String.format(
            "CTRE call failed after %d attempts (%s): %s", maxAttempts, actionDescription, status);
  }

  /**
   * Applies a TalonFX configuration with retry logic and error reporting.
   *
   * @param motor the TalonFX motor controller
   * @param config the configuration to apply
   * @return the resulting {@link StatusCode}
   */
  public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
    return tryUntilOK(
        () -> applyConfigurationNonBlocking(motor, config),
        motor.getDeviceID(),
        "TalonFXConfiguration");
  }

  /**
   * Applies a TalonFX configuration without retries.
   *
   * <p>This method is intended for fast, runtime configuration updates where blocking retries are
   * undesirable.
   *
   * @param motor the TalonFX motor controller
   * @param config the configuration to apply
   * @return the resulting {@link StatusCode}
   */
  public static StatusCode applyConfigurationNonBlocking(
      TalonFX motor, TalonFXConfiguration config) {
    return motor.getConfigurator().apply(config, DEFAULT_TIMEOUT_SEC);
  }

  /**
   * Applies a CANcoder configuration with retry logic and error reporting.
   *
   * @param cancoder the CANcoder device
   * @param config the configuration to apply
   * @return the resulting {@link StatusCode}
   */
  public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
    return tryUntilOK(
        () -> applyConfigurationNonBlocking(cancoder, config),
        cancoder.getDeviceID(),
        "CANcoderConfiguration");
  }

  /**
   * Applies a CANcoder configuration without retries.
   *
   * <p>This method is intended for fast, runtime configuration updates where blocking retries are
   * undesirable.
   *
   * @param cancoder the CANcoder device
   * @param config the configuration to apply
   * @return the resulting {@link StatusCode}
   */
  public static StatusCode applyConfigurationNonBlocking(
      CANcoder cancoder, CANcoderConfiguration config) {
    return cancoder.getConfigurator().apply(config, DEFAULT_TIMEOUT_SEC);
  }

  /**
   * Applies a Pigeon2 configuration with retry logic and error reporting.
   *
   * @param pigeon the Pigeon2 device
   * @param config the configuration to apply
   * @return the resulting {@link StatusCode}
   */
  public static StatusCode applyConfiguration(Pigeon2 pigeon, Pigeon2Configuration config) {
    return tryUntilOK(
        () -> applyConfigurationNonBlocking(pigeon, config),
        pigeon.getDeviceID(),
        "Pigeon2Configuration");
  }

  /**
   * Applies a Pigeon2 configuration without retries.
   *
   * <p>This method is intended for fast, runtime configuration updates where blocking retries are
   * undesirable.
   *
   * @param pigeon the Pigeon2 device
   * @param config the configuration to apply
   * @return the resulting {@link StatusCode}
   */
  public static StatusCode applyConfigurationNonBlocking(
      Pigeon2 pigeon, Pigeon2Configuration config) {
    return pigeon.getConfigurator().apply(config, DEFAULT_TIMEOUT_SEC);
  }

  /**
   * Determines whether the given CAN bus name refers to a CANivore bus.
   *
   * @param canBus the CAN bus name
   * @return true if the bus is a CANivore bus, false if it is the RIO bus
   */
  public static boolean isCanivoreCANBus(String canBus) {
    return !(canBus.equals("rio") || canBus.length() == 0);
  }

  /**
   * Registers a set of status signals for synchronized refresh.
   *
   * <p>Signals are grouped based on whether they are on the RIO or CANivore bus, allowing refresh
   * operations to be performed per bus.
   *
   * @param canbus the CAN bus name
   * @param signals the signals to register
   */
  public static void registerSignals(String canbus, BaseStatusSignal... signals) {
    registerSignals(isCanivoreCANBus(canbus), signals);
  }

  /**
   * Registers a set of status signals for synchronized refresh.
   *
   * <p>Signals are grouped based on whether they are on the RIO or CANivore bus, allowing refresh
   * operations to be performed per bus.
   *
   * @param isCanivore Indicates whether the signals belong to a CANivore CAN bus.
   * @param signals the signals to register
   */
  public static void registerSignals(boolean isCanivore, BaseStatusSignal... signals) {
    synchronized (signalLock) {
      List<BaseStatusSignal> targetList = isCanivore ? canivoreSignalsList : rioSignalsList;

      targetList.addAll(Arrays.asList(signals));
    }
  }

  /**
   * Refreshes all registered signals on both CANivore and RIO buses.
   *
   * @return a {@link CANStatus} containing the refresh results for each bus
   */
  public static CANStatus refreshAll() {
    synchronized (signalLock) {
      return new CANStatus(refreshCanivore(), refreshRIO());
    }
  }

  /**
   * Refreshes all registered CANivore signals.
   *
   * @return the resulting {@link StatusCode}
   */
  public static StatusCode refreshCanivore() {
    if (canivoreSignalsList.isEmpty()) {
      return StatusCode.OK;
    }

    StatusCode status =
        BaseStatusSignal.refreshAll(canivoreSignalsList.toArray(new BaseStatusSignal[0]));
    if (!status.isOK()) {
      DriverStation.reportWarning("Failed to refresh CANivore signals: " + status, false);
    }

    return status;
  }

  /**
   * Optimizes CAN bus utilization for the given CTRE device.
   *
   * @param device the CTRE device
   * @return the resulting {@link StatusCode}
   */
  public static StatusCode optimizeBusUtilization(CommonDevice device) {
    return tryUntilOK(
        device::optimizeBusUtilization, device.getDeviceID(), "Optimizing Bus Utilization");
  }

  /**
   * Optimizes CAN bus utilization for all given CTRE devices.
   *
   * @param devices the CTRE devices
   * @return the resulting {@link StatusCode}
   */
  public static StatusCode optimizeBusUtilizationForAll(CommonDevice... devices) {
    return tryUntilOK(
        () -> ParentDevice.optimizeBusUtilizationForAll(devices),
        "Optimize CAN Bus Utilization for Devices");
  }

  /**
   * Refreshes all registered RIO CAN bus signals.
   *
   * @return the resulting {@link StatusCode}
   */
  public static StatusCode refreshRIO() {
    if (rioSignalsList.isEmpty()) {
      return StatusCode.OK;
    }

    StatusCode status =
        BaseStatusSignal.refreshAll(rioSignalsList.toArray(new BaseStatusSignal[0]));
    if (!status.isOK()) {
      DriverStation.reportWarning("Failed to refresh RIO signals: " + status, false);
    }

    return status;
  }
}

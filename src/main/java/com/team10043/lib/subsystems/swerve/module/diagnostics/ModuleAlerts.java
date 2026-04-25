package com.team10043.lib.subsystems.swerve.module.diagnostics;

import com.team10043.frc2026.Robot;
import com.team10043.lib.subsystems.swerve.module.ModuleIOInputsAutoLogged;
import com.team10043.lib.util.DebouncedAlert;

/**
 * Manages debounced hardware connection alerts for a swerve module.
 *
 * <p>This class monitors drive motor, turn motor, and turn encoder connectivity and raises alerts
 * only after the fault condition has been stable for a configured debounce period.
 */
public class ModuleAlerts {

  /** Alert for a disconnected drive motor. */
  private final DebouncedAlert driveDisconnectedAlert;

  /** Alert for a disconnected turn motor. */
  private final DebouncedAlert turnDisconnectedAlert;

  /** Alert for a disconnected turn encoder. */
  private final DebouncedAlert turnEncoderDisconnectedAlert;

  /**
   * Creates a new {@code ModuleAlerts} instance.
   *
   * @param inputs module sensor and motor input data
   * @param index module index used for alert identification
   */
  public ModuleAlerts(ModuleIOInputsAutoLogged inputs, int index) {
    driveDisconnectedAlert =
        new DebouncedAlert(
            () -> inputs.driveData.connected(),
            () -> !Robot.isJITing(),
            "Disconnected drive motor on module " + index + ".");

    turnDisconnectedAlert =
        new DebouncedAlert(
            () -> inputs.turnData.connected(),
            () -> !Robot.isJITing(),
            "Disconnected turn motor on module " + index + ".");

    turnEncoderDisconnectedAlert =
        new DebouncedAlert(
            () -> inputs.turnData.encoderConnected(),
            () -> !Robot.isJITing(),
            "Disconnected turn encoder on module " + index + ".");
  }

  /**
   * Updates all module alerts.
   *
   * <p>This method should be called periodically.
   */
  public void update() {
    driveDisconnectedAlert.update();
    turnDisconnectedAlert.update();
    turnEncoderDisconnectedAlert.update();
  }
}

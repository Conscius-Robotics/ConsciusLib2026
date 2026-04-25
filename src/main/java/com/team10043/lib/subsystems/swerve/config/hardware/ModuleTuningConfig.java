package com.team10043.lib.subsystems.swerve.config.hardware;

import com.team10043.lib.util.control.ControlGains.FeedforwardGains;
import com.team10043.lib.util.control.ControlGains.PIDGains;
import lombok.Builder;

/**
 * Configuration for module PID and feedforward tuning values.
 *
 * <p>These values control how individual swerve modules track velocity (drive) and position (turn)
 * setpoints. Supports separate tuning values for real robot and simulation. Values are applied
 * through {@link com.team10043.lib.subsystems.swerve.module.diagnostics.ModuleTuning}.
 *
 * @param realDrivePID PID gains for drive motor velocity control (real robot)
 * @param realDriveFF feedforward gains for drive motor (real robot)
 * @param realTurnPID PID gains for turn motor position control (real robot)
 * @param realTurnFF feedforward gains for turn motor (real robot)
 * @param simDrivePID PID gains for drive motor velocity control (simulation)
 * @param simDriveFF feedforward gains for drive motor (simulation)
 * @param simTurnPID PID gains for turn motor position control (simulation)
 * @param simTurnFF feedforward gains for turn motor (simulation)
 */
@Builder
public record ModuleTuningConfig(
    PIDGains realDrivePID,
    FeedforwardGains realDriveFF,
    PIDGains realTurnPID,
    FeedforwardGains realTurnFF,
    PIDGains simDrivePID,
    FeedforwardGains simDriveFF,
    PIDGains simTurnPID,
    FeedforwardGains simTurnFF) {

  public ModuleTuningConfig {
    if (realDrivePID == null) {
      throw new IllegalArgumentException("realDrivePID cannot be null");
    }
    if (realDriveFF == null) {
      throw new IllegalArgumentException("realDriveFF cannot be null");
    }
    if (realTurnPID == null) {
      throw new IllegalArgumentException("realTurnPID cannot be null");
    }
    if (realTurnFF == null) {
      throw new IllegalArgumentException("realTurnFF cannot be null");
    }
    if (simDrivePID == null) {
      throw new IllegalArgumentException("simDrivePID cannot be null");
    }
    if (simDriveFF == null) {
      throw new IllegalArgumentException("simDriveFF cannot be null");
    }
    if (simTurnPID == null) {
      throw new IllegalArgumentException("simTurnPID cannot be null");
    }
    if (simTurnFF == null) {
      throw new IllegalArgumentException("simTurnFF cannot be null");
    }
  }
}

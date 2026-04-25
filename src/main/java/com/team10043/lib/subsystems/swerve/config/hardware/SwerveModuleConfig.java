package com.team10043.lib.subsystems.swerve.config.hardware;

import lombok.Builder;

/**
 * Aggregate configuration for a complete swerve module.
 *
 * @param drive drive motor configuration
 * @param turn turn motor and encoder configuration
 * @param tuning PID and feedforward tuning values (optional, uses mode-specific defaults if null)
 */
@Builder
public record SwerveModuleConfig(
    SwerveDriveMotorConfig drive, SwerveTurnMotorConfig turn, ModuleTuningConfig tuning) {

  /**
   * Factory method for a complete swerve module config.
   *
   * @param driveConfig the drive motor configuration
   * @param turnConfig the turn motor and encoder configuration
   * @return a new SwerveModuleConfig instance with default tuning
   */
  public static SwerveModuleConfig of(
      SwerveDriveMotorConfig driveConfig, SwerveTurnMotorConfig turnConfig) {
    return SwerveModuleConfig.builder().drive(driveConfig).turn(turnConfig).tuning(null).build();
  }

  /**
   * Factory method for a complete swerve module config with tuning.
   *
   * @param driveConfig the drive motor configuration
   * @param turnConfig the turn motor and encoder configuration
   * @param tuningConfig PID and feedforward tuning values
   * @return a new SwerveModuleConfig instance
   */
  public static SwerveModuleConfig of(
      SwerveDriveMotorConfig driveConfig,
      SwerveTurnMotorConfig turnConfig,
      ModuleTuningConfig tuningConfig) {
    return SwerveModuleConfig.builder()
        .drive(driveConfig)
        .turn(turnConfig)
        .tuning(tuningConfig)
        .build();
  }
}

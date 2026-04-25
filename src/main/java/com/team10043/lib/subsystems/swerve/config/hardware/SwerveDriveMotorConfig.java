package com.team10043.lib.subsystems.swerve.config.hardware;

import lombok.Builder;

/**
 * Immutable configuration for the drive motor of a swerve module.
 *
 * <p>Contains all hardware identifiers and inversion settings required to configure and control the
 * drive motor.
 */
@Builder
public record SwerveDriveMotorConfig(
    /** the CAN ID of the drive motor controller */
    int motorId,
    /** whether the drive motor direction should be inverted */
    boolean inverted) {

  /**
   * Factory method for a drive motor config.
   *
   * @param motorId the CAN ID of the drive motor controller
   * @param inverted whether the drive motor direction should be inverted
   * @return a new SwerveDriveConfig instance
   */
  public static SwerveDriveMotorConfig of(int motorId, boolean inverted) {
    return SwerveDriveMotorConfig.builder().motorId(motorId).inverted(inverted).build();
  }
}

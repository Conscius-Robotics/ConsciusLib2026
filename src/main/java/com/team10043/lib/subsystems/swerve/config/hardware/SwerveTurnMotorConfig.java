package com.team10043.lib.subsystems.swerve.config.hardware;

import lombok.Builder;

/**
 * Immutable configuration for the turn (azimuth) motor and absolute encoder of a swerve module.
 *
 * <p>Stores motor identifiers, encoder channel, calibration offset, and inversion settings required
 * for absolute positioning.
 */
@Builder
public record SwerveTurnMotorConfig(
    /** the CAN ID of the turn motor controller */
    int motorId,
    /** the CAN ID or channel of the absolute encoder */
    int encoderChannel,
    /** the offset used to calibrate the encoder zero position */
    double encoderOffset,
    /** whether the turn motor direction should be inverted */
    boolean motorInverted,
    /** whether the encoder reading direction should be inverted */
    boolean encoderInverted) {

  /**
   * Factory method for a turn motor and encoder config.
   *
   * @param motorId the CAN ID of the turn motor controller
   * @param encoderChannel the CAN ID or channel of the absolute encoder
   * @param encoderOffset the offset used to calibrate the encoder zero position
   * @param motorInverted whether the turn motor direction should be inverted
   * @param encoderInverted whether the encoder reading direction should be inverted
   * @return a new SwerveTurnConfig instance
   */
  public static SwerveTurnMotorConfig of(
      int motorId,
      int encoderChannel,
      double encoderOffset,
      boolean motorInverted,
      boolean encoderInverted) {

    return SwerveTurnMotorConfig.builder()
        .motorId(motorId)
        .encoderChannel(encoderChannel)
        .encoderOffset(encoderOffset)
        .motorInverted(motorInverted)
        .encoderInverted(encoderInverted)
        .build();
  }
}

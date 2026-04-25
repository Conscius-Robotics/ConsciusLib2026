package com.team10043.lib.subsystems.swerve.config.hardware;

/**
 * Immutable container for swerve module current limits.
 *
 * <p>Stores continuous current limits, in amperes, applied to the drive and turn (azimuth) motor
 * controllers for a swerve module.
 *
 * <p>Both values are expected to be non-negative and should be chosen based on the motor and motor
 * controller specifications to protect hardware while allowing desired performance.
 *
 * @param driveCurrentLimit continuous current limit for the drive motor in amperes
 * @param turnCurrentLimit continuous current limit for the turn (azimuth) motor in amperes
 */
public record SwerveModuleCurrentLimits(int driveCurrentLimit, int turnCurrentLimit) {

  /** Compact constructor with validation. */
  public SwerveModuleCurrentLimits {
    if (driveCurrentLimit <= 0) {
      throw new IllegalArgumentException(
          "driveCurrentLimit must be positive, got: " + driveCurrentLimit);
    }
    if (turnCurrentLimit <= 0) {
      throw new IllegalArgumentException(
          "turnCurrentLimit must be positive, got: " + turnCurrentLimit);
    }
  }

  /**
   * Factory method to create a SwerveModuleCurrentLimits instance.
   *
   * @param driveCurrentLimit continuous current limit for the drive motor in amperes
   * @param turnCurrentLimit continuous current limit for the turn (azimuth) motor in amperes
   * @return a new SwerveModuleCurrentLimits instance
   */
  public static SwerveModuleCurrentLimits of(int driveCurrentLimit, int turnCurrentLimit) {
    return new SwerveModuleCurrentLimits(driveCurrentLimit, turnCurrentLimit);
  }
}

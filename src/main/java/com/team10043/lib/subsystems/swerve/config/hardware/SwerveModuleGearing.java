package com.team10043.lib.subsystems.swerve.config.hardware;

/**
 * Represents the gear reductions used by a swerve module's drive and steering gearboxes.
 *
 * <p>Both values are unitless gear ratios describing the relationship between motor rotations
 * (input) and output rotations:
 *
 * <ul>
 *   <li>{@code driveReduction} — number of motor revolutions per single wheel revolution
 *       (motorRotations : wheelRotations = driveReduction : 1).
 *   <li>{@code turnReduction} — number of motor revolutions per single module-azimuth revolution
 *       (motorRotations : azimuthRotations = turnReduction : 1).
 * </ul>
 *
 * <p>Both ratios must be positive and non-zero. The record is immutable.
 *
 * <p>Common conversions:
 *
 * <ul>
 *   <li>wheelRotations = motorRotations / driveReduction
 *   <li>wheelLinearDistance = wheelRotations * wheelCircumference
 *   <li>wheelRPM = motorRPM / driveReduction
 *   <li>moduleRotations = motorRotations / turnReduction
 *   <li>moduleAngleRadians = moduleRotations * 2 * Math.PI
 * </ul>
 *
 * <p>Use this record to centralize gearbox parameters for kinematics, odometry, and
 * encoder-to-physical-unit conversions.
 *
 * @param driveReduction the drive gearbox ratio (motor rotations per wheel rotation)
 * @param turnReduction the turning/azimuth gearbox ratio (motor rotations per azimuth rotation)
 */
public record SwerveModuleGearing(double driveReduction, double turnReduction) {

  /** Compact constructor with validation. */
  public SwerveModuleGearing {
    if (driveReduction <= 0.0) {
      throw new IllegalArgumentException("driveReduction must be positive, got: " + driveReduction);
    }
    if (turnReduction <= 0.0) {
      throw new IllegalArgumentException("turnReduction must be positive, got: " + turnReduction);
    }
  }

  /**
   * Factory method to create a SwerveModuleGearing instance.
   *
   * @param driveReduction the drive gearbox ratio (motor rotations per wheel rotation)
   * @param turnReduction the turning/azimuth gearbox ratio (motor rotations per azimuth rotation)
   * @return a new SwerveModuleGearing instance
   */
  public static SwerveModuleGearing of(double driveReduction, double turnReduction) {
    return new SwerveModuleGearing(driveReduction, turnReduction);
  }
}

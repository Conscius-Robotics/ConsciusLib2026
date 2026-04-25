package com.team10043.lib.subsystems.swerve.config.control;

import com.team10043.lib.subsystems.swerve.config.hardware.SwerveChassisDimensions;

/**
 * Immutable container of drivetrain motion limits derived from SwerveMotionConstraints and
 * SwerveChassisDimensions.
 *
 * <p>This record stores the original linear limits and the angular limits computed from the robot's
 * drive base radius r = chassisDimensions.driveBaseRadius():
 *
 * <pre>
 *   maxAngularSpeed = maxLinearSpeed / r
 *   maxAngularAcceleration = maxLinearAcceleration / r
 * </pre>
 *
 * All linear quantities are expressed in the same units as supplied by SwerveMotionConstraints.
 *
 * <p>Use {@link #from(SwerveMotionConstraints, SwerveChassisDimensions)} to construct an instance.
 * A non-zero positive drive base radius is assumed; if r == 0 the computed angular limits are
 * undefined (division by zero).
 *
 * @param maxLinearSpeed the maximum translational speed (same units as SwerveMotionConstraints)
 * @param maxLinearAcceleration the maximum translational acceleration (same units as
 *     SwerveMotionConstraints)
 * @param maxAngularSpeed the maximum rotational speed derived from linear limits and drive base
 *     radius
 * @param maxAngularAcceleration the maximum rotational acceleration derived from linear limits and
 *     drive base radius
 * @see SwerveMotionConstraints
 * @see SwerveChassisDimensions
 */
public record SwerveDerivedLimits(
    double maxLinearSpeed,
    double maxLinearAcceleration,
    double maxAngularSpeed,
    double maxAngularAcceleration) {

  public static SwerveDerivedLimits from(
      SwerveMotionConstraints motionConstraints, SwerveChassisDimensions chassisDimensions) {
    double r = chassisDimensions.driveBaseRadius();

    return new SwerveDerivedLimits(
        motionConstraints.maxLinearSpeed(),
        motionConstraints.maxLinearAcceleration(),
        motionConstraints.maxLinearSpeed() / r,
        motionConstraints.maxLinearAcceleration() / r);
  }
}

package com.team10043.lib.subsystems.swerve.config.control;

/**
 * Immutable record that encapsulates motion limits for a swerve drivetrain.
 *
 * <p>All values are expressed in SI units unless otherwise noted:
 *
 * <ul>
 *   <li>{@code maxLinearSpeed} — maximum translational velocity in meters per second (m/s)
 *   <li>{@code maxLinearAcceleration} — maximum translational acceleration in meters per second
 *       squared (m/s²)
 * </ul>
 *
 * <p>Convenience methods compute angular limits for a module located at a given radius from the
 * robot center:
 *
 * <ul>
 *   <li>{@link #maxAngularSpeed(double)} — returns the maximum angular speed in radians per second
 *       (rad/s), computed as {@code maxLinearSpeed / driveBaseRadius}.
 *   <li>{@link #maxAngularAcceleration(double)} — returns the maximum angular acceleration in
 *       radians per second squared (rad/s²), computed as {@code maxLinearAcceleration /
 *       driveBaseRadius}.
 * </ul>
 *
 * <p>Preconditions:
 *
 * <ul>
 *   <li>{@code driveBaseRadius} is expected to be positive (meters). Passing zero or a negative
 *       value may produce {@code Infinity} or {@code NaN} due to IEEE-754 division semantics.
 * </ul>
 *
 * <p>Instances of this record are immutable and therefore thread-safe.
 *
 * @param maxLinearSpeed maximum translational speed (m/s)
 * @param maxLinearAcceleration maximum translational acceleration (m/s²)
 */
public record SwerveMotionConstraints(double maxLinearSpeed, double maxLinearAcceleration) {

  /** Compact constructor with validation. */
  public SwerveMotionConstraints {
    if (maxLinearSpeed <= 0.0) {
      throw new IllegalArgumentException("maxLinearSpeed must be positive, got: " + maxLinearSpeed);
    }
    if (maxLinearAcceleration <= 0.0) {
      throw new IllegalArgumentException(
          "maxLinearAcceleration must be positive, got: " + maxLinearAcceleration);
    }
  }

  /**
   * Factory method to create a SwerveMotionConstraints instance.
   *
   * @param maxLinearSpeed maximum translational speed (m/s)
   * @param maxLinearAcceleration maximum translational acceleration (m/s²)
   * @return a new SwerveMotionConstraints instance
   */
  public static SwerveMotionConstraints of(double maxLinearSpeed, double maxLinearAcceleration) {
    return new SwerveMotionConstraints(maxLinearSpeed, maxLinearAcceleration);
  }

  /**
   * Calculates the maximum angular speed based on the drive base radius.
   *
   * @param driveBaseRadius the radius from robot center to module position (in meters)
   * @return the maximum angular speed (in radians per second)
   */
  public double maxAngularSpeed(double driveBaseRadius) {
    return maxLinearSpeed / driveBaseRadius;
  }

  /**
   * Calculates the maximum angular acceleration based on the drive base radius.
   *
   * @param driveBaseRadius the radius from robot center to module position (in meters)
   * @return the maximum angular acceleration (in radians per second squared)
   */
  public double maxAngularAcceleration(double driveBaseRadius) {
    return maxLinearAcceleration / driveBaseRadius;
  }
}

package com.team10043.lib.subsystems.motor.components.follower;

/**
 * No-op implementation of {@link FollowerComponent} used when no follower motors are configured for
 * a motor subsystem.
 *
 * <p>This class follows the <em>Null Object</em> pattern to eliminate the need for null checks in
 * subsystem logic. All methods either perform no action or return values consistent with a system
 * that has only a single leader motor.
 *
 * <p>In this implementation:
 *
 * <ul>
 *   <li>Leader position, velocity, and current values are returned unchanged.
 *   <li>Follower-specific queries return neutral values (zero or empty).
 *   <li>No operations are performed when methods are called.
 * </ul>
 *
 * <p>This allows {@code MotorSubsystem} to treat follower handling uniformly, regardless of whether
 * followers are actually present.
 */
public final class NullFollowerComponent implements FollowerComponent {

  @Override
  public void periodic(String systemName) {
    // Do nothing
  }

  @Override
  public double getAveragePosition(double leaderPosition) {
    return leaderPosition;
  }

  @Override
  public double getAverageVelocity(double leaderVelocity) {
    return leaderVelocity;
  }

  @Override
  public void setCurrentPositionAsZero() {
    // Do nothing
  }

  @Override
  public void setCurrentPosition(double position) {
    // Do nothing
  }

  @Override
  public int getFollowerCount() {
    return 0;
  }

  @Override
  public double getFollowerPosition(int index) {
    return 0.0;
  }

  @Override
  public double getFollowerVelocity(int index) {
    return 0.0;
  }

  @Override
  public double getFollowerCurrent(int index) {
    return 0.0;
  }

  @Override
  public double getFollowerVoltage(int index) {
    return 0.0;
  }

  @Override
  public double getTotalCurrent(double leaderCurrent) {
    return leaderCurrent;
  }

  @Override
  public double getAverageCurrent(double leaderCurrent) {
    return leaderCurrent;
  }
}

package com.team10043.lib.subsystems.motor.components.follower;

/**
 * Abstraction for a motor follower group associated with a single leader motor.
 *
 * <p>This interface represents one or more follower motors that are configured to follow a leader
 * motor at the controller level (e.g. CAN-based following). It provides read-only access to
 * follower state and utility methods to aggregate feedback across the leader and its followers.
 *
 * <p>Implementations may actively manage and log follower motors (see {@code FollowerComponent}) or
 * act as a no-op placeholder when no followers are configured (see {@code NullFollowerComponent}).
 *
 * <p>The interface is designed to be used by {@code MotorSubsystem} without requiring null checks,
 * following the Null Object pattern.
 */
public interface FollowerComponent {

  /**
   * Periodic update hook for follower motors.
   *
   * <p>Typically used to read and log follower motor inputs.
   *
   * @param systemName name of the owning subsystem (used for logging)
   */
  void periodic(String systemName);

  /**
   * Returns the average position across the leader motor and all followers.
   *
   * @param leaderPosition current leader position in rotations
   * @return average position of leader and followers in rotations
   */
  double getAveragePosition(double leaderPosition);

  /**
   * Returns the average velocity across the leader motor and all followers.
   *
   * @param leaderVelocity current leader velocity in rotations per second
   * @return average velocity of leader and followers in rotations per second
   */
  double getAverageVelocity(double leaderVelocity);

  /**
   * Sets the current position of all follower motors to zero.
   *
   * <p>This does not affect the leader motor position.
   */
  void setCurrentPositionAsZero();

  /**
   * Sets the current position of all follower motors to the given value.
   *
   * @param position position in rotations
   */
  void setCurrentPosition(double position);

  /**
   * Returns the number of configured follower motors.
   *
   * @return follower count
   */
  int getFollowerCount();

  /**
   * Returns the position of the specified follower motor.
   *
   * @param index follower index
   * @return follower position in rotations, or {@code 0.0} if index is invalid
   */
  double getFollowerPosition(int index);

  /**
   * Returns the velocity of the specified follower motor.
   *
   * @param index follower index
   * @return follower velocity in rotations per second, or {@code 0.0} if index is invalid
   */
  double getFollowerVelocity(int index);

  /**
   * Returns the stator current draw of the specified follower motor.
   *
   * @param index follower index
   * @return follower current in amps, or {@code 0.0} if index is invalid
   */
  double getFollowerCurrent(int index);

  /**
   * Returns the applied voltage of the specified follower motor.
   *
   * @param index follower index
   * @return follower applied voltage, or {@code 0.0} if index is invalid
   */
  double getFollowerVoltage(int index);

  /**
   * Returns the total current draw across the leader motor and all followers.
   *
   * @param leaderCurrent current draw of the leader motor in amps
   * @return total current draw in amps
   */
  double getTotalCurrent(double leaderCurrent);

  /**
   * Returns the average current draw across the leader motor and all followers.
   *
   * @param leaderCurrent current draw of the leader motor in amps
   * @return average current draw in amps
   */
  double getAverageCurrent(double leaderCurrent);

  /**
   * Indicates whether this follower component is active.
   *
   * <p>Disabled or placeholder implementations return {@code false}, allowing calling code to skip
   * follower-specific logic.
   *
   * @return {@code true} if follower motors are configured
   */
  default boolean isEnabled() {
    return !(this instanceof NullFollowerComponent);
  }
}

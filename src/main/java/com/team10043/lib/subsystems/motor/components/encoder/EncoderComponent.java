package com.team10043.lib.subsystems.motor.components.encoder;

import com.team10043.lib.encoders.EncoderIOInputsAutoLogged;
import com.team10043.lib.motors.MotorIO;

/**
 * Abstraction for an optional encoder component used by a motor subsystem.
 *
 * <p>This interface defines a common contract for encoder handling regardless of whether the
 * feedback comes from:
 *
 * <ul>
 *   <li>An external encoder (e.g. standalone CANCoder)
 *   <li>A motor-integrated sensor
 *   <li>A remote sensor configuration handled directly by the motor controller
 * </ul>
 *
 * <p>Concrete implementations may actively read encoder data and synchronize motor position (see
 * {@link ActiveEncoderComponent}), or act as a no-op placeholder when a separate encoder component
 * is not required (see {@link NullEncoderComponent}).
 *
 * <p>The interface is designed to be used by {@code MotorSubsystem} without requiring null checks,
 * following the Null Object pattern.
 */
public interface EncoderComponent {

  /**
   * Periodic update for the encoder.
   *
   * <p>Reads encoder inputs, logs them, and initializes the motor position using the absolute
   * encoder reading if the offset has not yet been applied.
   *
   * @param systemName parent subsystem name (used for logging)
   * @param motorIO motor IO used to set position offsets
   */
  void periodic(String systemName, MotorIO motorIO);

  /**
   * Resets or reapplies the encoder offset to the motor position.
   *
   * <p>Used when an absolute encoder value should be treated as the current motor position.
   *
   * @param motorIO motor IO used to update the motor position
   */
  void resetOffset(MotorIO motorIO);

  /**
   * Returns the logged encoder input structure.
   *
   * @return auto-logged encoder inputs
   */
  EncoderIOInputsAutoLogged getInputs();

  /**
   * Indicates whether the encoder offset has already been applied.
   *
   * @return {@code true} if the offset has been set, otherwise {@code false}
   */
  boolean hasSetOffset();

  /**
   * Returns the absolute encoder position in rotations.
   *
   * @return absolute position in rotations, or {@link Double#NaN} if unavailable
   */
  double getAbsolutePosition();

  /**
   * Returns the encoder velocity in rotations per second.
   *
   * @return velocity in rotations per second
   */
  double getVelocity();

  /**
   * Indicates whether this encoder component is active.
   *
   * <p>Disabled or placeholder implementations return {@code false}, allowing calling code to
   * conditionally execute encoder-related logic.
   *
   * @return {@code true} if the encoder component is active
   */
  default boolean isEnabled() {
    return !(this instanceof NullEncoderComponent);
  }
}

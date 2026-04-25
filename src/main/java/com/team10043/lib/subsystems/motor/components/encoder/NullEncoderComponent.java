package com.team10043.lib.subsystems.motor.components.encoder;

import com.team10043.lib.encoders.EncoderIOInputsAutoLogged;
import com.team10043.lib.motors.MotorIO;

/**
 * No-op implementation of {@link EncoderComponent} used when a separate encoder component is not
 * required.
 *
 * <p>This implementation is selected when:
 *
 * <ul>
 *   <li>No external encoder is configured
 *   <li>The motor controller already provides the feedback internally
 *   <li>A remote/integrated sensor mode is used (e.g. TalonFX using a CANCoder as a remote sensor)
 * </ul>
 *
 * <p>This class follows the Null Object pattern to eliminate null checks in {@code MotorSubsystem}.
 * All methods are safe to call and intentionally perform no actions.
 *
 * <p>Behavior:
 *
 * <ul>
 *   <li>{@link #periodic(String, MotorIO)} and {@link #resetOffset(MotorIO)} do nothing
 *   <li>{@link #getInputs()} returns a static, empty input instance
 *   <li>{@link #hasSetOffset()} always returns {@code true}
 *   <li>Position and velocity return safe default values
 * </ul>
 */
public final class NullEncoderComponent implements EncoderComponent {

  private static final EncoderIOInputsAutoLogged EMPTY_INPUTS = new EncoderIOInputsAutoLogged();

  @Override
  public void periodic(String systemName, MotorIO motorIO) {
    // disabled
  }

  @Override
  public void resetOffset(MotorIO motorIO) {
    // disabled
  }

  @Override
  public EncoderIOInputsAutoLogged getInputs() {
    return EMPTY_INPUTS;
  }

  @Override
  public boolean hasSetOffset() {
    return true;
  }

  @Override
  public double getAbsolutePosition() {
    return Double.NaN;
  }

  @Override
  public double getVelocity() {
    return 0.0;
  }
}

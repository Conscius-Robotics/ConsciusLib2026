package com.team10043.lib.encoders;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for encoder devices.
 *
 * <p>This interface follows the AdvantageKit IO pattern, allowing encoder implementations to be
 * swapped without affecting higher-level subsystem logic.
 */
public interface EncoderIO {

  /**
   * Logged input container for encoder measurements.
   *
   * <p>Values are populated by {@link #updateInputs(EncoderIOInputs)} and automatically recorded by
   * AdvantageKit.
   */
  @AutoLog
  public class EncoderIOInputs {

    /**
     * Aggregated encoder data snapshot.
     *
     * <p>All values are initialized to {@link Double#NaN} to clearly indicate unavailable or
     * uninitialized sensor data.
     */
    public EncoderIOData data = new EncoderIOData(false, Double.NaN, Double.NaN, Double.NaN);
  }

  /**
   * Immutable snapshot of encoder measurements.
   *
   * @param connected whether the encoder is connected over CAN
   * @param positionRots relative position in motor rotations
   * @param absolutePositionRots absolute position in rotations, if available
   * @param velocityRotPerSec rotational velocity in rotations per second
   */
  public record EncoderIOData(
      boolean connected,
      double positionRots,
      double absolutePositionRots,
      double velocityRotPerSec) {}

  /**
   * Reads the latest encoder values into the provided input container.
   *
   * @param inputs container to populate with current encoder data
   */
  void updateInputs(EncoderIOInputs inputs);

  /**
   * Checks whether the canbus is connected.
   *
   * @return true if connected, false otherwise
   */
  boolean isConnected();

  /**
   * Updates the sampling frequency of the encoder, if supported by the hardware.
   *
   * <p>Implementations may ignore this call if frequency configuration is not applicable.
   *
   * @param frequencyHz desired update rate in Hertz
   */
  void updateFrequency(double frequencyHz);
}

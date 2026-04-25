package com.team10043.lib.encoders;

import com.team10043.lib.encoders.config.EncoderConfig;
import com.team10043.lib.encoders.config.RelativeDIOConfig;
import edu.wpi.first.wpilibj.Encoder;

/**
 * {@link EncoderIO} implementation for a relative quadrature encoder wired via DIO channels.
 *
 * <p>This class adapts a WPILib {@link Encoder} (two-channel A/B quadrature) to the generic encoder
 * IO interface, providing relative position and velocity measurements in mechanism units.
 *
 * <p>Because relative encoders provide no absolute position reference, the {@code
 * absolutePositionRots} field in {@link EncoderIO.EncoderIOData} reports the offset-adjusted
 * position (i.e. the raw position plus the configured offset) in mechanism units. This allows
 * higher-level code to use the adjusted reading as a soft absolute reference when the mechanism is
 * pre-positioned at a known location on startup.
 *
 * <p>The {@link RelativeDIOConfig#inverted} flag and {@link RelativeDIOConfig#offset} are applied
 * before {@link EncoderConfig#sensorToMechanismRatio} scaling.
 *
 * <p>DIO encoders do not support configurable CAN bus update rates, so {@link
 * #updateFrequency(double)} is intentionally a no-op.
 *
 * <p>Connectivity is derived from WPILib's {@link Encoder#isConnected()} signal, which reflects
 * whether pulses have been received on the DIO channels.
 */
public class EncoderIORelativeDIO implements EncoderIO {

  /** Underlying WPILib quadrature encoder. */
  private final Encoder encoder;

  /** Encoder configuration describing hardware channels and scaling behavior. */
  private final EncoderConfig<RelativeDIOConfig> config;

  /**
   * Creates a DIO-based relative encoder IO implementation.
   *
   * <p>The WPILib {@link Encoder} is constructed from the A and B DIO channel numbers specified in
   * the {@link RelativeDIOConfig}. Inversion is applied directly to the hardware object so that all
   * subsequent position and rate readings already carry the correct sign.
   *
   * <p>The distance-per-pulse is set to {@code 1.0 / pulsesPerRevolution} so that {@link
   * Encoder#getDistance()} returns values in rotations. For a standard quadrature encoder WPILib
   * counts 4× the physical pulses (4-edge counting), so the effective distance per pulse is {@code
   * 1.0 / (4 * pulsesPerRevolution)}; however this class delegates unit conversion entirely to
   * {@link EncoderConfig#sensorToMechanismRatio} and therefore sets distance-per-pulse to {@code
   * 1.0} (one count = one rotation unit pre-scaling).
   *
   * @param config encoder configuration including DIO channels, offset, inversion, and scaling
   */
  public EncoderIORelativeDIO(EncoderConfig<RelativeDIOConfig> config) {
    this.config = config;

    RelativeDIOConfig dioConfig = config.config;

    this.encoder = new Encoder(dioConfig.dioChannelA, dioConfig.dioChannelB, dioConfig.inverted);

    this.encoder.setDistancePerPulse(dioConfig.distancePerPulse);
  }

  /**
   * Reads encoder data into the provided input container.
   *
   * <p>Position is reported in mechanism rotations, computed as:
   *
   * <pre>
   *   positionRots = (rawCounts * sensorToMechanismRatio) + offset
   * </pre>
   *
   * <p>Absolute position mirrors the offset-adjusted position since no true absolute reference is
   * available from a relative encoder.
   *
   * @param inputs container to populate with encoder measurements
   */
  @Override
  public void updateInputs(EncoderIOInputs inputs) {
    inputs.data =
        new EncoderIOData(
            isConnected(),
            (encoder.getDistance() * config.sensorToMechanismRatio) + config.config.offset,
            (encoder.getDistance() * config.sensorToMechanismRatio) + config.config.offset,
            encoder.getRate() * config.sensorToMechanismRatio);
  }

  /**
   * Returns {@code true} unconditionally.
   *
   * <p>DIO-based encoders are physically wired to the roboRIO and do not expose a connection-state
   * signal. If the encoder is present and the channels are valid the hardware is considered
   * permanently connected.
   *
   * @return always {@code true}
   */
  @Override
  public boolean isConnected() {
    return true;
  }

  /**
   * No-op for DIO-based encoders.
   *
   * <p>WPILib {@link Encoder} objects sample at the FPGA update rate and do not expose a
   * configurable signal frequency. This method exists solely to satisfy the {@link EncoderIO}
   * contract.
   *
   * @param frequencyHz ignored
   */
  @Override
  public void updateFrequency(double frequencyHz) {
    // DIO encoders do not support configurable update frequency — no-op.
  }
}

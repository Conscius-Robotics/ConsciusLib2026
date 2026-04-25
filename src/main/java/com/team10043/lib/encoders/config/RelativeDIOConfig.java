package com.team10043.lib.encoders.config;

import lombok.Builder;

/**
 * Encoder configuration for a relative quadrature encoder wired via DIO channels.
 *
 * <p>This configuration supports two-channel (A/B) quadrature encoders such as the REV Through Bore
 * Encoder in relative mode, connected directly to the roboRIO's DIO ports.
 *
 * <p>Because DIO-based encoders do not provide absolute position feedback, the {@code offset} field
 * is applied to the raw position reading to produce an adjusted position value. The {@code
 * inverted} flag reverses the direction of all position and velocity readings.
 *
 * <p>The {@code offset} is expressed in sensor rotations (before the {@code sensorToMechanismRatio}
 * defined in {@link EncoderConfig} is applied). It is added to the raw position reading so that a
 * known reference point can be baked in at configuration time.
 *
 * <p>Instances are created via the Lombok-generated builder:
 *
 * <pre>{@code
 * RelativeDIOConfig config = RelativeDIOConfig.builder()
 *     .dioChannelA(0)
 *     .dioChannelB(1)
 *     .offset(0.25)
 *     .inverted(false)
 *     .build();
 * }</pre>
 */
@Builder
public non-sealed class RelativeDIOConfig implements EncoderConfiguration {

  /**
   * DIO channel for the encoder's A-phase signal.
   *
   * <p>Must correspond to a valid roboRIO DIO port (0–9 for the built-in ports).
   */
  public int dioChannelA;

  /**
   * DIO channel for the encoder's B-phase signal.
   *
   * <p>Must correspond to a valid roboRIO DIO port (0–9 for the built-in ports) and must differ
   * from {@link #dioChannelA}.
   */
  public int dioChannelB;

  /**
   * Position offset in sensor rotations added to the raw encoder reading.
   *
   * <p>Use this to shift the zero-point of the relative encoder to a known mechanical reference.
   * The offset is applied before {@link EncoderConfig#sensorToMechanismRatio} scaling.
   */
  public double offset;

  /**
   * When {@code true}, inverts the sign of all position and velocity readings.
   *
   * <p>Set this to match the physical mounting direction of the encoder so that positive values
   * correspond to the intended positive direction of the mechanism.
   */
  public boolean inverted;

  /**
   * Distance per pulse in sensor units (pre-scaling).
   *
   * <p>This is typically set to {@code 1.0} so that the raw encoder reading corresponds to the
   * number of pulses counted, and unit conversion to rotations and then to mechanism units is
   * handled entirely by {@link EncoderConfig#sensorToMechanismRatio}.
   *
   * <p>For a standard quadrature encoder with 4-edge counting, the effective distance per pulse is
   * {@code 1.0 / 4} if using raw pulse counts, or {@code 1.0} if using WPILib's built-in counting
   * mode where each pulse corresponds to one count.
   */
  public double distancePerPulse;
}

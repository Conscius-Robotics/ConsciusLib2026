package com.team10043.lib.encoders.config;

import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.encoders.EncoderIO;
import com.team10043.lib.encoders.EncoderIOCANCoder;
import com.team10043.lib.encoders.EncoderIORelativeDIO;
import java.util.Optional;
import lombok.Builder;

/**
 * Container class describing how an encoder is attached and configured.
 *
 * <p>This class binds a physical CAN device (if any) to a concrete {@link EncoderConfiguration}
 * implementation and defines how sensor units are converted to mechanism units.
 *
 * <p>The generic type parameter enforces compile-time consistency between the encoder type and its
 * configuration.
 *
 * @param <T> the specific encoder configuration type
 */
@Builder
public class EncoderConfig<T extends EncoderConfiguration> {

  /**
   * CAN device identifier for the encoder (if applicable).
   *
   * <p>This is only required for CAN-based encoders (e.g., CANCoder). DIO-based encoders (e.g.,
   * Through Bore Encoder) should leave this empty.
   */
  @Builder.Default public Optional<CANDeviceId> canId = Optional.empty();

  /** Encoder-specific configuration describing the encoder type and behavior. */
  public final T config;

  /**
   * Ratio converting sensor units to mechanism units.
   *
   * <p>For example, this may include gear reduction between the encoder and the driven mechanism.
   */
  @Builder.Default public double sensorToMechanismRatio = 1.0;

  /** Desired CAN status signal update frequency in Hz. (for CTRE products only) */
  @Builder.Default public double updateFrequencyHz = 50.0;

  @Builder.Default public boolean updateMotorConstantly = false;

  public EncoderIO createIO() {
    if (config instanceof CANCoderConfig) {
      @SuppressWarnings("unchecked")
      EncoderConfig<CANCoderConfig> cancoderConfig = (EncoderConfig<CANCoderConfig>) this;
      return new EncoderIOCANCoder(cancoderConfig);
    }

    if (config instanceof RelativeDIOConfig) {
      @SuppressWarnings("unchecked")
      EncoderConfig<RelativeDIOConfig> relativeDIOConfig = (EncoderConfig<RelativeDIOConfig>) this;
      return new EncoderIORelativeDIO(relativeDIOConfig);
    }

    throw new IllegalStateException("Unsupported encoder config type");
  }
}

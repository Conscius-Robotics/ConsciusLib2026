package com.team10043.lib.gyros.config;

import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.gyros.GyroIO;
import com.team10043.lib.gyros.GyroIOPigeon2;
import lombok.Builder;

/**
 * Generic configuration container for a gyro device.
 *
 * <p>This class binds a specific {@link GyroConfiguration} implementation to a CAN device ID,
 * allowing gyro implementations to be configured in a type-safe and extensible manner.
 *
 * <p>The generic type parameter ensures that only configuration objects compatible with the
 * selected gyro hardware are used.
 *
 * @param <T> the concrete gyro configuration type
 */
@Builder
public class GyroConfig<T extends GyroConfiguration> {

  /** CAN device ID of the gyro. */
  public final CANDeviceId canId;

  /** Hardware-specific gyro configuration. */
  public final T config;

  /** Desired CAN status signal update frequency in Hz. (for CTRE products only) */
  @Builder.Default public double updateFrequencyHz = 50.0;

  public GyroIO createIO() {
    if (config instanceof Pigeon2Config) {
      @SuppressWarnings("unchecked")
      GyroConfig<Pigeon2Config> pigeonConfig = (GyroConfig<Pigeon2Config>) this;
      return new GyroIOPigeon2(pigeonConfig);
    }

    throw new IllegalStateException("Unsupported gyro config type");
  }
}

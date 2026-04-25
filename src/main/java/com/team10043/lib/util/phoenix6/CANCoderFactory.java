package com.team10043.lib.util.phoenix6;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.encoders.config.CANCoderConfig;
import lombok.Getter;

/**
 * Factory for creating CANcoder absolute position encoder configurations with fluent API.
 *
 * <p>CANcoder is a CAN-based magnetic encoder that provides absolute and relative position
 * measurements along with filtered velocity. This factory simplifies configuration with sensible
 * defaults and method chaining.
 *
 * <p><strong>Usage Pattern:</strong> This factory is designed for single-use configuration. Each
 * configuration method modifies the internal config object. For multiple encoders with different
 * configurations, create a new factory instance for each encoder.
 *
 * <p><strong>Example Usage:</strong>
 *
 * <pre>{@code
 * // Create config and then build encoder
 * CANCoderConfig config = CANCoderFactory.create().magnetOffset(0.25)
 *     .sensorDirection(SensorDirectionValue.CounterClockwise_Positive).buildConfig();
 *
 * CANcoder encoder = CANCoderFactory.createCANcoder(new CANDeviceId(5, "drivetrain"), config);
 *
 * // Or use the convenient one-liner
 * CANcoder encoder = CANCoderFactory.create().magnetOffset(0.25)
 *     .sensorDirection(SensorDirectionValue.CounterClockwise_Positive)
 *     .createCANcoder(new CANDeviceId(5, "drivetrain"));
 *
 * // Swerve module encoder with discontinuity point
 * CANCoderConfig swerveConfig =
 *     CANCoderFactory.create().magnetOffset(0.123).absoluteSensorDiscontinuity(0.75).buildConfig();
 * }</pre>
 *
 * <p><strong>Default Configuration:</strong>
 *
 * <ul>
 *   <li>Sensor Direction: CounterClockwise_Positive
 *   <li>Magnet Offset: 0.0 rotations (north pole aligns with LED)
 *   <li>Absolute Sensor Discontinuity Point: 0.5 (signed ±0.5 range)
 *   <li>Future Proof Configs: true (factory default newer unsupported configs)
 * </ul>
 *
 * <p><strong>Magnet Offset:</strong> The offset is added to the reported position to trim the zero
 * position. When set to 0, position reports zero when the magnet's north pole aligns with the LED
 * on the CANcoder.
 *
 * <p><strong>Discontinuity Point:</strong> For mechanisms with a fixed range of motion (like swerve
 * modules or arms), set this to the center of the unreachable region to avoid the sensor wrapping
 * around in the middle of valid motion.
 *
 * @see CANCoderConfig
 * @see CANcoder
 */
public class CANCoderFactory {

  /** Default sensor direction (CounterClockwise is positive) */
  public static final SensorDirectionValue SENSOR_DIRECTION =
      SensorDirectionValue.CounterClockwise_Positive;

  /** Default magnet offset in rotations (0 = north pole aligns with LED) */
  public static final double MAGNET_OFFSET = 0.0;

  /**
   * Default discontinuity point in rotations. 0.5 creates a signed range of [-0.5, 0.5) rotations
   */
  public static final double DISCONTINUITY_POINT = 0.5;

  @Getter private final CANCoderConfig config;

  /** Private constructor - use {@link #create()} instead. */
  private CANCoderFactory() {
    config = getDefaultCANCoderConfig();
  }

  /**
   * Creates a new CANcoder factory with default configuration.
   *
   * <p>This is the preferred way to create a factory instance. Use method chaining to configure the
   * encoder before calling {@link #buildConfig()} or {@link #createCANcoder(CANDeviceId)}.
   *
   * <p><strong>Example:</strong>
   *
   * <pre>{@code
   * CANCoderConfig config = CANCoderFactory.create().magnetOffset(0.123)
   *     .sensorDirection(SensorDirectionValue.Clockwise_Positive).buildConfig();
   * }</pre>
   *
   * @return a new CANCoderFactory instance ready for configuration
   */
  public static CANCoderFactory create() {
    return new CANCoderFactory();
  }

  /**
   * Sets the sensor direction to determine positive rotation.
   *
   * <p>The direction is defined as seen facing the LED side of the CANcoder:
   *
   * <ul>
   *   <li>{@link SensorDirectionValue#CounterClockwise_Positive} - CCW is positive (default)
   *   <li>{@link SensorDirectionValue#Clockwise_Positive} - CW is positive
   * </ul>
   *
   * @param direction the desired sensor direction
   * @return this factory instance for method chaining
   */
  public CANCoderFactory sensorDirection(SensorDirectionValue direction) {
    config.MagnetSensor.SensorDirection = direction;
    return this;
  }

  /**
   * Sets the magnet offset to trim the zero position.
   *
   * <p>This offset is added to the reported position. When set to the default value of zero,
   * position reports zero when the magnet's north pole aligns with the LED on the CANcoder.
   *
   * <p><strong>Usage:</strong> After mechanically aligning your mechanism to its desired zero
   * position, read the current CANcoder position and negate it to use as the offset. For example,
   * if the encoder reads 0.237 at your zero position, set the offset to -0.237.
   *
   * @param offset the magnet offset in rotations (range: -1.0 to 1.0)
   * @return this factory instance for method chaining
   */
  public CANCoderFactory magnetOffset(double offset) {
    config.MagnetSensor.MagnetOffset = offset;
    return this;
  }

  /**
   * Sets the absolute sensor discontinuity point.
   *
   * <p>This determines where the absolute sensor wraps around, keeping the absolute position in the
   * range [x-1, x). Common settings:
   *
   * <ul>
   *   <li>1.0 - Unsigned range [0, 1)
   *   <li>0.5 - Signed range [-0.5, 0.5) (default)
   *   <li>0.0 - Always negative [-1, 0)
   * </ul>
   *
   * <p><strong>For mechanisms with limited range:</strong> Set this to the center of the
   * unreachable region. For example, a swerve module that travels from -0.2 to 0.6 rotations should
   * have its discontinuity point at 0.7 (center of the 0.6 to 0.8 range), resulting in a usable
   * range of [-0.3, 0.7) rotations.
   *
   * @param point the discontinuity point in rotations (range: 0.0 to 1.0)
   * @return this factory instance for method chaining
   */
  public CANCoderFactory absoluteSensorDiscontinuity(double point) {
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = point;
    return this;
  }

  /**
   * Builds and returns the configured CANCoderConfig.
   *
   * <p>This method returns the configuration object that can be used later to create CANcoder
   * instances or for other purposes.
   *
   * @return the configured CANCoderConfig
   */
  public CANCoderConfig buildConfig() {
    return config;
  }

  /**
   * Creates a CANcoder encoder with the current configuration.
   *
   * <p>This is a convenience method that combines buildConfig() and createCANcoder() into one step.
   *
   * @param deviceId the CAN device identifier for the encoder
   * @return a fully configured CANcoder encoder ready for use
   */
  public CANcoder createCANcoder(CANDeviceId deviceId) {
    return createCANcoder(deviceId, config);
  }

  /**
   * Creates a CANcoder encoder with the specified configuration.
   *
   * <p>This static method creates a new CANcoder instance, clears sticky faults, and applies the
   * provided configuration.
   *
   * @param deviceId the CAN device identifier
   * @param config the configuration to apply
   * @return a fully configured CANcoder encoder
   */
  public static CANcoder createCANcoder(CANDeviceId deviceId, CANCoderConfig config) {
    CANcoder cancoder = new CANcoder(deviceId.getDeviceNumber(), deviceId.getCANBus());
    cancoder.clearStickyFaults();
    CTREUtil.applyConfiguration(cancoder, config);
    return cancoder;
  }

  /**
   * Creates and returns the default CANcoder configuration.
   *
   * <p>This configuration provides sensible defaults suitable for most applications:
   *
   * <ul>
   *   <li>CounterClockwise positive direction
   *   <li>Zero offset
   *   <li>Signed ±0.5 rotation range
   *   <li>Future-proof configs enabled for forward compatibility
   * </ul>
   *
   * <p>This method can be used to obtain a base configuration for manual modification or as a
   * starting point for custom factory methods.
   *
   * @return a new CANCoderConfig with default settings
   */
  public static CANCoderConfig getDefaultCANCoderConfig() {
    CANCoderConfig config = new CANCoderConfig();

    // Magnet sensor configuration
    config.MagnetSensor.SensorDirection = SENSOR_DIRECTION;
    config.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = DISCONTINUITY_POINT;

    // Future-proofing for firmware updates
    config.FutureProofConfigs = true;

    return config;
  }
}

package com.team10043.lib.util.phoenix6;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.gyros.config.Pigeon2Config;
import lombok.Getter;

/**
 * Factory for creating Pigeon2 IMU configurations with fluent API.
 *
 * <p>This factory provides a builder-style interface for configuring Pigeon2 sensors with sensible
 * defaults and method chaining for common configuration options.
 *
 * <p><strong>Usage Pattern:</strong> This factory is designed for single-use configuration. Each
 * configuration method modifies the internal config object. For multiple sensors with different
 * configurations, create a new factory instance for each sensor.
 *
 * <p><strong>Example Usage:</strong>
 *
 * <pre>{@code
 * // Create config and then build sensor
 * Pigeon2Config config = Pigeon2Factory.create().mountPose(0, 90, 0) // mounted with X-axis
 *                                                                    // pointing up
 *     .disableCompass().enableNoMotionCalibration().buildConfig();
 *
 * Pigeon2 imu = Pigeon2Factory.createPigeon(new CANDeviceId(1, "canivore"), config);
 *
 * // Or use the convenient one-liner
 * Pigeon2 imu = Pigeon2Factory.create().mountPose(0, 0, 0).disableCompass()
 *     .createPigeon(new CANDeviceId(1, "canivore"));
 *
 * // Multiple sensors with different configs
 * Pigeon2Config primaryConfig = Pigeon2Factory.create().mountPose(0, 0, 0).buildConfig();
 *
 * Pigeon2Config secondaryConfig =
 *     Pigeon2Factory.create().mountPose(0, 90, 0).disableCompass().buildConfig();
 * }</pre>
 *
 * <p><strong>Default Configuration:</strong>
 *
 * <ul>
 *   <li>Mount Pose: Yaw=0, Pitch=0, Roll=0 (flat and forward-facing)
 *   <li>Gyro Trim: X=0, Y=0, Z=0 (no trimming)
 *   <li>No-Motion Calibration: Enabled
 *   <li>Temperature Compensation: Enabled
 *   <li>Compass: Disabled (recommended for FRC use)
 * </ul>
 *
 * @see Pigeon2Config
 * @see Pigeon2
 */
public class Pigeon2Factory {

  /** Default mount pose yaw in degrees */
  public static final double DEFAULT_MOUNT_POSE_YAW = 0.0;

  /** Default mount pose pitch in degrees */
  public static final double DEFAULT_MOUNT_POSE_PITCH = 0.0;

  /** Default mount pose roll in degrees */
  public static final double DEFAULT_MOUNT_POSE_ROLL = 0.0;

  /** Default gyro trim scalar for all axes */
  public static final double DEFAULT_GYRO_TRIM = 0.0;

  /** Default no-motion calibration state (enabled) */
  public static final boolean DEFAULT_NO_MOTION_CALIBRATION_ENABLED = true;

  /** Default temperature compensation state (enabled) */
  public static final boolean DEFAULT_TEMPERATURE_COMPENSATION_ENABLED = true;

  /** Default compass state (disabled for FRC) */
  public static final boolean DEFAULT_COMPASS_ENABLED = false;

  @Getter private final Pigeon2Config config;

  /** Private constructor - use {@link #create()} instead. */
  private Pigeon2Factory() {
    config = getDefaultPigeon2Config();
  }

  /**
   * Creates a new Pigeon2 factory with default configuration.
   *
   * <p>This is the preferred way to create a factory instance. Use method chaining to configure the
   * sensor before calling {@link #buildConfig()} or {@link #createPigeon(CANDeviceId)}.
   *
   * <p><strong>Example:</strong>
   *
   * <pre>{@code
   * Pigeon2Config config =
   *     Pigeon2Factory.create().mountPose(0, 90, 0).disableCompass().buildConfig();
   * }</pre>
   *
   * @return a new Pigeon2Factory instance ready for configuration
   */
  public static Pigeon2Factory create() {
    return new Pigeon2Factory();
  }

  /**
   * Sets the mount pose orientation of the Pigeon2.
   *
   * <p>This configures the Yaw-Pitch-Roll the Pigeon2 underwent to get to its current orientation,
   * referenced from the robot's point of view. This is necessary if the Pigeon2 is mounted at an
   * exotic angle or not forward-facing.
   *
   * <p><strong>Examples:</strong>
   *
   * <ul>
   *   <li>Flat and forward: yaw=0, pitch=0, roll=0
   *   <li>Pointed directly right: yaw=-90, pitch=0, roll=0
   *   <li>Pointed upwards (X-up): yaw=0, pitch=-90, roll=0
   *   <li>Rolled 90 degrees CCW: yaw=0, pitch=0, roll=90
   * </ul>
   *
   * @param yaw yaw angle in degrees
   * @param pitch pitch angle in degrees
   * @param roll roll angle in degrees
   * @return this factory instance for method chaining
   */
  public Pigeon2Factory mountPose(double yaw, double pitch, double roll) {
    config.MountPose.MountPoseYaw = yaw;
    config.MountPose.MountPosePitch = pitch;
    config.MountPose.MountPoseRoll = roll;
    return this;
  }

  /**
   * Sets the gyroscope trim scalars for all axes.
   *
   * <p>Gyro trim allows compensation for systematic gyroscope errors. Values represent the degrees
   * of error per rotation. For example, if the gyro overshoots by 1 degree after a full rotation,
   * use scalarX=1.
   *
   * <p>Most applications do not need to trim the gyro and can leave these at 0.
   *
   * @param scalarX X-axis trim value in degrees
   * @param scalarY Y-axis trim value in degrees
   * @param scalarZ Z-axis trim value in degrees
   * @return this factory instance for method chaining
   */
  public Pigeon2Factory gyroTrim(double scalarX, double scalarY, double scalarZ) {
    config.GyroTrim.GyroScalarX = scalarX;
    config.GyroTrim.GyroScalarY = scalarY;
    config.GyroTrim.GyroScalarZ = scalarZ;
    return this;
  }

  /**
   * Enables the no-motion calibration feature.
   *
   * <p>When enabled, the Pigeon2 will automatically recalibrate the gyroscope when it detects no
   * motion for approximately 4 seconds. This helps reduce drift over time.
   *
   * <p><strong>This is enabled by default and recommended for most applications.</strong>
   *
   * @return this factory instance for method chaining
   */
  public Pigeon2Factory enableNoMotionCalibration() {
    config.Pigeon2Features.DisableNoMotionCalibration = false;
    return this;
  }

  /**
   * Disables the no-motion calibration feature.
   *
   * <p>Only disable this if your application requires continuous motion detection without automatic
   * recalibration.
   *
   * @return this factory instance for method chaining
   */
  public Pigeon2Factory disableNoMotionCalibration() {
    config.Pigeon2Features.DisableNoMotionCalibration = true;
    return this;
  }

  /**
   * Enables temperature compensation.
   *
   * <p>Temperature compensation adjusts sensor readings to account for temperature changes. This
   * improves accuracy across different operating temperatures.
   *
   * <p><strong>This is enabled by default and recommended for most applications.</strong>
   *
   * @return this factory instance for method chaining
   */
  public Pigeon2Factory enableTemperatureCompensation() {
    config.Pigeon2Features.DisableTemperatureCompensation = false;
    return this;
  }

  /**
   * Disables temperature compensation.
   *
   * <p><strong>Warning:</strong> Disabling temperature compensation may reduce accuracy in
   * environments with significant temperature variation.
   *
   * @return this factory instance for method chaining
   */
  public Pigeon2Factory disableTemperatureCompensation() {
    config.Pigeon2Features.DisableTemperatureCompensation = true;
    return this;
  }

  /**
   * Enables the magnetometer (compass) feature.
   *
   * <p><strong>Warning:</strong> Compass features are generally not recommended for FRC use due to
   * magnetic interference from motors, metal structures, and other sources. The compass may provide
   * incorrect readings in a typical robot environment.
   *
   * <p>Only enable this if you are in an outdoor setting with minimal magnetic interference and
   * have properly calibrated the magnetometer.
   *
   * @return this factory instance for method chaining
   */
  public Pigeon2Factory enableCompass() {
    config.Pigeon2Features.EnableCompass = true;
    return this;
  }

  /**
   * Disables the magnetometer (compass) feature.
   *
   * <p><strong>This is disabled by default and recommended for FRC applications.</strong>
   *
   * @return this factory instance for method chaining
   */
  public Pigeon2Factory disableCompass() {
    config.Pigeon2Features.EnableCompass = false;
    return this;
  }

  /**
   * Builds and returns the configured Pigeon2Config.
   *
   * <p>This method returns the configuration object that can be used later to create Pigeon2
   * instances or for other purposes.
   *
   * @return the configured Pigeon2Config
   */
  public Pigeon2Config buildConfig() {
    return config;
  }

  /**
   * Creates a Pigeon2 IMU sensor with the current configuration.
   *
   * <p>This is a convenience method that combines buildConfig() and createPigeon() into one step.
   *
   * @param deviceId the CAN device identifier for the IMU sensor
   * @return a fully configured Pigeon2 IMU sensor ready for use
   */
  public Pigeon2 createPigeon(CANDeviceId deviceId) {
    return createPigeon(deviceId, config);
  }

  /**
   * Creates a Pigeon2 IMU sensor with the specified configuration.
   *
   * <p>This static method creates a new Pigeon2 instance, clears sticky faults, and applies the
   * provided configuration.
   *
   * @param deviceId the CAN device identifier
   * @param config the configuration to apply
   * @return a fully configured Pigeon2 IMU sensor
   */
  public static Pigeon2 createPigeon(CANDeviceId deviceId, Pigeon2Config config) {
    Pigeon2 pigeon = new Pigeon2(deviceId.getDeviceNumber(), deviceId.getCANBus());
    pigeon.clearStickyFaults();
    CTREUtil.applyConfiguration(pigeon, config);
    return pigeon;
  }

  /**
   * Creates and returns the default Pigeon2 configuration.
   *
   * <p>This configuration provides sensible defaults suitable for most FRC applications:
   *
   * <ul>
   *   <li>Mount pose at 0° (flat and forward-facing)
   *   <li>No gyro trimming needed
   *   <li>No-motion calibration enabled
   *   <li>Temperature compensation enabled
   *   <li>Compass disabled (recommended for FRC due to magnetic interference)
   * </ul>
   *
   * <p>This method can be used to obtain a base configuration for manual modification or as a
   * starting point for custom factory methods.
   *
   * @return a new Pigeon2Config with default settings
   */
  public static Pigeon2Config getDefaultPigeon2Config() {
    Pigeon2Config config = new Pigeon2Config();

    // Mount pose configuration - flat and forward
    config.MountPose.MountPoseYaw = DEFAULT_MOUNT_POSE_YAW;
    config.MountPose.MountPosePitch = DEFAULT_MOUNT_POSE_PITCH;
    config.MountPose.MountPoseRoll = DEFAULT_MOUNT_POSE_ROLL;

    // Gyro trim configuration - no trimming by default
    config.GyroTrim.GyroScalarX = DEFAULT_GYRO_TRIM;
    config.GyroTrim.GyroScalarY = DEFAULT_GYRO_TRIM;
    config.GyroTrim.GyroScalarZ = DEFAULT_GYRO_TRIM;

    // Pigeon2 features configuration
    config.Pigeon2Features.DisableNoMotionCalibration = !DEFAULT_NO_MOTION_CALIBRATION_ENABLED;
    config.Pigeon2Features.DisableTemperatureCompensation =
        !DEFAULT_TEMPERATURE_COMPENSATION_ENABLED;
    config.Pigeon2Features.EnableCompass = DEFAULT_COMPASS_ENABLED;

    return config;
  }
}

package com.team10043.lib.subsystems.swerve.config.system;

import com.team10043.lib.gyros.config.GyroConfiguration;

/**
 * Immutable container for swerve drive gyro configuration.
 *
 * <p>Stores the CAN ID and specific configuration parameters for the gyro used in the swerve drive
 * subsystem. This allows for flexible gyro integration and easy swapping of gyro types if needed.
 *
 * @param gyroCanId the CAN ID of the gyro sensor
 * @param gyroConfig the specific configuration parameters for the gyro
 * @param <T> the type of gyro configuration
 */
public record SwerveGyroConfig<T extends GyroConfiguration>(int gyroCanId, T gyroConfig) {}

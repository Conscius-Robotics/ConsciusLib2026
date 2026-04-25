package com.team10043.lib.gyros.config;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

/**
 * Gyro configuration implementation for the CTRE Pigeon 2.
 *
 * <p>This class directly extends {@link Pigeon2Configuration} from Phoenix 6, allowing native CTRE
 * configuration fields to be used without additional wrappers.
 *
 * <p>Being {@code non-sealed} allows future specialization or extension if team-specific defaults
 * or presets are required.
 */
public non-sealed class Pigeon2Config extends Pigeon2Configuration implements GyroConfiguration {}

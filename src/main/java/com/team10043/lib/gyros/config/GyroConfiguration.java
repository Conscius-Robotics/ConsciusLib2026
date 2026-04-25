package com.team10043.lib.gyros.config;

/**
 * Marker interface for gyro hardware configuration types.
 *
 * <p>This sealed interface defines the set of supported gyro configurations and enables
 * compile-time enforcement of valid gyro hardware options.
 *
 * <p>Each permitted implementation represents a concrete gyro model and encapsulates its
 * vendor-specific configuration parameters.
 */
public sealed interface GyroConfiguration permits Pigeon2Config {}

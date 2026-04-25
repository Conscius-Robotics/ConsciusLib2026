package com.team10043.lib.encoders.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

/**
 * Encoder configuration for a CTRE CANcoder device.
 *
 * <p>This class directly extends {@link CANcoderConfiguration}, allowing native Phoenix 6
 * configuration parameters to be used while also participating in the {@link EncoderConfiguration}
 * type hierarchy.
 *
 * <p>This enables seamless integration of CANcoder-specific settings into the generic encoder
 * configuration system.
 */
public non-sealed class CANCoderConfig extends CANcoderConfiguration
    implements EncoderConfiguration {}

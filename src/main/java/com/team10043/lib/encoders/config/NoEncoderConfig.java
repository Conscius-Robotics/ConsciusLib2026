package com.team10043.lib.encoders.config;

/**
 * Encoder configuration indicating that no encoder is present.
 *
 * <p>This class exists purely as an explicit signal to the programmer and to higher-level logic
 * that the motor or mechanism operates without encoder feedback.
 */
public non-sealed class NoEncoderConfig implements EncoderConfiguration {}

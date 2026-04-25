package com.team10043.lib.encoders.config;

/**
 * Encoder configuration indicating the use of a remote encoder.
 *
 * <p>This typically represents an encoder physically attached to another device (e.g. a drivetrain
 * module encoder shared between motors).
 *
 * <p>No hardware-specific configuration is stored here; the presence of this type communicates
 * intent to consuming code.
 */
public non-sealed class RemoteEncoderConfig implements EncoderConfiguration {}

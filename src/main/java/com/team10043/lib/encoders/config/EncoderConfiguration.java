package com.team10043.lib.encoders.config;

/**
 * Marker interface defining supported encoder configuration types.
 *
 * <p>This sealed interface restricts encoder configurations to a known, controlled set of
 * implementations, enabling explicit handling of:
 *
 * <ul>
 *   <li>No encoder present
 *   <li>Remote encoder attached to another device
 *   <li>Dedicated CAN-based encoder (e.g. CANcoder)
 *   <li>REV Through Bore Encoder via DIO
 * </ul>
 *
 * <p>This design allows higher-level code to switch on encoder intent rather than relying on null
 * checks or flags.
 */
public sealed interface EncoderConfiguration
    permits NoEncoderConfig, RemoteEncoderConfig, CANCoderConfig, RelativeDIOConfig {}

package com.team10043.lib.motors.config;

/**
 * Marker interface defining supported motor controller configuration types.
 *
 * <p>This sealed interface restricts motor configurations to known controller families, allowing
 * higher-level logic to remain controller-agnostic while preserving type safety.
 */
public sealed interface MotorConfiguration permits TalonFXMotorConfig, SparkMaxMotorConfig {}

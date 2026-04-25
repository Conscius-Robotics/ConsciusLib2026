package com.team10043.lib.motors.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

/**
 * Motor configuration for CTRE TalonFX controllers.
 *
 * <p>This class directly extends {@link TalonFXConfiguration}, allowing Phoenix 6 configuration
 * parameters to be used while participating in the generic {@link MotorConfiguration} type
 * hierarchy.
 */
public non-sealed class TalonFXMotorConfig extends TalonFXConfiguration
    implements MotorConfiguration {}

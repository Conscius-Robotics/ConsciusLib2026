package com.team10043.lib.motors.config;

import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Motor configuration for REV Spark MAX controllers.
 *
 * <p>This class directly extends {@link SparkMaxConfig}, allowing native REV configuration
 * parameters to be used while participating in the generic {@link MotorConfiguration} type
 * hierarchy.
 */
public non-sealed class SparkMaxMotorConfig extends SparkMaxConfig implements MotorConfiguration {}

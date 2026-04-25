package com.team10043.lib.subsystems.motor;

import com.team10043.lib.encoders.config.EncoderConfig;
import com.team10043.lib.encoders.config.EncoderConfiguration;
import com.team10043.lib.motors.config.MotorConfig;
import com.team10043.lib.motors.config.MotorConfiguration;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;
import lombok.Builder;
import lombok.Getter;
import lombok.Singular;

/**
 * Configuration container for a motor-driven subsystem.
 *
 * <p>
 * Example usage:
 *
 * <pre>{@code
 * MotorSubsystemConfig.builder()
 *     .systemName("Elevator")
 *     .motorConfig(primaryMotorConfig)
 *     .follower(FollowerConfig.of(motor1)...)
 *     .build();
 * }</pre>
 */
@Builder
@Getter
public class MotorSubsystemConfig<T extends MotorConfiguration, U extends EncoderConfiguration> {

  @Builder.Default
  public String systemName = "MotorSubsystem";

  public final MotorConfig<T> motorConfig;

  public final EncoderConfig<U> encoderConfig;

  @Singular
  public final List<FollowerConfig<T>> followers;

  @Builder.Default
  public double positionToleranceRots = 0.005;

  @Builder.Default
  public double velocityToleranceRotPerSec = 0.5;

  @Builder.Default
  public SysIdConfig sysIdConfig = new SysIdConfig(
      Units.Volts.of(0.25).per(Units.Second), Units.Volts.of(5.0), Units.Seconds.of(10));

  /**
   * Configuration describing a follower motor.
   *
   * <p>
   * Follower motors reuse the same encoder and mechanism configuration as the
   * leader, differing
   * only in inversion and CAN ID.
   */
  @Builder
  @Getter
  public static class FollowerConfig<T extends MotorConfiguration> {

    /** Motor configuration for the follower device. */
    public final MotorConfig<T> motor;

    /** Whether the follower motor output is inverted relative to the leader. */
    public final boolean inverted;

    public static <T extends MotorConfiguration> FollowerConfig<T> of(MotorConfig<T> motor) {
      return FollowerConfig.<T>builder().motor(motor).inverted(false).build();
    }

    public static <T extends MotorConfiguration> FollowerConfig<T> withInversion(
        MotorConfig<T> motor) {
      return FollowerConfig.<T>builder().motor(motor).inverted(true).build();
    }
  }

  @Builder
  // Record'u generic sınıfın tip parametrelerinden tamamen bağımsız hale getirdik
  public static record SysIdConfig(
      Velocity<VoltageUnit> rampRate,
      Voltage stepVoltage,
      Time timeout) {
  }
}

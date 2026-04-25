package com.team10043.lib.util.revlib;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.motors.config.SparkMaxMotorConfig;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;

/**
 *
 *
 * <h1>SparkMaxFactory</h1>
 *
 * *
 *
 * <p>A robust factory for generating {@link SparkMax} motor configurations using a fluent API. This
 * utility is fully updated for <b>REVLib 2025</b>, providing deep integration for MAXMotion,
 * absolute encoders, and complex closed-loop control. *
 *
 * <h3>Design Goals:</h3>
 *
 * <ul>
 *   <li><b>Readability:</b> Chainable methods allow for clear and expressive motor setup.
 *   <li><b>Reliability:</b> Default values are biased toward hardware safety (e.g., Brake mode, 40A
 *       limit).
 *   <li><b>Maintainability:</b> Centralizes complex REVLib configuration logic into a single
 *       builder.
 * </ul>
 *
 * * @author Team 10043
 *
 * @version 2025.1.0
 */
public class SparkMaxFactory {

  /** Default idle mode: {@link IdleMode#kBrake} to ensure safety and prevent drift. */
  public static final IdleMode IDLE_MODE = IdleMode.kBrake;

  /** Default motor inversion: {@code false} (standard clockwise rotation). */
  public static final boolean INVERTED_VALUE = false;

  /** Default Smart Current Limit: 40 Amps (conservative for NEO/NEO Vortex motors). */
  public static final int SMART_CURRENT_LIMIT = 40;

  /** Pre-config reset mode: Wipes existing parameters to ensure a clean state. */
  public static final ResetMode RESET_MODE = ResetMode.kResetSafeParameters;

  /** Persistence mode: Saves parameters to Flash memory to survive power cycles. */
  public static final PersistMode PERSIST_MODE = PersistMode.kPersistParameters;

  /** Default voltage compensation: 12.0V to ensure consistent performance. */
  public static final double VOLTAGE_COMPENSATION = 12.0;

  @Getter private final SparkMaxMotorConfig config;

  private SparkMaxFactory() {
    config = getDefaultSparkMaxConfig();
  }

  /**
   * Entry point for the fluent API.
   *
   * @return A new factory instance.
   */
  public static SparkMaxFactory create() {
    return new SparkMaxFactory();
  }

  // =========================================================================
  // 1. Core Motor Configuration
  // =========================================================================

  /**
   * Defines the motor's behavior when the neutral signal is applied.
   *
   * @param mode {@link IdleMode#kBrake} or {@link IdleMode#kCoast}.
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory idleMode(IdleMode mode) {
    config.idleMode(mode);
    return this;
  }

  /**
   * Sets whether the motor output is inverted.
   *
   * @param inverted True for inverted, false for standard.
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory inverted(boolean inverted) {
    config.inverted(inverted);
    return this;
  }

  /**
   * Configures the Smart Current Limit to prevent motor overheating and brownouts.
   *
   * @param amps Maximum allowable current in Amperes.
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory smartCurrentLimit(int amps) {
    config.smartCurrentLimit(amps);
    return this;
  }

  /**
   * Sets the Secondary (Hard) Current Limit. This acts as an instantaneous cutoff.
   *
   * @param amps Hard limit in Amperes.
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory secondaryCurrentLimit(double amps) {
    config.secondaryCurrentLimit(amps);
    return this;
  }

  /**
   * Enables voltage compensation to ensure consistent performance across varying battery levels.
   *
   * @param nominalVoltage Target voltage (standard is 12.0V).
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory voltageCompensation(double nominalVoltage) {
    config.voltageCompensation(nominalVoltage);
    return this;
  }

  /**
   * Sets the ramp rate for open loop control (seconds from 0 to full throttle).
   *
   * @param rampRate Time in seconds.
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory openLoopRampRate(double rampRate) {
    config.openLoopRampRate(rampRate);
    return this;
  }

  /**
   * Sets the ramp rate for closed loop control.
   *
   * @param rampRate Time in seconds.
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory closedLoopRampRate(double rampRate) {
    config.closedLoopRampRate(rampRate);
    return this;
  }

  // =========================================================================
  // 2. Relative Encoder Configuration
  // =========================================================================

  /**
   * Configures the scale factor for position measurements.
   *
   * @param factor Units per rotation (e.g., wheel circumference for meters).
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory positionConversionFactor(double factor) {
    config.encoder.positionConversionFactor(factor);
    return this;
  }

  /**
   * Configures the scale factor for velocity measurements.
   *
   * @param factor Units per RPM (e.g., (wheel circumference / 60) for m/s).
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory velocityConversionFactor(double factor) {
    config.encoder.velocityConversionFactor(factor);
    return this;
  }

  /**
   * Sets the encoder direction inversion.
   *
   * @param inverted whether the encoder is inverted
   * @return this factory instance
   */
  public SparkMaxFactory encoderInverted(boolean inverted) {
    config.encoder.inverted(inverted);
    return this;
  }

  /**
   * Sets the number of samples used for velocity averaging.
   *
   * @param depth Must be a power of 2 (1, 2, 4, 8, 16, 32, 64). Default is 64.
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory encoderAverageDepth(int depth) {
    config.encoder.quadratureAverageDepth(depth);
    return this;
  }

  /**
   * Defines pulses per revolution for brushed motors with external encoders. <b>Note:</b> NEO
   * internal encoders are fixed at 42 CPR.
   */
  public SparkMaxFactory encoderCountsPerRevolution(int cpr) {
    config.encoder.countsPerRevolution(cpr);
    return this;
  }

  // =========================================================================
  // 5. Soft Limits
  // =========================================================================

  /**
   * Configures the forward soft limit.
   *
   * @param limit soft limit value
   * @param enable whether the limit is enabled
   * @return this factory instance
   */
  public SparkMaxFactory forwardSoftLimit(double limit, boolean enable) {
    config.softLimit.forwardSoftLimit(limit);
    config.softLimit.forwardSoftLimitEnabled(enable);
    return this;
  }

  /**
   * Configures the reverse soft limit.
   *
   * @param limit soft limit value
   * @param enable whether the limit is enabled
   * @return this factory instance
   */
  public SparkMaxFactory reverseSoftLimit(double limit, boolean enable) {
    config.softLimit.reverseSoftLimit(limit);
    config.softLimit.reverseSoftLimitEnabled(enable);
    return this;
  }

  // =========================================================================
  // 6. Hard Limits (Limit Switches)
  // =========================================================================

  /**
   * Configures the forward limit switch.
   *
   * @param type limit switch type
   * @param behavior trigger behavior
   * @return this factory instance
   */
  public SparkMaxFactory forwardLimitSwitch(Type type, Behavior behavior) {
    config.limitSwitch.forwardLimitSwitchType(type);
    config.limitSwitch.forwardLimitSwitchTriggerBehavior(behavior);
    return this;
  }

  /**
   * Configures the reverse limit switch.
   *
   * @param type limit switch type
   * @param behavior trigger behavior
   * @return this factory instance
   */
  public SparkMaxFactory reverseLimitSwitch(Type type, Behavior behavior) {
    config.limitSwitch.reverseLimitSwitchType(type);
    config.limitSwitch.reverseLimitSwitchTriggerBehavior(behavior);
    return this;
  }

  // =========================================================================
  // 7. Closed Loop & MAXMotion (Smart Motion)
  // =========================================================================

  /**
   * Assigns PIDF gains for the specified control slot.
   *
   * @param kP Proportional gain.
   * @param kI Integral gain.
   * @param kD Derivative gain.
   * @param slot Slot index (0-3).
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory pid(double kP, double kI, double kD, int slot) {
    config.closedLoop.pid(kP, kI, kD, REVUtil.getSlotByIndex(slot));
    return this;
  }

  /**
   * Sets the integral zone (I-Zone) for the given PID slot.
   *
   * @param iZone integral zone threshold
   * @param slot PID slot index
   * @return this factory instance
   */
  public SparkMaxFactory pidIZone(double iZone, int slot) {
    config.closedLoop.iZone(iZone, REVUtil.getSlotByIndex(slot));
    return this;
  }

  /**
   * Sets the output limits for the given PID slot.
   *
   * @param min minimum output
   * @param max maximum output
   * @param slot PID slot index
   * @return this factory instance
   */
  public SparkMaxFactory pidOutputRange(double min, double max, int slot) {
    config.closedLoop.outputRange(min, max, REVUtil.getSlotByIndex(slot));
    return this;
  }

  /**
   * Enables Position PID Wrapping to allow the controller to take the shortest path.
   *
   * @param minInput Range floor (e.g., 0).
   * @param maxInput Range ceiling (e.g., 360).
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory positionPIDWrapping(double minInput, double maxInput) {
    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingInputRange(minInput, maxInput);
    return this;
  }

  /**
   * Configures feedforward gains using a static gravity term (kG).
   *
   * <p>Intended for elevator or linear mechanisms where gravity compensation is constant and
   * independent of position.
   *
   * <p>Any configured position conversion factor is applied normally, but kCos and kCosRatio are
   * not used in this mode.
   *
   * <p>Do not use this method together with cosine-based gravity compensation (kCos).
   *
   * @param kS the static friction feedforward gain
   * @param kV the velocity feedforward gain
   * @param kA the acceleration feedforward gain
   * @param kG the static gravity feedforward gain in volts
   * @param slot the closed-loop slot to apply the configuration to
   * @return this factory instance for method chaining
   */
  public SparkMaxFactory feedforward(double kS, double kV, double kA, double kG, int slot) {
    return feedforward(kS, kV, kA, kG, 0.0, 0.0, slot);
  }

  /**
   * Configures feedforward gains using cosine-based gravity compensation (kCos).
   *
   * <p>Intended for arm or rotary mechanisms where gravity varies with the cosine of the mechanism
   * angle.
   *
   * <p>The configured position conversion factor is applied first to convert sensor units into
   * mechanism units. The kCosRatio is then applied to convert those units into absolute mechanism
   * rotations used for the cosine calculation.
   *
   * <p>The encoder must be zeroed such that 0 represents a horizontal arm position.
   *
   * <p>Do not use this method together with static gravity compensation (kG).
   *
   * @param kS the static friction feedforward gain
   * @param kV the velocity feedforward gain
   * @param kA the acceleration feedforward gain
   * @param kCos the cosine gravity feedforward gain in volts
   * @param kCosRatio the ratio applied after the conversion factor to obtain absolute mechanism
   *     rotations
   * @param slot the closed-loop slot to apply the configuration to
   * @return this factory instance for method chaining
   */
  public SparkMaxFactory feedforward(
      double kS, double kV, double kA, double kCos, double kCosRatio, int slot) {
    return feedforward(kS, kV, kA, 0.0, kCos, kCosRatio, slot);
  }

  /**
   * Applies the complete feedforward configuration to the specified closed-loop slot.
   *
   * <p>Position-based gravity compensation follows this order:
   *
   * <ol>
   *   <li>Raw sensor position
   *   <li>Position conversion factor
   *   <li>kCosRatio
   *   <li>Cosine of the resulting absolute mechanism rotations
   * </ol>
   *
   * <p>Only one gravity compensation strategy should be active at a time: static (kG) or
   * cosine-based (kCos).
   *
   * @param kS the static friction feedforward gain
   * @param kV the velocity feedforward gain
   * @param kA the acceleration feedforward gain
   * @param kG the static gravity feedforward gain (elevator mechanisms)
   * @param kCos the cosine gravity feedforward gain (arm mechanisms)
   * @param kCosRatio the ratio applied after the conversion factor to obtain absolute mechanism
   *     rotations
   * @param slot the closed-loop slot to apply the configuration to
   * @return this factory instance for method chaining
   */
  private SparkMaxFactory feedforward(
      double kS, double kV, double kA, double kG, double kCos, double kCosRatio, int slot) {
    ClosedLoopSlot closedLoopSlot = REVUtil.getSlotByIndex(slot);

    FeedForwardConfig feedForward =
        new FeedForwardConfig()
            .kS(kS, closedLoopSlot)
            .kV(kV, closedLoopSlot)
            .kA(kA, closedLoopSlot);

    if (kG != 0.0 && kCos != 0.0) {
      DriverStation.reportError(
          "[REV PID, SparkMaxFactory] kG and kCos cannot be used together. Both were non-zero -> kCos won't be used.\n"
              + "Use kG for linear mechanisms, kCos (+kCosRatio) for arms. \\n Values:"
              + "kG "
              + kG
              + " kCos "
              + kCos,
          false);

      kCos = 0.0;
    }

    if (kG != 0.0) {
      feedForward.kG(kG, closedLoopSlot);
    } else if (kCos != 0.0) {
      feedForward.kCos(kCos, closedLoopSlot).kCosRatio(kCosRatio, closedLoopSlot);
    }

    config.closedLoop.feedForward.apply(feedForward);
    return this;
  }

  /**
   * Selects the feedback device used for closed-loop calculations.
   *
   * @param sensor {@link FeedbackSensor} (e.g., kPrimaryEncoder, kAbsoluteEncoder).
   * @return The factory instance for chaining.
   */
  public SparkMaxFactory feedbackSensor(FeedbackSensor sensor) {
    config.closedLoop.feedbackSensor(sensor);
    return this;
  }

  /**
   * Sets MAXMotion cruise velocity.
   *
   * @param cruiseVelocity cruise velocity in user units
   * @param slot control slot index
   * @return this factory instance
   */
  public SparkMaxFactory smartMotionCruiseVelocity(double cruiseVelocity, int slot) {
    config.closedLoop.maxMotion.cruiseVelocity(cruiseVelocity, REVUtil.getSlotByIndex(slot));
    return this;
  }

  /**
   * Sets MAXMotion maximum acceleration.
   *
   * @param maxAccel max acceleration in user units
   * @param slot control slot index
   * @return this factory instance
   */
  public SparkMaxFactory smartMotionMaxAcceleration(double maxAccel, int slot) {
    config.closedLoop.maxMotion.maxAcceleration(maxAccel, REVUtil.getSlotByIndex(slot));
    return this;
  }

  /**
   * Sets MAXMotion allowed closed-loop error.
   *
   * @param allowedError allowed closed-loop error
   * @param slot control slot index
   * @return this factory instance
   */
  public SparkMaxFactory smartMotionAllowedClosedLoopError(double allowedError, int slot) {
    config.closedLoop.maxMotion.allowedProfileError(allowedError, REVUtil.getSlotByIndex(slot));
    return this;
  }

  // =========================================================================
  // 8. Follower & Object Creation
  // =========================================================================
  /**
   * Configures this motor to follow a leader SPARK MAX.
   *
   * @param leaderCanId leader CAN ID
   * @param invertFromLeader whether to invert output from the leader
   * @return this factory instance
   */
  public SparkMaxFactory follow(int leaderCanId, boolean invertFromLeader) {
    config.follow(leaderCanId, invertFromLeader);
    return this;
  }

  /**
   * Configures this motor to follow a leader SPARK MAX.
   *
   * @param leaderCanId leader CAN ID
   * @return this factory instance
   */
  public SparkMaxFactory follow(int leaderCanId) {
    return follow(leaderCanId, false);
  }

  /**
   * Builds and returns the motor configuration.
   *
   * @return the configured SparkMaxMotorConfig
   */
  public SparkMaxMotorConfig buildConfig() {
    return config;
  }

  /** Creates and initializes a SparkMax instance with standard reset/persist modes. */
  public SparkMax createSpark(CANDeviceId deviceId, MotorType motorType) {
    return createSpark(deviceId, motorType, config, RESET_MODE, PERSIST_MODE);
  }

  /** Creates a SparkMax instance without performing a factory reset or burning to flash. */
  public SparkMax createSparkWithoutPersist(CANDeviceId deviceId, MotorType motorType) {
    return createSpark(
        deviceId,
        motorType,
        config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public static SparkMax createSpark(
      CANDeviceId deviceId, MotorType motorType, SparkMaxMotorConfig config) {
    return createSpark(deviceId, motorType, config, RESET_MODE, PERSIST_MODE);
  }

  /** Static helper to instantiate and apply configuration to a SparkMax. */
  public static SparkMax createSpark(
      CANDeviceId deviceId,
      MotorType motorType,
      SparkMaxMotorConfig config,
      ResetMode resetMode,
      PersistMode persistMode) {
    SparkMax spark = new SparkMax(deviceId.getDeviceNumber(), motorType);
    REVUtil.applyConfiguration(spark, config, resetMode, persistMode);
    return spark;
  }

  /**
   * Provides a base configuration with industry-standard defaults.
   *
   * @return A default {@link SparkMaxMotorConfig}.
   */
  public static SparkMaxMotorConfig getDefaultSparkMaxConfig() {
    SparkMaxMotorConfig config = new SparkMaxMotorConfig();
    config.idleMode(IDLE_MODE);
    config.inverted(INVERTED_VALUE);
    config.smartCurrentLimit(SMART_CURRENT_LIMIT);
    config.voltageCompensation(VOLTAGE_COMPENSATION);
    config.openLoopRampRate(0);
    config.closedLoopRampRate(0);
    config.encoder.positionConversionFactor(1.0);
    config.encoder.velocityConversionFactor(1.0);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.softLimit.forwardSoftLimitEnabled(false);
    config.softLimit.reverseSoftLimitEnabled(false);
    return config;
  }
}

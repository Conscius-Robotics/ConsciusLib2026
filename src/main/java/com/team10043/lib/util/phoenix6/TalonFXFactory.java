package com.team10043.lib.util.phoenix6;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.motors.config.TalonFXMotorConfig;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;

/**
 * Factory for creating TalonFX motor configurations with fluent API.
 *
 * <p>This factory provides a builder-style interface for configuring TalonFX motors with sensible
 * defaults and method chaining for common configuration options.
 *
 * <p><strong>Usage Pattern:</strong> This factory is designed for single-use configuration. Each
 * configuration method modifies the internal config object. For multiple motors with different
 * configurations, create a new factory instance for each motor.
 *
 * <p><strong>Example Usage:</strong>
 *
 * <pre>{@code
 * // Create config and then build motor
 * TalonFXMotorConfig config = TalonFXFactory.create().supplyCurrentLimit(50)
 *     .inverted(InvertedValue.Clockwise_Positive).neutralMode(NeutralModeValue.Brake)
 *     .pid(0.1, 0.001, 1.0, 0).feedbackSource(FeedbackSensorSourceValue.FusedCANcoder)
 *     .sensorToMechanismRatio(12.8).buildConfig();
 *
 * TalonFX motor = TalonFXFactory.createTalon(new CANDeviceId(1, ""), config);
 *
 * // Or use the convenient one-liner
 * TalonFX motor = TalonFXFactory.create().supplyCurrentLimit(40)
 *     .neutralMode(NeutralModeValue.Brake).createTalon(new CANDeviceId(1, ""));
 * }</pre>
 *
 * <p><strong>Default Configuration:</strong>
 *
 * <ul>
 *   <li>Neutral Mode: Brake
 *   <li>Invert: CounterClockwise_Positive
 *   <li>Supply Current Limit: 35A (enabled)
 *   <li>Stator Current Limit: Disabled
 *   <li>Neutral Deadband: 0.04 (4%)
 *   <li>Software Limits: Disabled
 *   <li>Hardware Limits: Disabled
 *   <li>Feedback: RotorSensor, 1:1 ratio
 *   <li>Open Loop Ramp: Disabled
 *   <li>Closed Loop Ramp: Disabled
 * </ul>
 *
 * @see TalonFXMotorConfig
 * @see TalonFX
 */
public class TalonFXFactory {

  /** Default neutral mode for motor controllers (Brake) */
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

  /** Default motor inversion setting (CounterClockwise is positive) */
  public static final InvertedValue INVERT_VALUE = InvertedValue.CounterClockwise_Positive;

  /** Default neutral deadband threshold (4%) */
  public static final double NEUTRAL_DEADBAND = 0.04;

  /** Default supply current limit in amps */
  public static final double SUPPLY_CURRENT_LIMIT = 39.0;

  /** Default stator current limit in amps */
  public static final double STATOR_CURRENT_LIMIT = 120.0;

  @Getter private TalonFXMotorConfig config;

  /** Private constructor - use {@link #create()} instead. */
  private TalonFXFactory() {
    config = new TalonFXMotorConfig();
  }

  private TalonFXFactory(TalonFXMotorConfig config) {
    this.config = config;
  }

  /**
   * Creates a new TalonFX factory with default configuration.
   *
   * <p>This is the preferred way to create a factory instance. Use method chaining to configure the
   * motor before calling {@link #buildConfig()} or {@link #createTalon(CANDeviceId)}.
   *
   * <p><strong>Example:</strong>
   *
   * <pre>{@code
   * TalonFXMotorConfig config = TalonFXFactory.create().supplyCurrentLimit(50)
   *     .neutralMode(NeutralModeValue.Brake).buildConfig();
   * }</pre>
   *
   * @return a new TalonFXFactory instance ready for configuration
   */
  public static TalonFXFactory create() {
    TalonFXFactory factory = new TalonFXFactory(getDefaultTalonFXConfig());
    return factory;
  }

  /**
   * Creates a TalonFX motor controller with the current configuration.
   *
   * <p>This is a convenience method that combines buildConfig() and createTalon() into one step.
   *
   * @param deviceId the CAN device identifier for the motor controller
   * @return a fully configured TalonFX motor controller ready for use
   */
  public TalonFX createTalon(CANDeviceId deviceId) {
    return createTalon(deviceId, config);
  }

  /**
   * Creates a TalonFX motor controller with the specified configuration.
   *
   * <p>This static method creates a new TalonFX instance, clears sticky faults, and applies the
   * provided configuration.
   *
   * @param deviceId the CAN device identifier
   * @param config the configuration to apply
   * @return a fully configured TalonFX motor controller
   */
  public static TalonFX createTalon(CANDeviceId deviceId, TalonFXMotorConfig config) {
    TalonFX talon = new TalonFX(deviceId.getDeviceNumber(), deviceId.getCANBus());
    talon.clearStickyFaults();
    CTREUtil.applyConfiguration(talon, config);
    return talon;
  }

  // ========================================
  // MOTOR OUTPUT CONFIGURATION
  // ========================================

  /**
   * Sets the neutral mode behavior of the motor.
   *
   * <p>Neutral mode determines how the motor behaves when no output is applied:
   *
   * <ul>
   *   <li>{@link NeutralModeValue#Coast} - Motor freewheels
   *   <li>{@link NeutralModeValue#Brake} - Motor actively resists movement
   * </ul>
   *
   * @param mode the desired neutral mode
   * @return this factory instance for method chaining
   */
  public TalonFXFactory neutralMode(NeutralModeValue mode) {
    config.MotorOutput.NeutralMode = mode;
    return this;
  }

  /**
   * Sets the motor direction inversion.
   *
   * <p>Controls which direction is considered "positive":
   *
   * <ul>
   *   <li>{@link InvertedValue#CounterClockwise_Positive} - CCW is positive (default)
   *   <li>{@link InvertedValue#Clockwise_Positive} - CW is positive
   * </ul>
   *
   * @param inverted the desired inversion setting
   * @return this factory instance for method chaining
   */
  public TalonFXFactory inverted(InvertedValue inverted) {
    config.MotorOutput.Inverted = inverted;
    return this;
  }

  /**
   * Sets the neutral deadband for duty cycle control.
   *
   * <p>The minimum output below which the motor will be considered at neutral. This helps eliminate
   * motor hum and unnecessary power draw at very low outputs.
   *
   * @param deadband the deadband value (0.0 to 0.25, typically 0.01 to 0.10)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory neutralDeadband(double deadband) {
    config.MotorOutput.DutyCycleNeutralDeadband = deadband;
    return this;
  }

  /**
   * Sets the peak forward and reverse duty cycle limits.
   *
   * <p>Limits the maximum output power in both directions. Useful for reducing maximum speed or
   * protecting mechanisms.
   *
   * @param forward peak forward duty cycle (0.0 to 1.0)
   * @param reverse peak reverse duty cycle (-1.0 to 0.0)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory peakOutput(double forward, double reverse) {
    config.MotorOutput.PeakForwardDutyCycle = forward;
    config.MotorOutput.PeakReverseDutyCycle = reverse;
    return this;
  }

  // ========================================
  // CURRENT LIMITING
  // ========================================

  /**
   * Sets the supply current limit and enables it.
   *
   * <p>Supply current is the current drawn from the battery/power supply. This limit helps prevent
   * brownouts and protects the electrical system.
   *
   * @param amps the current limit in amperes (typical range: 20-80A)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory supplyCurrentLimit(double amps) {
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = amps;
    return this;
  }

  /**
   * Sets the stator current limit and enables it.
   *
   * <p>Stator current is the current through the motor windings. Limiting this helps protect the
   * motor from overheating and mechanical damage.
   *
   * @param amps the current limit in amperes
   * @return this factory instance for method chaining
   */
  public TalonFXFactory statorCurrentLimit(double amps) {
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = amps;
    return this;
  }

  /**
   * Disables the stator current limit.
   *
   * <p><strong>Warning:</strong> Disabling stator current limiting may allow the motor to draw
   * excessive current, potentially causing damage.
   *
   * @return this factory instance for method chaining
   */
  public TalonFXFactory disableStatorLimit() {
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    return this;
  }

  /**
   * Disables the supply current limit.
   *
   * <p><strong>Warning:</strong> Disabling supply current limiting may cause brownouts.
   *
   * @return this factory instance for method chaining
   */
  public TalonFXFactory disableSupplyLimit() {
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    return this;
  }

  // ========================================
  // VOLTAGE COMPENSATION
  // ========================================

  /**
   * Enables voltage compensation at the specified nominal voltage.
   *
   * <p>Voltage compensation adjusts motor output to maintain consistent behavior regardless of
   * battery voltage fluctuations. Highly recommended for consistent robot performance.
   *
   * @param nominalVoltage the nominal voltage (typically 12.0V)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory voltageCompensation(double nominalVoltage) {
    config.Voltage.PeakForwardVoltage = nominalVoltage;
    config.Voltage.PeakReverseVoltage = -nominalVoltage;
    return this;
  }

  // ========================================
  // RAMP RATES
  // ========================================

  /**
   * Sets the open-loop ramp rate.
   *
   * <p>Limits how quickly the motor output can change in open-loop control modes (e.g., duty cycle,
   * voltage). This helps prevent sudden jerks and reduces mechanical stress.
   *
   * @param rampTime time in seconds to go from neutral to full output (0 disables)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory openLoopRamp(double rampTime) {
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampTime;
    config.OpenLoopRamps.TorqueOpenLoopRampPeriod = rampTime;
    return this;
  }

  /**
   * Sets the closed-loop ramp rate.
   *
   * <p>Limits how quickly the motor output can change in closed-loop control modes (e.g., position,
   * velocity). This helps prevent sudden jerks in closed-loop control.
   *
   * @param rampTime time in seconds to go from neutral to full output (0 disables)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory closedLoopRamp(double rampTime) {
    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampTime;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampTime;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampTime;
    return this;
  }

  // ========================================
  // SOFTWARE LIMITS
  // ========================================

  /**
   * Configures software position limits.
   *
   * <p>Software limits prevent the motor from moving beyond specified positions in software. These
   * are useful for mechanisms with physical travel constraints.
   *
   * <p><strong>Note:</strong> These limits are in rotations of the mechanism (after applying
   * sensor-to-mechanism ratio).
   *
   * @param forwardEnable whether to enable the forward soft limit
   * @param forward the forward limit position in rotations
   * @param reverseEnable whether to enable the reverse soft limit
   * @param reverse the reverse limit position in rotations
   * @return this factory instance for method chaining
   */
  public TalonFXFactory softLimits(
      boolean forwardEnable, double forward, boolean reverseEnable, double reverse) {
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardEnable;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forward;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseEnable;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverse;
    return this;
  }

  // ========================================
  // HARDWARE LIMITS
  // ========================================

  /**
   * Configures forward hardware limit switch.
   *
   * <p>Hardware limit switches provide physical protection by stopping the motor when a limit is
   * reached.
   *
   * @param enable whether to enable the forward limit switch
   * @param source the source of the limit signal
   * @param type the type of limit switch (normally open or normally closed)
   * @param autosetPosition whether to automatically reset position when limit is hit
   * @param autosetValue the position value to set when limit is hit (if autoset enabled)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory forwardHardLimit(
      boolean enable,
      ForwardLimitSourceValue source,
      ForwardLimitTypeValue type,
      boolean autosetPosition,
      double autosetValue) {
    config.HardwareLimitSwitch.ForwardLimitEnable = enable;
    config.HardwareLimitSwitch.ForwardLimitSource = source;
    config.HardwareLimitSwitch.ForwardLimitType = type;
    config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = autosetPosition;
    config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = autosetValue;
    return this;
  }

  /**
   * Configures reverse hardware limit switch.
   *
   * <p>Hardware limit switches provide physical protection by stopping the motor when a limit is
   * reached.
   *
   * @param enable whether to enable the reverse limit switch
   * @param source the source of the limit signal
   * @param type the type of limit switch (normally open or normally closed)
   * @param autosetPosition whether to automatically reset position when limit is hit
   * @param autosetValue the position value to set when limit is hit (if autoset enabled)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory reverseHardLimit(
      boolean enable,
      ReverseLimitSourceValue source,
      ReverseLimitTypeValue type,
      boolean autosetPosition,
      double autosetValue) {
    config.HardwareLimitSwitch.ReverseLimitEnable = enable;
    config.HardwareLimitSwitch.ReverseLimitSource = source;
    config.HardwareLimitSwitch.ReverseLimitType = type;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = autosetPosition;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = autosetValue;
    return this;
  }

  /**
   * Enables remote limit switches from another device.
   *
   * @param forwardDeviceId the CAN ID of the device providing forward limit
   * @param reverseDeviceId the CAN ID of the device providing reverse limit
   * @return this factory instance for method chaining
   */
  public TalonFXFactory remoteLimits(int forwardDeviceId, int reverseDeviceId) {
    config.HardwareLimitSwitch.ForwardLimitEnable = true;
    config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteTalonFX;
    config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = forwardDeviceId;

    config.HardwareLimitSwitch.ReverseLimitEnable = true;
    config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteTalonFX;
    config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = reverseDeviceId;
    return this;
  }

  // ========================================
  // FEEDBACK CONFIGURATION
  // ========================================

  /**
   * Sets the feedback sensor source.
   *
   * <p>Available options:
   *
   * <ul>
   *   <li>{@link FeedbackSensorSourceValue#RotorSensor} - Internal rotor sensor (default)
   *   <li>{@link FeedbackSensorSourceValue#RemoteCANcoder} - External CANcoder
   *   <li>{@link FeedbackSensorSourceValue#FusedCANcoder} - Fusion of rotor and CANcoder
   *   <li>{@link FeedbackSensorSourceValue#SyncCANcoder} - Synchronized CANcoder
   * </ul>
   *
   * @param source the feedback sensor source
   * @return this factory instance for method chaining
   */
  public TalonFXFactory feedbackSource(FeedbackSensorSourceValue source) {
    config.Feedback.FeedbackSensorSource = source;
    return this;
  }

  /**
   * Sets the remote feedback sensor device ID (for CANcoder).
   *
   * <p>Required when using RemoteCANcoder, FusedCANcoder, or SyncCANcoder as feedback source.
   *
   * @param deviceId the CAN ID of the remote sensor
   * @return this factory instance for method chaining
   */
  public TalonFXFactory feedbackRemoteId(int deviceId) {
    config.Feedback.FeedbackRemoteSensorID = deviceId;
    return this;
  }

  /**
   * Sets the sensor-to-mechanism ratio.
   *
   * <p>This is the gear ratio between the sensor (rotor or CANcoder) and the mechanism output. For
   * example, if there's a 10:1 reduction between motor and output, set this to 10.0.
   *
   * <p>After setting this ratio, all position and velocity values will be in terms of mechanism
   * rotations, not motor/sensor rotations.
   *
   * @param ratio the gear ratio (sensor rotations / mechanism rotations)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory sensorToMechanismRatio(double ratio) {
    config.Feedback.SensorToMechanismRatio = ratio;
    return this;
  }

  /**
   * Sets the rotor-to-sensor ratio.
   *
   * <p>This is the gear ratio between the motor rotor and an external sensor (CANcoder). Only
   * relevant when using a remote CANcoder that's geared differently from the rotor.
   *
   * @param ratio the gear ratio (rotor rotations / sensor rotations)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory rotorToSensorRatio(double ratio) {
    config.Feedback.RotorToSensorRatio = ratio;
    return this;
  }

  /**
   * Sets the feedback rotor offset.
   *
   * <p>This is the offset applied to the rotor position to align it with a reference point. Useful
   * for setting a "zero" position for mechanisms.
   *
   * @param offset the offset in rotations
   * @return this factory instance for method chaining
   */
  public TalonFXFactory feedbackRotorOffset(double offset) {
    config.Feedback.FeedbackRotorOffset = offset;
    return this;
  }

  /**
   * Configures complete feedback system.
   *
   * <p>Convenience method for setting up a feedback system with gear ratios.
   *
   * @param feedbackSource the feedback sensor source (should be a CANcoder type)
   * @param sensorId the CAN ID of the remote sensor
   * @param sensorToMechRatio gear ratio from sensor to mechanism output
   * @param rotorToSensorRatio gear ratio from rotor to sensor (typically 1.0)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory feedbackSensor(
      FeedbackSensorSourceValue feedbackSource,
      int sensorId,
      double sensorToMechRatio,
      double rotorToSensorRatio) {
    config.Feedback.FeedbackSensorSource = feedbackSource;
    config.Feedback.FeedbackRemoteSensorID = sensorId;
    config.Feedback.SensorToMechanismRatio = sensorToMechRatio;
    config.Feedback.RotorToSensorRatio = rotorToSensorRatio;
    return this;
  }

  // ========================================
  // PID AND FEEDFORWARD
  // ========================================

  /**
   * Sets PID gains for the specified TalonFX control slot.
   *
   * <p>TalonFX supports multiple PID slots (0–2). This method configures the proportional (kP),
   * integral (kI), and derivative (kD) gains for the selected slot.
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param slot PID slot index (0–2)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory pid(double kP, double kI, double kD, int slot) {
    switch (slot) {
      case 0:
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        break;
      case 1:
        config.Slot1.kP = kP;
        config.Slot1.kI = kI;
        config.Slot1.kD = kD;
        break;
      case 2:
        config.Slot2.kP = kP;
        config.Slot2.kI = kI;
        config.Slot2.kD = kD;
        break;
      default:
        break;
    }
    return this;
  }

  /**
   * Sets feedforward gains for the specified TalonFX control slot.
   *
   * <p>Configures static (kS), velocity (kV), acceleration (kA), and gravity (kG) feedforward terms
   * for the selected slot (0–2).
   *
   * @param kS static feedforward (volts)
   * @param kV velocity feedforward (volts / (rotation per second))
   * @param kA acceleration feedforward (volts / (rotation per second²))
   * @param kG gravity feedforward (volts)
   * @param slot control slot index (0–2)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory feedforward(double kS, double kV, double kA, double kG, int slot) {
    switch (slot) {
      case 0:
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;
        break;
      case 1:
        config.Slot1.kS = kS;
        config.Slot1.kV = kV;
        config.Slot1.kA = kA;
        config.Slot1.kG = kG;
        break;
      case 2:
        config.Slot2.kS = kS;
        config.Slot2.kV = kV;
        config.Slot2.kA = kA;
        config.Slot2.kG = kG;
        break;
      default:
        break;
    }
    return this;
  }

  /**
   * Sets the gravity type for feedforward compensation.
   *
   * <p>Determines when and how gravity compensation (kG) is applied:
   *
   * <ul>
   *   <li>{@link GravityTypeValue#Elevator_Static} - Constant upward force
   *   <li>{@link GravityTypeValue#Arm_Cosine} - Cosine-based for rotating arms
   * </ul>
   *
   * @param gravityType the gravity compensation type
   * @param slot control slot index (0-2)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory gravityType(GravityTypeValue gravityType, int slot) {
    switch (slot) {
      case 0:
        config.Slot0.GravityType = gravityType;
        break;
      case 1:
        config.Slot1.GravityType = gravityType;
        break;
      case 2:
        config.Slot2.GravityType = gravityType;
        break;
      default:
        break;
    }
    return this;
  }

  /**
   * Sets the static feedforward sign for the specified slot.
   *
   * <p>Determines when static feedforward (kS) is applied based on velocity:
   *
   * <ul>
   *   <li>{@link StaticFeedforwardSignValue#UseVelocitySign} - Apply based on velocity direction
   *   <li>{@link StaticFeedforwardSignValue#UseClosedLoopSign} - Apply based on control effort
   * </ul>
   *
   * @param signType the static feedforward sign type
   * @param slot control slot index (0-2)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory staticFeedforwardSign(StaticFeedforwardSignValue signType, int slot) {
    switch (slot) {
      case 0:
        config.Slot0.StaticFeedforwardSign = signType;
        break;
      case 1:
        config.Slot1.StaticFeedforwardSign = signType;
        break;
      case 2:
        config.Slot2.StaticFeedforwardSign = signType;
        break;
      default:
        break;
    }
    return this;
  }

  // ========================================
  // MOTION MAGIC
  // ========================================

  /**
   * Configures Motion Magic parameters for smooth motion profiling.
   *
   * <p>Motion Magic generates trapezoidal or S-curve motion profiles for smooth, controlled
   * movement to target positions.
   *
   * @param cruiseVelocity maximum velocity during motion (rotations/sec)
   * @param acceleration acceleration to reach cruise velocity (rotations/sec²)
   * @param jerk jerk limit for S-curve profiling (rotations/sec³, 0 for trapezoidal)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory motionMagic(double cruiseVelocity, double acceleration, double jerk) {
    config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = acceleration;
    config.MotionMagic.MotionMagicJerk = jerk;
    return this;
  }

  /**
   * Sets the Motion Magic expo parameters for dynamic profile adjustment.
   *
   * @param kV velocity-based expo factor
   * @param kA acceleration-based expo factor
   * @return this factory instance for method chaining
   */
  public TalonFXFactory motionMagicExpo(double kV, double kA) {
    config.MotionMagic.MotionMagicExpo_kV = kV;
    config.MotionMagic.MotionMagicExpo_kA = kA;
    return this;
  }

  // ========================================
  // CONTINUOUS WRAP
  // ========================================

  /**
   * Enables continuous wrap for mechanisms that rotate continuously.
   *
   * <p>When enabled, the controller will take the shortest path to the target position, wrapping
   * around at the specified limits. Useful for turrets and swerve modules.
   *
   * @param enable whether to enable continuous wrap
   * @return this factory instance for method chaining
   */
  public TalonFXFactory continuousWrap(boolean enable) {
    config.ClosedLoopGeneral.ContinuousWrap = enable;
    if (enable) {
      // For continuous wrap to work properly, software limits should typically be disabled
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    }
    return this;
  }

  // ========================================
  // AUDIO CONFIGURATION
  // ========================================

  /**
   * Sets whether the motor should beep on boot.
   *
   * @param beep true to enable boot beep
   * @return this factory instance for method chaining
   */
  public TalonFXFactory beepOnBoot(boolean beep) {
    config.Audio.BeepOnBoot = beep;
    return this;
  }

  /**
   * Sets whether the motor should beep on configuration.
   *
   * @param beep true to enable config beep
   * @return this factory instance for method chaining
   */
  public TalonFXFactory beepOnConfig(boolean beep) {
    config.Audio.BeepOnConfig = beep;
    return this;
  }

  /**
   * Configures whether the motor should allow music/chirp control.
   *
   * @param allow true to allow music control
   * @return this factory instance for method chaining
   */
  public TalonFXFactory allowMusicDurDisable(boolean allow) {
    config.Audio.AllowMusicDurDisable = allow;
    return this;
  }

  // ========================================
  // TORQUE CURRENT CONFIGURATION
  // ========================================

  /**
   * Sets the peak forward and reverse torque current.
   *
   * <p>Torque current limits control the maximum torque the motor can produce. These are
   * alternative limits to duty cycle limits, providing more precise torque control.
   *
   * @param forwardAmps peak forward torque current in amps
   * @param reverseAmps peak reverse torque current in amps (negative value)
   * @return this factory instance for method chaining
   */
  public TalonFXFactory peakTorqueCurrent(double forwardAmps, double reverseAmps) {
    if (forwardAmps < 0) {
      forwardAmps = -forwardAmps;
      DriverStation.reportWarning(
          "TalonFXFactory: peakTorqueCurrent forwardAmps should be positive. Automatically negating value.",
          false);
    }

    if (reverseAmps > 0) {
      reverseAmps = -reverseAmps;
      DriverStation.reportWarning(
          "TalonFXFactory: peakTorqueCurrent reverseAmps should be negative. Automatically negating value.",
          false);
    }

    config.TorqueCurrent.PeakForwardTorqueCurrent = forwardAmps;
    config.TorqueCurrent.PeakReverseTorqueCurrent = reverseAmps;
    return this;
  }

  /**
   * Sets the torque neutral deadband.
   *
   * @param deadband the deadband value in amps
   * @return this factory instance for method chaining
   */
  public TalonFXFactory torqueNeutralDeadband(double deadband) {
    config.TorqueCurrent.TorqueNeutralDeadband = deadband;
    return this;
  }

  // ========================================
  // DIFFERENTIAL CONFIGURATION
  // ========================================

  /**
   * Configures differential sensor sources.
   *
   * <p>Differential control allows controlling position and velocity of two outputs (like
   * left/right sides) with separate setpoints from a single controller.
   *
   * @param differentialSensorSource the source for differential sensing
   * @param differentialTalonFXSensorID the CAN ID of the remote TalonFX for differential
   * @param differentialRemoteCANcoderID the CAN ID of the remote CANcoder for differential
   * @return this factory instance for method chaining
   */
  public TalonFXFactory differentialSensors(
      DifferentialSensorSourceValue differentialSensorSource,
      int differentialTalonFXSensorID,
      int differentialRemoteCANcoderID) {
    config.DifferentialSensors.DifferentialSensorSource = differentialSensorSource;
    config.DifferentialSensors.DifferentialTalonFXSensorID = differentialTalonFXSensorID;
    config.DifferentialSensors.DifferentialRemoteSensorID = differentialRemoteCANcoderID;
    return this;
  }

  /**
   * Configures differential constants for peak outputs.
   *
   * @param peakDifferentialDutyCycle maximum differential duty cycle
   * @param peakDifferentialVoltage maximum differential voltage
   * @param peakDifferentialTorqueCurrent maximum differential torque current
   * @return this factory instance for method chaining
   */
  public TalonFXFactory differentialConstants(
      double peakDifferentialDutyCycle,
      double peakDifferentialVoltage,
      double peakDifferentialTorqueCurrent) {
    config.DifferentialConstants.PeakDifferentialDutyCycle = peakDifferentialDutyCycle;
    config.DifferentialConstants.PeakDifferentialVoltage = peakDifferentialVoltage;
    config.DifferentialConstants.PeakDifferentialTorqueCurrent = peakDifferentialTorqueCurrent;
    return this;
  }

  // ========================================
  // BUILDER METHODS
  // ========================================

  /**
   * Builds and returns the configured TalonFXMotorConfig.
   *
   * <p>This method returns the configuration object that can be used later to create TalonFX
   * instances or for other purposes.
   *
   * @return the configured TalonFXMotorConfig
   */
  public TalonFXMotorConfig buildConfig() {
    return config;
  }

  // ========================================
  // DEFAULT CONFIGURATION
  // ========================================

  /**
   * Creates and returns the default TalonFX configuration.
   *
   * <p>This configuration provides sensible defaults suitable for most applications:
   *
   * <ul>
   *   <li>Brake neutral mode for safety
   *   <li>35A supply current limit to prevent brownouts
   *   <li>No stator current limiting by default
   *   <li>All limits and special features disabled
   *   <li>Rotor sensor feedback with 1:1 ratio
   *   <li>No ramp rates applied
   *   <li>Full duty cycle range (-1.0 to 1.0)
   *   <li>4% neutral deadband
   * </ul>
   *
   * <p>This method can be used to obtain a base configuration for manual modification.
   *
   * @return a new TalonFXMotorConfig with default settings
   */
  public static TalonFXMotorConfig getDefaultTalonFXConfig() {
    return new TalonFXFactory()
        .neutralMode(NEUTRAL_MODE)
        .inverted(INVERT_VALUE)
        .neutralDeadband(NEUTRAL_DEADBAND)
        .peakOutput(1.0, -1.0)
        .supplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
        .statorCurrentLimit(STATOR_CURRENT_LIMIT)
        .voltageCompensation(12.0)
        .peakTorqueCurrent(800.0, -800.0)
        .torqueNeutralDeadband(0.0)
        .softLimits(false, 0.0, false, 0.0)
        .feedbackSource(FeedbackSensorSourceValue.RotorSensor)
        .feedbackRemoteId(0)
        .feedbackRotorOffset(0.0)
        .sensorToMechanismRatio(1.0)
        .rotorToSensorRatio(1.0)
        .forwardHardLimit(
            false,
            ForwardLimitSourceValue.LimitSwitchPin,
            ForwardLimitTypeValue.NormallyOpen,
            false,
            0.0)
        .reverseHardLimit(
            false,
            ReverseLimitSourceValue.LimitSwitchPin,
            ReverseLimitTypeValue.NormallyOpen,
            false,
            0.0)
        .openLoopRamp(0.0)
        .closedLoopRamp(0.0)
        .motionMagic(0.0, 0.0, 0.0)
        .motionMagicExpo(0.0, 0.0)
        .continuousWrap(false)
        .differentialSensors(DifferentialSensorSourceValue.Disabled, 0, 0)
        .differentialConstants(2.0, 32.0, 1600.0)
        .beepOnBoot(true)
        .beepOnConfig(true)
        .allowMusicDurDisable(false)
        .pid(0.0, 0.0, 0.0, 0)
        .feedforward(0.0, 0.0, 0.0, 0.0, 0)
        .gravityType(GravityTypeValue.Elevator_Static, 0)
        .staticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 0)
        .pid(0.0, 0.0, 0.0, 1)
        .feedforward(0.0, 0.0, 0.0, 0.0, 1)
        .gravityType(GravityTypeValue.Elevator_Static, 1)
        .staticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 1)
        .pid(0.0, 0.0, 0.0, 2)
        .feedforward(0.0, 0.0, 0.0, 0.0, 2)
        .gravityType(GravityTypeValue.Elevator_Static, 2)
        .staticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 2)
        .buildConfig();
  }
}

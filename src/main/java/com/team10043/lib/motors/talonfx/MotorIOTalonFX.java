package com.team10043.lib.motors.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.motors.MotorIO.MotorIOInputs;
import com.team10043.lib.motors.config.MotorConfig;
import com.team10043.lib.motors.config.TalonFXMotorConfig;
import com.team10043.lib.util.control.ControlGains.FeedforwardGains;
import com.team10043.lib.util.control.ControlGains.PIDFConfig;
import com.team10043.lib.util.control.ControlGains.PIDGains;
import com.team10043.lib.util.phoenix6.CTREUtil;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/**
 * TalonFX (Falcon 500 / Kraken X60) motor controller implementation of {@link MotorIO}.
 *
 * <p>This class provides a hardware abstraction layer over CTRE Phoenix 6 TalonFX controllers,
 * handling configuration application, closed-loop control modes, telemetry collection, and CAN bus
 * optimization.
 *
 * <p>It integrates with {@link MotorConfig} for configuration consistency and AdvantageKit-style
 * logging via {@link MotorIOInputs}.
 */
public abstract class MotorIOTalonFX implements MotorIO {

  /** Underlying CTRE TalonFX hardware device. */
  protected final TalonFX motor;

  /** Motor and encoder configuration container. */
  protected final MotorConfig<TalonFXMotorConfig> config;

  /** Open-loop duty cycle control request. */
  protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

  /** Direct voltage control request. */
  protected final VoltageOut voltageControl = new VoltageOut(0.0);

  /** CAN follower control request. */
  protected final Follower followerControl = new Follower(0, MotorAlignmentValue.Aligned);

  /** All status signals used by this motor instance. */
  protected BaseStatusSignal[] signals;

  /**
   * Constructs a TalonFX motor IO instance and applies configuration.
   *
   * <p>This constructor:
   *
   * <ul>
   *   <li>Creates the TalonFX device
   *   <li>Applies CTRE configuration
   *   <li>Validates and synchronizes configuration ratios and limits
   * </ul>
   *
   * <p>Note: Subclasses must call {@link #initializeSignals()} after initializing their status
   * signals to complete the setup.
   *
   * @param config motor and encoder configuration
   */
  public MotorIOTalonFX(MotorConfig<TalonFXMotorConfig> config) {
    this.config = config;
    this.motor = new TalonFX(config.canId.getDeviceNumber(), config.canId.getCANBus());
    this.signals = null; // Will be initialized by subclass via getSignals()

    CTREUtil.applyConfiguration(motor, this.config.config);

    validateAndSyncConfig();
  }

  /**
   * Initializes CAN status signals, sets update frequencies, and optimizes bus utilization.
   *
   * <p>This method must be called by subclasses after they have initialized their status signal
   * fields. It retrieves the signals via {@link #getSignals()}, configures their update frequency,
   * optimizes CAN bus usage, and registers them for monitoring.
   */
  protected final void initializeSignals() {
    this.signals = getSignals();

    updateFrequency(config.updateFrequencyHz);

    CTREUtil.optimizeBusUtilization(motor);

    CTREUtil.registerSignals(config.canId.getBusName(), signals);
  }

  /**
   * Validates configuration values against the applied TalonFX configuration.
   *
   * <p>If mismatches are detected (ratios or soft limits), warnings are reported and the local
   * {@link MotorConfig} values are updated to reflect the hardware configuration.
   */
  private void validateAndSyncConfig() {
    if (config.kRotorToSensorRatio != config.config.Feedback.RotorToSensorRatio) {
      DriverStation.reportWarning(
          String.format(
              "TalonFX ID [%s] [%s]: RotorToSensorRatio mismatch. Expected=%.5f, Actual=%.5f. Updating kRotorToSensorRatio.",
              config.canId.getDeviceNumber(),
              config.canId.getBusName(),
              config.kRotorToSensorRatio,
              config.config.Feedback.RotorToSensorRatio),
          false);
      config.kRotorToSensorRatio = config.config.Feedback.RotorToSensorRatio;
    }

    if (config.kSensorToMechanismRatio != config.config.Feedback.SensorToMechanismRatio) {
      DriverStation.reportWarning(
          String.format(
              "TalonFX ID [%s] [%s]: SensorToMechanismRatio mismatch. Expected=%.5f, Actual=%.5f. Updating kSensorToMechanismRatio.",
              config.canId.getDeviceNumber(),
              config.canId.getBusName(),
              config.kSensorToMechanismRatio,
              config.config.Feedback.SensorToMechanismRatio),
          false);
      config.kSensorToMechanismRatio = config.config.Feedback.SensorToMechanismRatio;
    }

    if (config.kMaxPositionRots != config.config.SoftwareLimitSwitch.ForwardSoftLimitThreshold) {
      DriverStation.reportWarning(
          String.format(
              "TalonFX ID [%s] [%s]: ForwardSoftLimitThreshold mismatch. Expected=%.3f rot, Actual=%.3f rot. Updating kMaxPositionRots.",
              config.canId.getDeviceNumber(),
              config.canId.getBusName(),
              config.kMaxPositionRots,
              config.config.SoftwareLimitSwitch.ForwardSoftLimitThreshold),
          false);
      config.kMaxPositionRots = config.config.SoftwareLimitSwitch.ForwardSoftLimitThreshold;
    }

    if (config.kMinPositionRots != config.config.SoftwareLimitSwitch.ReverseSoftLimitThreshold) {
      DriverStation.reportWarning(
          String.format(
              "TalonFX ID [%s] [%s]: ReverseSoftLimitThreshold mismatch. Expected=%.3f rot, Actual=%.3f rot. Updating kMinPositionRots.",
              config.canId.getDeviceNumber(),
              config.canId.getBusName(),
              config.kMinPositionRots,
              config.config.SoftwareLimitSwitch.ReverseSoftLimitThreshold),
          false);
      config.kMinPositionRots = config.config.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
    }
  }

  @Override
  public boolean isConnected() {
    return BaseStatusSignal.isAllGood(signals);
  }

  public abstract BaseStatusSignal[] getSignals();

  @Override
  public abstract void updateInputs(MotorIOInputs inputs);

  @Override
  public void setVoltageOutput(double voltage) {
    motor.setControl(voltageControl.withOutput(voltage).withEnableFOC(config.usePhoenixPro));
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    motor.setControl(dutyCycleControl.withOutput(dutyCycle).withEnableFOC(config.usePhoenixPro));
  }

  @Override
  public abstract void setPositionSetpoint(double position, int slot);

  @Override
  public abstract void setVelocitySetpoint(double velocity, int slot);

  @Override
  public abstract void setMotionMagicSetpoint(double position, int slot);

  @Override
  public abstract void setMotionMagicSetpoint(double position, double feedforward, int slot);

  @Override
  public final void setBrakeMode(NeutralMode mode) {
    NeutralModeValue talonMode =
        mode == NeutralMode.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    config.config.MotorOutput.NeutralMode = talonMode;
    CTREUtil.applyConfiguration(motor, config.config);
  }

  @Override
  public final void setCurrentPosition(double position) {
    motor.setPosition(position);
  }

  @Override
  public final void follow(CANDeviceId masterId, boolean opposeMasterDirection) {
    CTREUtil.tryUntilOK(
        () ->
            motor.setControl(
                followerControl
                    .withLeaderID(masterId.getDeviceNumber())
                    .withMotorAlignment(
                        opposeMasterDirection
                            ? MotorAlignmentValue.Opposed
                            : MotorAlignmentValue.Aligned)),
        this.config.canId.getDeviceNumber(),
        "Follow Master");
  }

  @Override
  public final void setPIDF(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double kCos,
      double kCosRatio,
      int slot) {
    config.config.Slot0.kP = kP;
    config.config.Slot0.kI = kI;
    config.config.Slot0.kD = kD;

    config.config.Slot0.kS = kS;
    config.config.Slot0.kV = kV;
    config.config.Slot0.kA = kA;
    config.config.Slot0.kG = kG;

    CTREUtil.applyConfiguration(motor, config.config);
  }

  @Override
  public final PIDFConfig getPIDF(int slot) {
    return switch (slot) {
      default -> new PIDFConfig(
          new PIDGains(config.config.Slot0.kP, config.config.Slot0.kI, config.config.Slot0.kD),
          new FeedforwardGains(
              config.config.Slot0.kS,
              config.config.Slot0.kV,
              config.config.Slot0.kA,
              config.config.Slot0.kG != 0 ? Optional.of(config.config.Slot0.kG) : Optional.empty(),
              Optional.empty(),
              Optional.empty()));
      case 1 -> new PIDFConfig(
          new PIDGains(config.config.Slot1.kP, config.config.Slot1.kI, config.config.Slot1.kD),
          new FeedforwardGains(
              config.config.Slot1.kS,
              config.config.Slot1.kV,
              config.config.Slot1.kA,
              config.config.Slot1.kG != 0 ? Optional.of(config.config.Slot1.kG) : Optional.empty(),
              Optional.empty(),
              Optional.empty()));
      case 2 -> new PIDFConfig(
          new PIDGains(config.config.Slot2.kP, config.config.Slot2.kI, config.config.Slot2.kD),
          new FeedforwardGains(
              config.config.Slot2.kS,
              config.config.Slot2.kV,
              config.config.Slot2.kA,
              config.config.Slot2.kG != 0 ? Optional.of(config.config.Slot2.kG) : Optional.empty(),
              Optional.empty(),
              Optional.empty()));
    };
  }

  @Override
  public final void updateFrequency(double frequencyHz) {
    CTREUtil.tryUntilOK(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(frequencyHz, signals),
        motor.getDeviceID(),
        "Setting status signal update frequency");
  }
}

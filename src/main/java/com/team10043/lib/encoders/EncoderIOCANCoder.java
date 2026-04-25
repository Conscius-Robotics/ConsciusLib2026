package com.team10043.lib.encoders;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.encoders.config.CANCoderConfig;
import com.team10043.lib.encoders.config.EncoderConfig;
import com.team10043.lib.util.phoenix6.CTREUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * {@link EncoderIO} implementation for a CTRE CANcoder device.
 *
 * <p>This class adapts Phoenix 6 CANcoder signals to the generic encoder IO interface, providing
 * position, absolute position, and velocity measurements in mechanism units.
 *
 * <p>Signals are registered for synchronized refresh via {@link CTREUtil}, enabling consistent
 * sampling across multiple devices.
 */
public class EncoderIOCANCoder implements EncoderIO {

  /** Startup delay used to ensure stable signal validity. */
  private static final double STARTUP_TIME_SECONDS = 0.5;

  /** Number of update cycles required before the encoder is considered ready. */
  private final int startupCycles;

  /** Underlying CANcoder hardware device. */
  protected final CANcoder canCoder;

  /** Encoder configuration describing hardware and scaling behavior. */
  protected final EncoderConfig<CANCoderConfig> config;

  /** Relative position signal. */
  private final StatusSignal<Angle> positionSignal;

  /** Absolute position signal. */
  private final StatusSignal<Angle> absolutePositionSignal;

  /** Velocity signal. */
  private final StatusSignal<AngularVelocity> velocitySignal;

  /** All signals associated with this encoder, used for synchronized updates. */
  private final BaseStatusSignal[] signals;

  private int startupCounter = 0;
  private boolean initialized = false;

  /**
   * Creates a CANcoder-based encoder IO implementation.
   *
   * <p>The CANcoder is configured, its status signals are initialized, and all signals are
   * registered for synchronized refresh on the appropriate CAN bus.
   *
   * @param config encoder configuration including CAN ID, scaling, and CANcoder-specific settings
   * @throws IllegalArgumentException if canId is not present (CANcoder requires CAN device ID)
   */
  public EncoderIOCANCoder(EncoderConfig<CANCoderConfig> config) {
    this.config = config;

    // CANcoder requires a CAN device ID
    CANDeviceId canDeviceId =
        config.canId.orElseThrow(
            () ->
                new IllegalArgumentException(
                    "CANcoder encoder requires a CAN device ID. Set canId in EncoderConfig."));

    this.canCoder = new CANcoder(canDeviceId.getDeviceNumber(), canDeviceId.getCANBus());

    CTREUtil.applyConfiguration(canCoder, config.config);

    positionSignal = canCoder.getPosition();
    absolutePositionSignal = canCoder.getAbsolutePosition();
    velocitySignal = canCoder.getVelocity();

    signals = new BaseStatusSignal[] {positionSignal, absolutePositionSignal, velocitySignal};

    updateFrequency(config.updateFrequencyHz);
    startupCycles = (int) (config.updateFrequencyHz * STARTUP_TIME_SECONDS);

    CTREUtil.optimizeBusUtilization(canCoder);

    CTREUtil.registerSignals(canDeviceId.getBusName(), signals);
  }

  /**
   * Reads encoder data into the provided input container.
   *
   * <p>Data is only populated once the encoder has passed an initialization period during which all
   * signals report valid values.
   *
   * @param inputs container to populate with encoder measurements
   */
  @Override
  public void updateInputs(EncoderIOInputs inputs) {
    if (!isInitialized()) {
      return;
    }

    readSignals(inputs);
  }

  @Override
  public boolean isConnected() {
    return BaseStatusSignal.isAllGood(signals);
  }

  /**
   * Determines whether the encoder has completed its startup validation period.
   *
   * <p>The encoder is considered initialized once all signals report valid data for a sufficient
   * number of consecutive update cycles.
   *
   * @return true if the encoder is ready for use
   */
  private boolean isInitialized() {
    if (initialized) {
      return true;
    }

    if (BaseStatusSignal.isAllGood(signals)) {
      startupCounter++;
    }

    if (startupCounter >= startupCycles) {
      initialized = true;
    }

    return initialized;
  }

  /**
   * Reads scaled signal values into the input container.
   *
   * <p>All values are converted from sensor units to mechanism units using {@link
   * EncoderConfig#sensorToMechanismRatio}.
   *
   * @param inputs container to populate
   */
  private void readSignals(EncoderIOInputs inputs) {
    double ratio = config.sensorToMechanismRatio;

    inputs.data =
        new EncoderIOData(
            isConnected(),
            positionSignal.getValueAsDouble() * ratio,
            absolutePositionSignal.getValueAsDouble() * ratio,
            velocitySignal.getValueAsDouble() * ratio);
  }

  /**
   * Updates the CANcoder status signal update frequency.
   *
   * @param frequencyHz desired update rate in Hertz
   */
  @Override
  public void updateFrequency(double frequencyHz) {
    BaseStatusSignal.setUpdateFrequencyForAll(frequencyHz, signals);
  }
}

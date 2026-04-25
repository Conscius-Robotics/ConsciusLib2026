package com.team10043.lib.subsystems.motor.components.encoder;

import com.team10043.lib.encoders.EncoderIO;
import com.team10043.lib.encoders.EncoderIOInputsAutoLogged;
import com.team10043.lib.encoders.config.EncoderConfig;
import com.team10043.lib.encoders.config.EncoderConfiguration;
import com.team10043.lib.motors.MotorIO;
import com.team10043.lib.util.DebouncedAlert;
import org.littletonrobotics.junction.Logger;

/**
 * EncoderComponent handles encoder integration for a motor.
 *
 * <p>This component is intended for standalone encoders and should NOT be used with remote or
 * integrated sensors such as TalonFX internal encoders or CANcoders configured in remote mode.
 *
 * <p>Responsibilities:
 *
 * <ul>
 *   <li>Read and log encoder inputs
 *   <li>Apply absolute position as motor offset once
 *   <li>Expose encoder position and velocity
 * </ul>
 *
 * @param <T> Encoder configuration type
 */
public class ActiveEncoderComponent<T extends EncoderConfiguration> implements EncoderComponent {

  private final EncoderIO io;
  private final EncoderIOInputsAutoLogged inputs;
  private final EncoderConfig<T> config;
  private boolean hasSetOffset = false;

  private DebouncedAlert encoderDisconnectedAlert;

  private String systemName; // Store system name for logging purposes

  /**
   * Creates a new encoder component.
   *
   * @param inputs auto-logged encoder inputs
   * @param config encoder configuration
   */
  public ActiveEncoderComponent(EncoderConfig<T> config) {
    this.io = config.createIO();
    this.inputs = new EncoderIOInputsAutoLogged();
    this.config = config;

    // Format encoder identifier for alerts - handle optional CAN ID for DIO encoders
    String encoderIdentifier =
        config
            .canId
            .map(id -> String.format("CAN [%d] [%s]", id.getDeviceNumber(), id.getBusName()))
            .orElse("DIO Encoder");

    this.encoderDisconnectedAlert =
        new DebouncedAlert(
            () -> inputs.data.connected(),
            () -> true,
            String.format("Encoder %s disconnected.", encoderIdentifier));
  }

  @Override
  public void periodic(String systemName, MotorIO motorIO) {
    this.systemName = systemName;

    io.updateInputs(inputs);
    Logger.processInputs(systemName + "/encoder", inputs);
    Logger.recordOutput(systemName + "/encoder/hasSetOffset", hasSetOffset);

    encoderDisconnectedAlert.update();

    if (!hasSetOffset || config.updateMotorConstantly) {
      resetOffset(motorIO);
    }
  }

  @Override
  public void resetOffset(MotorIO motorIO) {
    double absolutePosition = inputs.data.absolutePositionRots();

    if (Double.isNaN(absolutePosition)) {
      Logger.recordOutput(systemName + "/encoder/offsetStatus", "Waiting for valid position (NaN)");
      return;
    }

    motorIO.setCurrentPosition(absolutePosition);
    hasSetOffset = true;
    Logger.recordOutput(systemName + "/encoder/offsetStatus", "Offset applied");
  }

  @Override
  public EncoderIOInputsAutoLogged getInputs() {
    return inputs;
  }

  @Override
  public boolean hasSetOffset() {
    return hasSetOffset;
  }

  @Override
  public double getAbsolutePosition() {
    return inputs.data.absolutePositionRots();
  }

  @Override
  public double getVelocity() {
    return inputs.data.velocityRotPerSec();
  }
}

package com.team10043.lib.motors;

import com.team10043.lib.drivers.CANDeviceId;
import com.team10043.lib.util.control.ControlGains.PIDFConfig;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for motor controllers.
 *
 * <p>This interface defines a vendor-agnostic API for controlling motors and reading telemetry. It
 * is designed to support simulation, replay logging, and multiple motor controller implementations
 * (e.g. TalonFX, Spark MAX).
 */
public interface MotorIO {

  @RequiredArgsConstructor
  @Getter
  enum NeutralMode {
    BRAKE,
    COAST;
  }

  /**
   * Logged motor input container.
   *
   * <p>Used by AdvantageKit to capture motor telemetry for logging and replay.
   */
  @AutoLog
  public class MotorIOInputs {

    /** Latest motor sensor and electrical data. */
    public MotorIOData data = new MotorIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Immutable snapshot of motor telemetry.
   *
   * @param connected whether the motor controller is connected over CAN
   * @param velocityRotPerSec motor velocity in rotations per second
   * @param positionRots mechanism position in rotations
   * @param rawRotorPositionRots raw rotor position in rotations
   * @param appliedVolts applied motor voltage
   * @param torqueCurrentAmps motor torque current draw
   * @param statorCurrentAmps stator current draw
   * @param supplyCurrentAmps supply current draw
   */
  public record MotorIOData(
      boolean connected,
      double velocityRotPerSec,
      double positionRots,
      double rawRotorPositionRots,
      double appliedVolts,
      double torqueCurrentAmps,
      double statorCurrentAmps,
      double supplyCurrentAmps) {

    public static MotorIOData disconnected() {
      return new MotorIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
  }

  default void updateInputs(MotorIOInputs inputs) {}

  /**
   * Checks whether the canbus is connected.
   *
   * @return true if connected, false otherwise
   */
  default boolean isConnected() {
    return false;
  }

  /**
   * Sets a direct voltage output to the motor.
   *
   * @param voltage output voltage
   */
  default void setVoltageOutput(double voltage) {}

  /**
   * Sets an open-loop duty cycle output.
   *
   * @param dutyCycle output duty cycle (-1.0 to 1.0)
   */
  default void setOpenLoopDutyCycle(double dutyCycle) {}

  /**
   * Sets a direct torque current output to the motor.
   *
   * @param current output current in Amperes
   */
  default void setTorqueCurrentOutput(double current) {
    throw new UnsupportedOperationException(
        "Torque current control is not supported by this motor controller.");
  }

  /**
   * Sets a closed-loop position setpoint using the default control slot.
   *
   * @param position target position in rotations
   */
  default void setPositionSetpoint(double position) {
    setPositionSetpoint(position, 0);
  }

  /**
   * Sets a closed-loop position setpoint.
   *
   * @param position target position in rotations
   * @param slot PID slot index
   */
  default void setPositionSetpoint(double position, int slot) {}

  /**
   * Sets a closed-loop velocity setpoint using the default control slot.
   *
   * @param velocity target velocity in rotations per second
   */
  default void setVelocitySetpoint(double velocity) {
    setVelocitySetpoint(velocity, 0);
  }

  /**
   * Sets a closed-loop velocity setpoint.
   *
   * @param velocity target velocity in rotations per second
   * @param slot PID slot index
   */
  default void setVelocitySetpoint(double velocity, int slot) {}

  /**
   * Sets a Motion Magic position setpoint using the default control slot.
   *
   * @param position target position in rotations
   */
  default void setMotionMagicSetpoint(double position) {
    setMotionMagicSetpoint(position, 0);
  }

  /**
   * Sets a Motion Magic position setpoint.
   *
   * @param position target position in rotations
   * @param slot PID slot index
   */
  default void setMotionMagicSetpoint(double position, int slot) {}

  /**
   * Sets a Motion Magic position setpoint with feedforward using the default control slot.
   *
   * @param position target position in rotations
   * @param feedforward feedforward output
   */
  default void setMotionMagicSetpoint(double position, double feedforward) {
    setMotionMagicSetpoint(position, feedforward, 0);
  }

  /**
   * Sets a Motion Magic position setpoint with feedforward.
   *
   * @param position target position in rotations
   * @param feedforward feedforward output
   * @param slot PID slot index
   */
  default void setMotionMagicSetpoint(double position, double feedforward, int slot) {}

  /**
   * Sets the motor neutral mode.
   *
   * @param mode brake or coast
   */
  default void setBrakeMode(NeutralMode mode) {}

  /** Sets the current motor position to zero. */
  default void setCurrentPositionAsZero() {
    setCurrentPosition(0.0);
  }

  /**
   * Sets the current motor position.
   *
   * @param position position in rotations
   */
  default void setCurrentPosition(double position) {}

  /**
   * Configures this motor controller to follow another motor over CAN.
   *
   * <p>When {@code opposeMasterDirection} is set to {@code true}, the follower motor output is
   * inverted relative to the master motor (i.e., it spins in the opposite direction).
   *
   * @param masterId CAN ID of the master motor
   * @param opposeMasterDirection whether to invert the follower motor output relative to the master
   */
  default void follow(CANDeviceId masterId, boolean opposeMasterDirection) {}

  /**
   * Sets the motor controller's PIDF tuning parameters.
   *
   * <p><b>Gravity feedforward rules:</b>
   *
   * <ul>
   *   <li>{@code kG} (static gravity feedforward) and {@code kCos} (cosine gravity feedforward) are
   *       <b>mutually exclusive</b>.
   *   <li>If both {@code kG} and {@code kCos} are provided as non-zero values, {@code kG} will be
   *       <b>automatically forced to 0</b> and a DriverStation warning/error will be reported.
   *   <li>Use {@code kG} for <b>linear mechanisms</b> such as elevators.
   *   <li>Use {@code kCos} together with {@code kCosRatio} for <b>rotary/arm mechanisms</b>.
   * </ul>
   *
   * <p><b>Hardware-specific notes:</b>
   *
   * <ul>
   *   <li>{@code kCos} and {@code kCosRatio} are <b>Spark MAX specific features</b> and are ignored
   *       or unsupported by other motor controllers.
   *   <li>{@code kG} is supported across motor controllers that implement static gravity
   *       feedforward.
   * </ul>
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kS static feedforward gain
   * @param kV velocity feedforward gain
   * @param kA acceleration feedforward gain
   * @param kG static gravity feedforward gain (linear mechanisms)
   * @param kCos cosine gravity feedforward gain (Spark MAX only, arm mechanisms)
   * @param kCosRatio cosine ratio used to convert position to absolute rotations for {@code kCos}
   *     (Spark MAX only)
   * @param slot PID slot index
   */
  default void setPIDF(PIDFConfig config, int slot) {
    setPIDF(
        config.kP(),
        config.kI(),
        config.kD(),
        config.kS(),
        config.kV(),
        config.kA(),
        config.kG().orElse(0.0),
        config.ff().kCos().orElse(0.0),
        config.ff().kCosRatio().orElse(0.0),
        slot);
  }

  default void setPIDF(PIDFConfig config) {
    setPIDF(config, 0);
  }

  @SuppressWarnings("java:S107")
  default void setPIDF(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double kCos,
      double kCosRatio,
      int slot) {}

  /** Retrieves the motor controller's current PIDF tuning parameters. */
  default PIDFConfig getPIDF(int slot) {
    return PIDFConfig.zero();
  }

  /**
   * Updates the motor controller's status signal update frequency.
   *
   * @param frequencyHz desired update rate in Hertz
   */
  default void updateFrequency(double frequencyHz) {}
}

package com.team10043.lib.subsystems.swerve.module.motors;

import com.team10043.lib.subsystems.swerve.module.ModuleIO.ModuleIOInputs;
import edu.wpi.first.math.geometry.Rotation2d;

public interface TurnMotorIO {

  /**
   * Data structure for logging turn motor inputs
   *
   * @param connected Whether the turn motor is connected
   * @param encoderConnected Whether the turn encoder is connected
   * @param absolutePosition The absolute position of the turn motor
   * @param position The current position of the turn motor
   * @param velocityRadPerSec The current velocity of the turn motor in radians per second
   * @param appliedVolts The voltage applied to the turn motor
   * @param supplyCurrentAmps The supply current to the turn motor in amps
   * @param statorCurrentAmps The stator current of the turn motor in amps
   * @param torqueCurrentAmps The torque current of the turn motor in amps
   */
  public record ModuleIOTurnData(
      boolean connected,
      boolean encoderConnected,
      Rotation2d absolutePosition,
      Rotation2d position,
      double velocityRadPerSec,
      double appliedVolts,
      double supplyCurrentAmps,
      double statorCurrentAmps,
      double torqueCurrentAmps) {}

  /**
   * Update the inputs data structure with the latest readings from the hardware
   *
   * @param inputs The ModuleIOInputs object to update
   */
  void updateInputs(ModuleIOInputs inputs);

  /**
   * Runs the turn motor in open-loop control mode with the specified output.
   *
   * @param output the output value (e.g., voltage percentage) to apply to the turn motor
   */
  void runOpenLoop(double output);

  /**
   * Runs the turn motor in closed-loop position control mode with the specified target position.
   *
   * @param rotation the target position as a Rotation2d for the turn motor
   */
  void runPosition(Rotation2d rotation);

  /**
   * Configures the PID and feedforward parameters for the turn motor controller.
   *
   * @param kP the proportional gain
   * @param kI the integral gain
   * @param kD the derivative gain
   * @param kS the static feedforward (volts)
   * @param kV the velocity feedforward (volts per radian/sec)
   * @param kA the acceleration feedforward (volts per radian/sec²)
   */
  void setPIDFF(double kP, double kI, double kD, double kS, double kV, double kA);

  /** Sets the brake mode of the turn motor. */
  void setBrakeMode(boolean enabled);
}

package com.team10043.lib.subsystems.swerve.module.motors;

import com.team10043.lib.subsystems.swerve.module.ModuleIO.ModuleIOInputs;

public interface DriveMotorIO {

  /**
   * Data structure for logging drive motor inputs
   *
   * @param connected Whether the drive motor is connected
   * @param positionRad The current position of the drive motor in radians
   * @param velocityRadPerSec The current velocity of the drive motor in radians per second
   * @param appliedVolts The voltage applied to the drive motor
   * @param supplyCurrentAmps The supply current to the drive motor in amps
   * @param statorCurrentAmps The stator current of the drive motor in amps
   * @param torqueCurrentAmps The torque current of the drive motor in amps
   */
  public record ModuleIODriveData(
      boolean connected,
      double positionRad,
      double velocityRadPerSec,
      double appliedVolts,
      double supplyCurrentAmps,
      double statorCurrentAmps,
      double torqueCurrentAmps) {}
  /**
   * Updates the input data structure with the latest sensor readings.
   *
   * @param inputs the ModuleIOInputs object to populate with current sensor data
   */
  void updateInputs(ModuleIOInputs inputs);

  /**
   * Runs the drive motor in open-loop control mode with the specified output.
   *
   * @param output the output value (e.g., voltage percentage) to apply to the drive motor
   */
  void runOpenLoop(double output);

  /**
   * Runs the drive motor in closed-loop velocity control mode with the specified target velocity.
   *
   * @param velocityRadPerSec the target velocity in radians per second for the drive motor
   */
  void runVelocity(double velocityRadPerSec);

  /**
   * Configures the PIDF parameters for the drive motor controller.
   *
   * @param kP the proportional gain
   * @param kI the integral gain
   * @param kD the derivative gain
   * @param kS the static feedforward gain
   * @param kV the velocity feedforward gain
   * @param kA the acceleration feedforward gain
   */
  void setPIDF(double kP, double kI, double kD, double kS, double kV, double kA);

  /** Sets the brake mode of the drive motor. */
  void setBrakeMode(boolean enabled);
}

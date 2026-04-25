package com.team10043.lib.subsystems.swerve.module;

import com.team10043.lib.subsystems.swerve.module.motors.DriveMotorIO.ModuleIODriveData;
import com.team10043.lib.subsystems.swerve.module.motors.TurnMotorIO.ModuleIOTurnData;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction layer for a single swerve module. Defines the interface for controlling
 * drive and turn motors independently, supporting both open-loop and closed-loop control modes.
 *
 * <p>Implementations handle vendor-specific motor controller communication (hardware) or simulated
 * physics (sim).
 */
public interface ModuleIO {
  /**
   * Container for all sensor inputs from a swerve module. Includes drive motor state, turn motor
   * state, and odometry samples.
   */
  @AutoLog
  public static class ModuleIOInputs {
    public ModuleIODriveData driveData = new ModuleIODriveData(false, 0, 0, 0, 0, 0, 0);

    public ModuleIOTurnData turnData =
        new ModuleIOTurnData(false, false, Rotation2d.kZero, Rotation2d.kZero, 0, 0, 0, 0, 0);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs Container to populate with current sensor readings
   */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Run the drive motor at the specified open loop value.
   *
   * @param output Voltage output (-12 to 12 V) or duty cycle (-1 to 1), depending on implementation
   */
  public default void runDriveOpenLoop(double output) {}

  /**
   * Run the turn motor at the specified open loop value.
   *
   * @param output Voltage output (-12 to 12 V) or duty cycle (-1 to 1), depending on implementation
   */
  public default void runTurnOpenLoop(double output) {}

  /**
   * Run the drive motor at the specified velocity using closed-loop control.
   *
   * @param velocityRadPerSec Desired velocity in radians per second
   */
  public default void runDriveVelocity(double velocityRadPerSec) {}

  /**
   * Run the turn motor to the specified rotation using closed-loop control.
   *
   * @param rotation Desired rotation angle
   */
  public default void runTurnPosition(Rotation2d rotation) {}

  /**
   * Set PID and feedforward gains for closed-loop control on drive motor.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kS Static friction feedforward
   * @param kV Velocity feedforward
   * @param kA Acceleration feedforward
   */
  public default void setDrivePIDFF(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

  /**
   * Set PID and feedforward gains for closed-loop control on turn motor.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kS Static friction feedforward
   * @param kV Velocity feedforward
   * @param kA Acceleration feedforward
   */
  public default void setTurnPIDFF(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

  /**
   * Set brake mode on drive and turn motors.
   *
   * @param enabled true for brake mode, false for coast mode
   */
  public default void setBrakeMode(boolean enabled) {}
}

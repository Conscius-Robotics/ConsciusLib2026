package com.team10043.lib.subsystems.swerve.module;

import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig.ModulePosition;
import com.team10043.lib.subsystems.swerve.module.diagnostics.ModuleAlerts;
import com.team10043.lib.subsystems.swerve.module.diagnostics.ModuleTuning;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a single swerve module with independent drive and turn control. Handles state
 * optimization, odometry sample management, tuning, and fault detection.
 *
 * <p>The module supports both closed-loop velocity control (PID) and open-loop voltage control for
 * the drive motor. The turn motor always uses closed-loop position control.
 */
public class Module {

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final ModuleTuning tuning;
  private final ModuleAlerts alerts;

  private final double wheelRadiusMeters;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /**
   * Constructs a new swerve module.
   *
   * @param config Swerve configuration containing chassis dimensions and module settings
   * @param io Hardware abstraction layer for motor and sensor control
   * @param index Module index (0-3 for FL, FR, BL, BR)
   * @param position Module position enum for config lookup
   */
  public Module(SwerveConfig<?> config, ModuleIO io, int index, ModulePosition position) {
    this.io = io;
    this.index = index;
    this.wheelRadiusMeters = config.chassisDimensions().wheelRadius();

    this.tuning = new ModuleTuning(io, config.moduleConfig(position).tuning());
    this.alerts = new ModuleAlerts(inputs, index);
  }

  /** Updates sensor inputs from hardware and logs them via AdvantageKit. */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);
  }

  /**
   * Converts raw odometry samples to {@link SwerveModulePosition} objects. Multiplies drive
   * positions (radians) by wheel radius to get meters.
   */
  private void updateOdometryPositions() {
    int sampleCount = inputs.odometryDrivePositionsRad.length;
    odometryPositions = new SwerveModulePosition[sampleCount];

    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /**
   * Updates inputs, odometry positions, tuning parameters, and fault alerts. Call this method once
   * per loop iteration.
   */
  public void periodic() {
    updateInputs();
    updateOdometryPositions();
    tuning.update();
    alerts.update();
  }

  /**
   * Runs the module with the specified setpoint state using closed-loop velocity control.
   *
   * @param state The desired module state with speed in m/s
   */
  public void runSetpoint(SwerveModuleState state) {
    SwerveModuleState optimized = optimizeState(state);

    io.runDriveVelocity(optimized.speedMetersPerSecond / wheelRadiusMeters);
    io.runTurnPosition(optimized.angle);
  }

  /**
   * Runs the module with the specified setpoint state using open-loop drive control. Drive motor
   * runs at voltage output (open loop), turn motor uses PID control.
   *
   * @param state The desired module state with speed as voltage output (-12 to 12)
   */
  public void runOpenLoopSetpoint(SwerveModuleState state) {
    SwerveModuleState optimized = optimizeState(state);

    io.runDriveOpenLoop(optimized.speedMetersPerSecond);
    io.runTurnPosition(optimized.angle);
  }

  /**
   * Optimizes the module state by finding the shortest rotation path and applying cosine scaling.
   * Returns a new optimized state without mutating the input.
   *
   * @param state The desired module state
   * @return A new optimized module state
   */
  public SwerveModuleState optimizeState(SwerveModuleState state) {
    SwerveModuleState optimized = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
    optimized.optimize(getAngle());
    optimized.cosineScale(inputs.turnData.position());
    return optimized;
  }

  /**
   * Runs the module for drive characterization. Drive motor uses voltage output, turn motor holds
   * zero degrees.
   *
   * @param output Voltage output for the drive motor
   */
  public void runDriveCharacterization(double output) {
    io.runDriveOpenLoop(output);
    io.runTurnPosition(Rotation2d.kZero);
  }

  /**
   * Runs the module for turn characterization. Turn motor uses voltage output, drive motor is
   * stopped.
   *
   * @param output Voltage output for the turn motor
   */
  public void runTurnCharacterization(double output) {
    io.runDriveOpenLoop(0.0);
    io.runTurnOpenLoop(output);
  }

  /** Returns the turn motor velocity in radians/sec for characterization. */
  public double getTurnVelocity() {
    return inputs.turnData.velocityRadPerSec();
  }

  /** Returns the turn motor torque current in amps for characterization. */
  public double getTurnTorqueCurrent() {
    return inputs.turnData.torqueCurrentAmps();
  }

  /** Returns the drive motor torque current in amps for characterization. */
  public double getDriveTorqueCurrent() {
    return inputs.driveData.torqueCurrentAmps();
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.runDriveOpenLoop(0.0);
    io.runTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    double radians = inputs.turnData.position().getRadians();
    double normalized = MathUtil.angleModulus(radians);
    return new Rotation2d(normalized);
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.driveData.positionRad() * wheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveData.velocityRadPerSec() * wheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.driveData.positionRad();
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveData.velocityRadPerSec());
  }

  /**
   * Sets brake mode for both drive and turn motors.
   *
   * @param enabled true for brake mode, false for coast mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }
}

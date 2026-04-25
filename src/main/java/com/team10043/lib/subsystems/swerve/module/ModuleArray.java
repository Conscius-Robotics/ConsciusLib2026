package com.team10043.lib.subsystems.swerve.module;

import com.team10043.lib.subsystems.swerve.config.SwerveConfig;
import com.team10043.lib.subsystems.swerve.config.SwerveConfig.ModulePosition;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Manages the four swerve modules as a group. Provides convenient methods for operations on all
 * modules.
 */
public class ModuleArray {
  private static final int MODULE_COUNT = 4;
  private static final int FL = 0;
  private static final int FR = 1;
  private static final int BL = 2;
  private static final int BR = 3;

  private final Module[] modules;

  public ModuleArray(
      SwerveConfig<?> config,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    modules = new Module[MODULE_COUNT];
    modules[FL] = new Module(config, flModuleIO, FL, ModulePosition.FRONT_LEFT);
    modules[FR] = new Module(config, frModuleIO, FR, ModulePosition.FRONT_RIGHT);
    modules[BL] = new Module(config, blModuleIO, BL, ModulePosition.BACK_LEFT);
    modules[BR] = new Module(config, brModuleIO, BR, ModulePosition.BACK_RIGHT);
  }

  /** Updates all modules periodically. */
  public void periodic() {
    for (Module module : modules) {
      module.periodic();
    }
  }

  /** Stops all modules. */
  public void stop() {
    for (Module module : modules) {
      module.stop();
    }
  }

  /** Runs all modules with the given setpoint states. */
  public void runSetpoints(SwerveModuleState[] states) {
    for (int i = 0; i < MODULE_COUNT; i++) {
      modules[i].runSetpoint(states[i]);
    }
  }

  /**
   * Runs all modules with the given setpoint states using open-loop drive control.
   *
   * <p>Drive motors apply open-loop commands (voltage or FOC torque current depending on
   * configuration), while turn motors hold steering via position control.
   */
  public void runOpenLoopSetpoints(SwerveModuleState[] states) {
    for (int i = 0; i < MODULE_COUNT; i++) {
      modules[i].runOpenLoopSetpoint(states[i]);
    }
  }

  /** Runs drive characterization on all modules with the given output. */
  public void runDriveCharacterization(double output) {
    for (int i = 0; i < MODULE_COUNT; i++) {
      modules[i].runDriveCharacterization(output);
    }
  }

  /** Runs turn characterization on all modules with the given output. */
  public void runTurnCharacterization(double output) {
    for (int i = 0; i < MODULE_COUNT; i++) {
      modules[i].runTurnCharacterization(output);
    }
  }

  /** Gets the current state of all modules. */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[MODULE_COUNT];
    for (int i = 0; i < MODULE_COUNT; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Gets the current position of all modules. */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[MODULE_COUNT];
    for (int i = 0; i < MODULE_COUNT; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Gets odometry timestamps from the first module. All modules are sampled together, so we only
   * need timestamps from one.
   */
  public double[] getOdometryTimestamps() {
    return modules[0].getOdometryTimestamps();
  }

  /**
   * Gets all odometry position samples from all modules. Returns a 2D array:
   * [sampleIndex][moduleIndex]
   */
  public SwerveModulePosition[][] getOdometryPositionSamples() {
    double[] timestamps = getOdometryTimestamps();
    int sampleCount = timestamps.length;

    SwerveModulePosition[][] samples = new SwerveModulePosition[sampleCount][MODULE_COUNT];

    for (int sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++) {
      for (int moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
        samples[sampleIndex][moduleIndex] =
            modules[moduleIndex].getOdometryPositions()[sampleIndex];
      }
    }

    return samples;
  }

  /** Gets wheel radius characterization positions for all modules. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] positions = new double[MODULE_COUNT];
    for (int i = 0; i < MODULE_COUNT; i++) {
      positions[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return positions;
  }

  /** Gets the average feedforward characterization velocity across all modules. */
  public double getAverageFFCharacterizationVelocity() {
    double sum = 0.0;
    for (int i = 0; i < MODULE_COUNT; i++) {
      sum += modules[i].getFFCharacterizationVelocity();
    }
    return sum / MODULE_COUNT;
  }

  /** Gets the average turn characterization velocity across all modules (rad/s). */
  public double getAverageTurnVelocity() {
    double sum = 0.0;
    for (int i = 0; i < MODULE_COUNT; i++) {
      sum += modules[i].getTurnVelocity();
    }
    return sum / MODULE_COUNT;
  }

  /** Gets the average turn motor torque current across all modules (amps). */
  public double getAverageTurnTorqueCurrent() {
    double sum = 0.0;
    for (int i = 0; i < MODULE_COUNT; i++) {
      sum += modules[i].getTurnTorqueCurrent();
    }
    return sum / MODULE_COUNT;
  }

  /** Gets the average drive motor torque current across all modules (amps). */
  public double getAverageDriveTorqueCurrent() {
    double sum = 0.0;
    for (int i = 0; i < MODULE_COUNT; i++) {
      sum += modules[i].getDriveTorqueCurrent();
    }
    return sum / MODULE_COUNT;
  }
}

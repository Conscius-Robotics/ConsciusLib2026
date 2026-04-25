package com.team10043.lib.subsystems.swerve.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/** Simple logger for drive subsystem telemetry. Publishes all data to AdvantageKit Logger. */
public class SwerveLogger {
  private static final String ODOMETRY_PREFIX = "Odometry/";
  private static final String DRIVE_PREFIX = "Swerve/";
  private static final String SWERVE_STATES_PREFIX = DRIVE_PREFIX + "SwerveStates/";
  private static final String CHASSIS_SPEEDS_PREFIX = DRIVE_PREFIX + "SwerveChassisSpeeds/";

  private final SwerveDriveKinematics kinematics;

  /**
   * Record for logging open-loop module setpoints. Contains voltage command and target angle.
   *
   * @param voltageVolts Drive motor voltage command in volts
   * @param angle Target turn angle
   */
  public record OpenLoopModuleSetpoint(double voltageVolts, Rotation2d angle) {}

  public SwerveLogger(SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;
  }

  /** Logs setpoint states before and after optimization. */
  public void logSetpoints(
      SwerveModuleState[] unoptimized, SwerveModuleState[] optimized, ChassisSpeeds speeds) {
    Logger.recordOutput(SWERVE_STATES_PREFIX + "Setpoints", unoptimized);
    Logger.recordOutput(SWERVE_STATES_PREFIX + "SetpointsOptimized", optimized);
    Logger.recordOutput(CHASSIS_SPEEDS_PREFIX + "Setpoints", speeds);
  }

  /**
   * Logs setpoint states for open-loop control mode. Logs duty cycle values and target angles.
   *
   * @param states Array of module states
   */
  public void logOpenLoopSetpoints(SwerveModuleState[] states) {
    OpenLoopModuleSetpoint[] openLoopSetpoints = new OpenLoopModuleSetpoint[states.length];
    for (int i = 0; i < states.length; i++) {
      openLoopSetpoints[i] =
          new OpenLoopModuleSetpoint(states[i].speedMetersPerSecond, states[i].angle);
    }

    Logger.recordOutput(SWERVE_STATES_PREFIX + "OpenLoopSetpoints", openLoopSetpoints);
  }

  /** Logs empty setpoint states (used when disabled). */
  public void logEmptySetpoints() {
    SwerveModuleState[] empty = new SwerveModuleState[] {};
    OpenLoopModuleSetpoint[] emptyOpenLoop = new OpenLoopModuleSetpoint[] {};

    Logger.recordOutput(SWERVE_STATES_PREFIX + "Setpoints", empty);
    Logger.recordOutput(SWERVE_STATES_PREFIX + "SetpointsOptimized", empty);
    Logger.recordOutput(SWERVE_STATES_PREFIX + "OpenLoopSetpoints", emptyOpenLoop);
  }

  /** Logs measured module states and chassis speeds. */
  public void logMeasuredStates(SwerveModuleState[] states) {
    Logger.recordOutput(SWERVE_STATES_PREFIX + "Measured", states);

    ChassisSpeeds measuredSpeeds = kinematics.toChassisSpeeds(states);
    Logger.recordOutput(CHASSIS_SPEEDS_PREFIX + "Measured", measuredSpeeds);
  }

  /** Logs the current robot pose. */
  public void logPose(Pose2d pose) {
    Logger.recordOutput(ODOMETRY_PREFIX + "Robot", pose);
  }

  /** Logs the current drive control mode. */
  public void logControlMode(String mode) {
    Logger.recordOutput(DRIVE_PREFIX + "ControlMode", mode);
  }

  /** Logs gyro connection status. */
  public void logGyroStatus(boolean connected) {
    Logger.recordOutput(DRIVE_PREFIX + "GyroConnected", connected);
  }

  /** Logs SysId state. */
  public void logSysIdState(String state) {
    Logger.recordOutput(DRIVE_PREFIX + "SysIdState", state);
  }
}

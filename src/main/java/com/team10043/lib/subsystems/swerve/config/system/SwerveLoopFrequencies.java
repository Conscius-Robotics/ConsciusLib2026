package com.team10043.lib.subsystems.swerve.config.system;

/**
 * Immutable holder for the periodic update frequencies used by a swerve-drive subsystem.
 *
 * <p>This record groups two distinct loop frequencies (in Hertz, cycles per second) that are
 * commonly required when configuring swerve-drive control and state-estimation code:
 *
 * <ul>
 *   <li><b>odometryHz</b> — The update rate at which sensors (encoders, gyro, vision, etc.) are
 *       sampled and the pose estimator/odometry is integrated. A higher odometry frequency gives
 *       finer-grained pose updates but may increase CPU/communication load.
 *   <li><b>controlLoopHz</b> — The frequency at which the closed-loop drive controllers and motion
 *       control (trajectory following, kinematics) are executed. This typically drives the rate at
 *       which motor setpoints are computed and sent to actuators.
 * </ul>
 *
 * <p>Both values are expressed in Hertz (Hz) and are expected to be positive non-zero values.
 * Choosing appropriate values depends on your robot's sensors, control algorithms, and CPU limits:
 * lower frequencies reduce CPU usage but increase latency; higher frequencies reduce latency but
 * increase CPU and bus bandwidth usage.
 *
 * <p>This type is immutable and thread-safe.
 *
 * @param odometryHz frequency (Hz) for odometry / pose-estimation updates; must be > 0
 * @param controlLoopHz frequency (Hz) for the control loop / trajectory follower; must be > 0
 */
public record SwerveLoopFrequencies(double odometryHz, double controlLoopHz) {

  /**
   * Factory method to create a SwerveLoopFrequencies instance.
   *
   * @param odometryHz frequency (Hz) for odometry / pose-estimation updates; must be > 0
   * @param controlLoopHz frequency (Hz) for the control loop / trajectory follower; must be > 0
   * @return a new SwerveLoopFrequencies instance with the specified frequencies
   */
  public static SwerveLoopFrequencies of(double odometryHz, double controlLoopHz) {
    return new SwerveLoopFrequencies(odometryHz, controlLoopHz);
  }
}
